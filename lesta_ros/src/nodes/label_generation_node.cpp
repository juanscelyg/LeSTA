/*
 * label_generation_node.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta_ros/nodes/label_generation_node.h"
#include "lesta_ros/utils/config_loader.h"
#include "lesta_ros/utils/pc_utils.h"

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

namespace lesta_ros {

LabelGenerationNode::LabelGenerationNode() : nh_("~") {

  // ROS node
  LabelGenerationNode::loadConfig(nh_);
  initializeTimers();
  initializePubSubs();
  initializeServices();

  // Global Mapper
  ros::NodeHandle nh_mapper(nh_, "mapper");
  auto cfg1 = global_mapper::loadConfig(nh_mapper);
  mapper_ = std::make_unique<GlobalMapper>(cfg1);

  // Feature Extractor
  ros::NodeHandle nh_feature_extractor(nh_, "feature_extractor");
  auto cfg2 = feature_extractor::loadConfig(nh_feature_extractor);
  feature_extractor_ = std::make_unique<FeatureExtractor>(cfg2);

  // Transform object
  ros::NodeHandle nh_frame_id(nh_, "frame_id");
  frameID = TransformHandler::loadFrameIDs(nh_frame_id);

  std::cout << "\033[1;33m[lesta_ros::LabelGenerationNode]: "
               "Label generation node initialized. Waiting for scan inputs...\033[0m\n";
}

void LabelGenerationNode::loadConfig(const ros::NodeHandle &nh) {

  // Topic parameters
  cfg_.lidarscan_topic = nh.param<std::string>("lidar_topic", "/velodyne/points");

  // Timer parameters
  cfg_.map_publish_rate = nh.param<double>("map_publish_rate", 3.0);
  cfg_.robot_pose_update_rate = nh.param<double>("robot_pose_update_rate", 10.0);

  // Options
  cfg_.remove_backward_points = nh.param<bool>("remove_backward_points", true);
  cfg_.debug_mode = nh.param<bool>("debug_mode", false);
}

void LabelGenerationNode::initializeTimers() {

  auto robot_pose_update_duration = ros::Duration(1.0 / cfg_.robot_pose_update_rate);
  auto map_publish_duration = ros::Duration(1.0 / cfg_.map_publish_rate);

  robot_pose_update_timer_ =
      nh_.createTimer(robot_pose_update_duration, &LabelGenerationNode::updateRobotPose, this);
  map_publish_timer_ = nh_.createTimer(map_publish_duration, &LabelGenerationNode::publishLabelMap, this);
}

void LabelGenerationNode::initializePubSubs() {

  // Subscribers
  sub_lidarscan_ = nh_.subscribe(cfg_.lidarscan_topic, 1, &LabelGenerationNode::lidarScanCallback, this);

  // Publishers
  pub_labelmap_local_ = nh_.advertise<grid_map_msgs::GridMap>("labelmap_local", 1);
  pub_labelmap_global_ = nh_.advertise<sensor_msgs::PointCloud2>("labelmap_global", 1);
  pub_labelmap_region_ = nh_.advertise<visualization_msgs::Marker>("labelmap_region", 1);

  if (cfg_.debug_mode) {
    // TODO: Add debug publishers
  }
}

void LabelGenerationNode::initializeServices() {
  // TODO: Add service servers
}

void LabelGenerationNode::lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  if (!lidarscan_received_) {
    lidarscan_received_ = true;
    frameID.sensor = msg->header.frame_id;
    map_publish_timer_.start();
    std::cout << "\033[1;32m[lesta_ros::LabelGenerationNode]: Pointcloud Received! "
              << "Use LiDAR scans for label generation... \033[0m\n";
  }

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped sensor2base, base2map;
  if (!tf_.lookupTransform(frameID.base_link, frameID.sensor, sensor2base) ||
      !tf_.lookupTransform(frameID.map, frameID.base_link, base2map))
    return;

  // 2. Convert ROS msg to PCL data
  auto scan_raw = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  // 3. Preprocess scan data
  auto scan_preprocessed = preprocessScan(scan_raw, sensor2base, base2map);

  // 4. Terrain mapping
  auto sensor2map = tf_.combineTransforms(sensor2base, base2map);
  terrainMapping(scan_preprocessed, sensor2map);

  // 5. Feature extraction
  featureExtraction(scan_preprocessed);
}

pcl::PointCloud<Laser>::Ptr
LabelGenerationNode::preprocessScan(const pcl::PointCloud<Laser>::Ptr &scan_raw,
                                    const geometry_msgs::TransformStamped &sensor2base,
                                    const geometry_msgs::TransformStamped &base2map) {

  // 1. Transform pointcloud to base frame
  auto scan_base = pc_utils::applyTransform<Laser>(scan_raw, sensor2base);

  // 2. Fast height filtering
  auto scan_preprocessed = boost::make_shared<pcl::PointCloud<Laser>>();
  mapper_->fastHeightFilter(scan_base, scan_preprocessed);

  // 3. Pass through filter
  scan_preprocessed = pc_utils::passThrough<Laser>(scan_preprocessed, "x", -5.0, 5.0);
  scan_preprocessed = pc_utils::passThrough<Laser>(scan_preprocessed, "y", -5.0, 5.0);

  // (Optional) Remove remoter points
  if (cfg_.remove_backward_points)
    scan_preprocessed = pc_utils::filterAngle<Laser>(scan_preprocessed, -135.0, 135.0);

  // 4. Transform pointcloud to map frame
  scan_preprocessed = pc_utils::applyTransform<Laser>(scan_preprocessed, base2map);

  if (scan_preprocessed->empty())
    return nullptr;
  return scan_preprocessed;
}

void LabelGenerationNode::terrainMapping(const pcl::PointCloud<Laser>::Ptr &inputcloud,
                                         const geometry_msgs::TransformStamped &sensor2map) {

  // mapping
  auto cloud_rasterized = mapper_->heightMapping(inputcloud);

  // raycasting
  Eigen::Vector3f sensorOrigin3D(sensor2map.transform.translation.x, sensor2map.transform.translation.y,
                                 sensor2map.transform.translation.z);
  mapper_->raycasting(sensorOrigin3D, cloud_rasterized);
}

void LabelGenerationNode::featureExtraction(const pcl::PointCloud<Laser>::Ptr &inputcloud) {
  // TODO: Implement feature extraction
}

void LabelGenerationNode::updateRobotPose(const ros::TimerEvent &event) {

  geometry_msgs::TransformStamped base2map;
  if (!tf_.lookupTransform(frameID.map, frameID.base_link, base2map))
    return;

  //
}

void LabelGenerationNode::publishLabelMap(const ros::TimerEvent &event) {

  // publish label map
  sensor_msgs::PointCloud2 cloud_msg;
  toPointCloud2(mapper_->getHeightMap(), mapper_->getHeightMap().getLayers(),
                mapper_->getMeasuredGridIndices(), cloud_msg);
  pub_labelmap_global_.publish(cloud_msg);
}

// void LabelGenerationNode::updateLabelmapFrom(const grid_map::HeightMap &featuremap) {
//   bool param_vis = pnh_.param<bool>("enableNegativeLabeler", false);

//   // Update labelmap with featuremap
//   for (grid_map::GridMapIterator it(featuremap); !it.isPastEnd(); ++it) {
//     const auto &featuremap_iterator = *it;
//     if (featuremap.isEmptyAt(featuremap_iterator))
//       continue;

//     grid_map::Position grid_position;
//     featuremap.getPosition(featuremap_iterator, grid_position);

//     grid_map::Index labelmap_index;
//     if (!labelmap_.getIndex(grid_position, labelmap_index))
//       continue;

//     if (labelmap_.at(labelmap_.getVarianceLayer(), labelmap_index) > 0.03)
//       continue;

//     if (!labelmap_.isEmptyAt("footprint", labelmap_index))
//       continue;

//     const auto &height = featuremap.at(featuremap.getHeightLayer(), featuremap_iterator);
//     const auto &step = featuremap.at("step", featuremap_iterator);
//     const auto &slope = featuremap.at("slope", featuremap_iterator);
//     const auto &roughness = featuremap.at("roughness", featuremap_iterator);
//     const auto &curvature = featuremap.at("curvature", featuremap_iterator);
//     const auto &variance = featuremap.at(featuremap.getVarianceLayer(), featuremap_iterator);

//     labelmap_.at(labelmap_.getHeightLayer(), labelmap_index) = height;
//     labelmap_.at("step", labelmap_index) = step;
//     labelmap_.at("slope", labelmap_index) = slope;
//     labelmap_.at("roughness", labelmap_index) = roughness;
//     labelmap_.at("curvature", labelmap_index) = curvature;
//     labelmap_.at(labelmap_.getVarianceLayer(), labelmap_index) = variance;

//     valid_indices_.insert(labelmap_index);

//     if (param_vis && step > max_acceptable_step_) {
//       labelmap_.at("traversability_label", labelmap_index) = (float)Traversability::NON_TRAVERSABLE;
//     }
//   }
// }

// void LabelGenerationNode::initializeLabelMap(const grid_map::HeightMap &featuremap) {
//   // Define grid resolution
//   auto resolution = featuremap.getResolution();
//   // labelmap_ = std::make_shared<grid_map::HeightMap>(map_length.x(), map_length.y(), resolution);
//   labelmap_.setGeometry(labelmap_.getLength(), resolution);
//   labelmap_.setFrameId(featuremap.getFrameId());

//   // Add layers
//   labelmap_.addLayer("step");
//   labelmap_.addLayer("slope");
//   labelmap_.addLayer("roughness");
//   labelmap_.addLayer("curvature");
//   labelmap_.addLayer("footprint");
//   labelmap_.addLayer("traversability_label");
//   labelmap_.setBasicLayers({labelmap_.getHeightLayer(), labelmap_.getVarianceLayer()});

//   valid_indices_.reserve(labelmap_.getSize().prod());
// }

// void LabelGenerationNode::visualizeLabelSubmap(const grid_map::Length &length) {
//   // get current robot pose
//   auto [get_transform_b2m, base2map] = tf_.getTransform(baselink_frame, map_frame);
//   if (!get_transform_b2m)
//     return;

//   auto robot_position =
//       grid_map::Position(base2map.transform.translation.x, base2map.transform.translation.y);

//   // Visualize submap
//   bool get_submap{false};
//   auto submap = labelmap_.getSubmap(robot_position, length, get_submap);
//   if (!get_submap)
//     return;

//   grid_map_msgs::GridMap message;
//   grid_map::GridMapRosConverter::toMessage(submap, message);
//   pub_labelmap_local_.publish(message);
// }

// void LabelGenerationNode::visualizeLabelMap() {
//   // Visualize global labelmap
//   sensor_msgs::PointCloud2 cloud_msg;
//   toPointCloud2(labelmap_, labelmap_.getLayers(), valid_indices_, cloud_msg);
//   pub_labelmap_global_.publish(cloud_msg);

//   // Visualize label map region
//   visualization_msgs::Marker msg_map_region;
//   HeightMapMsgs::toMapRegion(labelmap_, msg_map_region);
//   pub_labelmap_region_.publish(msg_map_region);
// }

// void LabelGenerationNode::recordFootprints() {
//   // Get Transform from base_link to map (typically provided by 3D pose estimator)
//   auto [get_transform_b2m, base2map] = tf_.getTransform(baselink_frame, map_frame);
//   if (!get_transform_b2m)
//     return;

//   grid_map::Position footprint(base2map.transform.translation.x, base2map.transform.translation.y);
//   grid_map::CircleIterator iterator(labelmap_, footprint, footprint_radius_);

//   for (iterator; !iterator.isPastEnd(); ++iterator) {
//     Eigen::Vector3d grid_with_elevation;
//     if (!labelmap_.getPosition3(labelmap_.getHeightLayer(), *iterator, grid_with_elevation))
//       continue;

//     // Due to perception error, footprint terrain geometry sometimes become messy
//     if (labelmap_.at("step", *iterator) > max_acceptable_step_)
//       continue;

//     labelmap_.at("traversability_label", *iterator) = (float)Traversability::TRAVERSABLE;
//     labelmap_.at("footprint", *iterator) = (float)Traversability::TRAVERSABLE;
//   }
// }

// void LabelGenerationNode::recordUnknownAreas(grid_map::HeightMap &labelmap) {
//   for (grid_map::GridMapIterator iter(labelmap); !iter.isPastEnd(); ++iter) {
//     Eigen::Vector3d grid_with_elevation;
//     if (!labelmap.getPosition3(labelmap.getHeightLayer(), *iter, grid_with_elevation))
//       continue;

//     const auto &traversability_label = labelmap.at("traversability_label", *iter);
//     bool has_label = std::isfinite(traversability_label);
//     if (has_label)
//       continue;

//     labelmap.at("traversability_label", *iter) = (float)Traversability::UNKNOWN;
//   }
// }

// bool LabelGenerationNode::visualizeNegativeLabels(std_srvs::Empty::Request &req,
//                                                   std_srvs::Empty::Response &res) {
//   std::cout << "[LeSTA @LabelGeneration] Visualizing Traversability Labels..." << std::endl;

//   recordNegativeLabel();

//   std::cout << "\033[32m[LeSTA @LabelGeneration] Recorded Labels visualized.\033[0m" << std::endl;

//   return true;
// }

// void LabelGenerationNode::recordNegativeLabel() {
//   for (grid_map::GridMapIterator iter(labelmap_); !iter.isPastEnd(); ++iter) {
//     Eigen::Vector3d grid_with_elevation;
//     if (!labelmap_.getPosition3(labelmap_.getHeightLayer(), *iter, grid_with_elevation))
//       continue;

//     if (!std::isfinite(labelmap_.at("step", *iter)))
//       continue;

//     // Condition for non-traversable areas
//     bool non_traversable = labelmap_.at("step", *iter) > max_acceptable_step_;
//     if (non_traversable) {
//       labelmap_.at("traversability_label", *iter) = (float)Traversability::NON_TRAVERSABLE;
//       continue;
//     } else // Remove noisy labels
//     {
//       bool label_conflict = std::abs(labelmap_.at("traversability_label", *iter) -
//                                      (float)Traversability::NON_TRAVERSABLE) < 1e-6;
//       if (label_conflict) {
//         labelmap_.at("traversability_label", *iter) = NAN;
//       }
//     }
//   }
// }

// bool LabelGenerationNode::saveTrainingData(lesta::save_training_data::Request &req,
//                                            lesta::save_training_data::Response &res) {
//   std::cout << "[LeSTA @LabelGeneration] Generating Traversability Labels..." << std::endl;

//   recordNegativeLabel();

//   std::cout << "\033[32m[LeSTA @LabelGeneration] Done.\033[0m" << std::endl;

//   std::cout << "[LeSTA @LabelGeneration] Saving Traversability Dataset..." << std::endl;

//   // Check if the directory exists
//   auto dataset_dir = req.destination;
//   if (!std::filesystem::exists(dataset_dir)) {
//     // color yellow print
//     std::cerr << "\033[33m[LeSTA @LabelGeneration] Directory does not exist: " << dataset_dir << "\033[0m"
//               << std::endl;
//     std::cerr << "\033[33m[LeSTA @LabelGeneration] Is the path relative? Please give absolute path!\033[0m"
//               << std::endl;
//     res.success = false;
//     return res.success;
//   }

//   // Format time
//   auto now = std::chrono::system_clock::now();
//   auto now_c = std::chrono::system_clock::to_time_t(now);
//   std::stringstream ss;
//   ss << std::put_time(std::localtime(&now_c), "%Y-%m-%d-%H-%M-%S");

//   auto labeled_data_path = dataset_dir + "/labeled_data_" + ss.str() + ".csv";
//   auto unlabeled_data_path = dataset_dir + "/unlabeled_data_" + ss.str() + ".csv";

//   // Save labeled data
//   std::ofstream labeled_data(labeled_data_path);
//   labeled_data << "step,slope,roughness,curvature,variance,traversability_label\n"; // header

//   // Save unlabeled data
//   std::ofstream unlabeled_data(unlabeled_data_path);
//   unlabeled_data << "step,slope,roughness,curvature,variance,traversability_label\n"; // header

//   // Iterate over the grid map
//   for (grid_map::GridMapIterator iter(labelmap_); !iter.isPastEnd(); ++iter) {
//     if (labelmap_.isEmptyAt(*iter))
//       continue;

//     if (labelmap_.isEmptyAt("step", *iter))
//       continue;

//     // Write data to csv
//     float step = labelmap_.at("step", *iter);
//     float slope = labelmap_.at("slope", *iter);
//     float roughness = labelmap_.at("roughness", *iter);
//     float curvature = labelmap_.at("curvature", *iter);
//     float variance = labelmap_.at("variance", *iter);
//     int traversability_label = labelmap_.at("traversability_label", *iter);

//     if (!std::isfinite(traversability_label)) {
//       auto label_unknown = (int)Traversability::UNKNOWN;
//       unlabeled_data << step << "," << slope << "," << roughness << "," << curvature << "," << variance <<
//       ","
//                      << label_unknown << "\n";
//       continue;
//     }

//     labeled_data << step << "," << slope << "," << roughness << "," << curvature << "," << variance << ","
//                  << traversability_label << "\n";
//   }

//   labeled_data.close();
//   unlabeled_data.close();

//   std::cout << "\033[32m[LeSTA @LabelGeneration] Done.\033[0m" << std::endl;

//   res.success = true;

//   return res.success;
// }

void LabelGenerationNode::toPointCloud2(const grid_map::HeightMap &map,
                                        const std::vector<std::string> &layers,
                                        const std::unordered_set<grid_map::Index> &measuredIndices,
                                        sensor_msgs::PointCloud2 &cloud) {

  // Setup cloud header
  cloud.header.frame_id = map.getFrameId();
  cloud.header.stamp.fromNSec(map.getTimestamp());
  cloud.is_dense = false;

  // Setup field names and cloud structure
  std::vector<std::string> fieldNames;
  fieldNames.reserve(layers.size());

  // Setup field names
  fieldNames.insert(fieldNames.end(), {"x", "y", "z"});
  for (const auto &layer : layers) {
    if (layer == "color") {
      fieldNames.push_back("rgb");
    } else {
      fieldNames.push_back(layer);
    }
  }

  // Setup point field structure
  cloud.fields.clear();
  cloud.fields.reserve(fieldNames.size());
  int offset = 0;

  for (const auto &name : fieldNames) {
    sensor_msgs::PointField field;
    field.name = name;
    field.count = 1;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.offset = offset;
    cloud.fields.push_back(field);
    offset += sizeof(float);
  }

  // Initialize cloud size
  const size_t num_points = measuredIndices.size();
  cloud.height = 1;
  cloud.width = num_points;
  cloud.point_step = offset;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);

  // Setup point field iterators
  std::unordered_map<std::string, sensor_msgs::PointCloud2Iterator<float>> iterators;
  for (const auto &name : fieldNames) {
    iterators.emplace(name, sensor_msgs::PointCloud2Iterator<float>(cloud, name));
  }

  // Fill point cloud data
  size_t validPoints = 0;
  for (const auto &index : measuredIndices) {
    grid_map::Position3 position;
    if (!map.getPosition3(grid_map::HeightMap::CoreLayers::ELEVATION, index, position)) {
      continue;
    }

    // Update each field
    for (auto &[fieldName, iterator] : iterators) {
      if (fieldName == "x")
        *iterator = static_cast<float>(position.x());
      else if (fieldName == "y")
        *iterator = static_cast<float>(position.y());
      else if (fieldName == "z")
        *iterator = static_cast<float>(position.z());
      else if (fieldName == "rgb")
        *iterator = static_cast<float>(map.at("color", index));
      else
        *iterator = static_cast<float>(map.at(fieldName, index));
      ++iterator;
    }
    ++validPoints;
  }

  // Adjust final cloud size
  cloud.width = validPoints;
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.height * cloud.row_step);
}

void LabelGenerationNode::toMapRegion(const grid_map::HeightMap &map, visualization_msgs::Marker &marker) {

  marker.ns = "height_map";
  marker.lifetime = ros::Duration();
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  marker.scale.x = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.header.frame_id = map.getFrameId();
  marker.header.stamp = ros::Time::now();

  float length_x_half = (map.getLength().x() - 0.5 * map.getResolution()) / 2.0;
  float length_y_half = (map.getLength().y() - 0.5 * map.getResolution()) / 2.0;

  marker.points.resize(5);
  marker.points[0].x = map.getPosition().x() + length_x_half;
  marker.points[0].y = map.getPosition().y() + length_x_half;
  marker.points[0].z = 0;

  marker.points[1].x = map.getPosition().x() + length_x_half;
  marker.points[1].y = map.getPosition().y() - length_x_half;
  marker.points[1].z = 0;

  marker.points[2].x = map.getPosition().x() - length_x_half;
  marker.points[2].y = map.getPosition().y() - length_x_half;
  marker.points[2].z = 0;

  marker.points[3].x = map.getPosition().x() - length_x_half;
  marker.points[3].y = map.getPosition().y() + length_x_half;
  marker.points[3].z = 0;

  marker.points[4] = marker.points[0];
}
} // namespace lesta_ros

int main(int argc, char **argv) {

  ros::init(argc, argv, "label_generation_node");
  lesta_ros::LabelGenerationNode node;
  ros::spin();

  return 0;
}