/*
 * trav_estimation_node.cpp
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta/ros/config.h"
#include "lesta/ros/trav_estimation_node.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace lesta_ros {

TravEstimationNode::TravEstimationNode() : nh_("~"), lidarscan_received_(false) {

  // ROS node
  ros::NodeHandle nh_node(nh_, "node");
  TravEstimationNode::loadConfig(nh_node);
  initializePubSubs();
  initializeServices();
  initializeTimers();

  // Height Mapper
  ros::NodeHandle nh_mapper(nh_, "height_mapper");
  auto cfg1 = height_mapper::loadConfig(nh_mapper);
  mapper_ = std::make_unique<HeightMapper>(cfg1);

  // Feature Extractor
  ros::NodeHandle nh_feature_extractor(nh_, "feature_extractor");
  auto cfg2 = feature_extractor::loadConfig(nh_feature_extractor);
  feature_extractor_ = std::make_unique<FeatureExtractor>(cfg2);

  // Traversability Estimator
  ros::NodeHandle nh_traversability_estimator(nh_, "traversability_estimator");
  auto cfg3 = traversability_estimator::loadConfig(nh_traversability_estimator);
  traversability_estimator_ = std::make_unique<TraversabilityEstimator>(cfg3);

  // Transform object
  ros::NodeHandle nh_frame_id(nh_, "frame_id");
  frame_id_ = FrameID::loadConfig(nh_frame_id);
}

void TravEstimationNode::loadConfig(const ros::NodeHandle &nh) {

  cfg_.lidarscan_topic = nh.param<std::string>("lidarscan_topic", "/velodyne_points");
  cfg_.pose_update_rate = nh.param<double>("pose_update_rate", 10.0);
  cfg_.map_pub_rate = nh.param<double>("map_publish_rate", 10.0);
  cfg_.remove_backpoints = nh.param<bool>("remove_backpoints", true);
  cfg_.debug_mode = nh.param<bool>("debug_mode", false);
}

void TravEstimationNode::initializePubSubs() {

  sub_lidarscan_ = nh_.subscribe(cfg_.lidarscan_topic, 1, &TravEstimationNode::lidarScanCallback, this);
  pub_travmap_ = nh_.advertise<grid_map_msgs::GridMap>("/lesta/estimation/map_grid", 1);
}

void TravEstimationNode::initializeServices() {
  // TODO: Implement this
}

void TravEstimationNode::initializeTimers() {

  ros::Duration pose_update_dt(1.0 / cfg_.pose_update_rate);
  ros::Duration map_pub_dt(1.0 / cfg_.map_pub_rate);

  pose_update_timer_ =
      nh_.createTimer(pose_update_dt, &TravEstimationNode::updateMapOrigin, this, false, false);
  map_publish_timer_ = nh_.createTimer(map_pub_dt, &TravEstimationNode::publishTravMap, this, false, false);
}

void TravEstimationNode::lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg) {

  if (!lidarscan_received_) {
    lidarscan_received_ = true;
    frame_id_.sensor_frame = msg->header.frame_id;
    pose_update_timer_.start();
    map_publish_timer_.start();
    std::cout << "\033[1;32m[lesta_ros::TravEstimationNode]: Pointcloud Received! "
              << "Use LiDAR scans for traversability estimation... \033[0m\n";
  }

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped sensor2base, base2map;
  if (!tf_.lookupTransform(frame_id_.base_frame, frame_id_.sensor_frame, sensor2base) ||
      !tf_.lookupTransform(frame_id_.map_frame, frame_id_.base_frame, base2map))
    return;

  // 2. Convert ROS msg to PCL data
  auto scan_raw = boost::make_shared<pcl::PointCloud<Laser>>();
  pcl::moveFromROSMsg(*msg, *scan_raw);

  // 3. Preprocess scan data: ready for terrain mapping
  auto scan_preprocessed = preprocessScan(scan_raw, sensor2base, base2map);
  if (!scan_preprocessed)
    return;

  // 4. Terrain mapping
  auto transform_sensor2map = TransformOps::multiplyTransforms(sensor2base, base2map);
  Eigen::Vector3f sensor_origin(transform_sensor2map.transform.translation.x,
                                transform_sensor2map.transform.translation.y,
                                transform_sensor2map.transform.translation.z);
  auto scan_mapped = boost::make_shared<pcl::PointCloud<Laser>>();
  terrainMapping(scan_preprocessed, sensor_origin, scan_mapped);

  // 5. Feature extraction
  feature_extractor_->extractFeatures(mapper_->getHeightMap(), scan_mapped);

  // 6. Traversability estimation
  traversability_estimator_->predict(mapper_->getHeightMap(), scan_mapped);
}

pcl::PointCloud<Laser>::Ptr
TravEstimationNode::preprocessScan(const pcl::PointCloud<Laser>::Ptr &scan_raw,
                                   const geometry_msgs::TransformStamped &sensor2base,
                                   const geometry_msgs::TransformStamped &base2map) {

  // 1. Transform pointcloud to base frame
  auto scan_base = PointCloudOps::applyTransform<Laser>(scan_raw, sensor2base);

  // 2. Fast height filtering
  auto scan_preprocessed = boost::make_shared<pcl::PointCloud<Laser>>();
  mapper_->fastHeightFilter(scan_base, scan_preprocessed);

  // 3. Pass through filter
  auto range = mapper_->getHeightMap().getLength() / 2.0;
  scan_preprocessed = PointCloudOps::passThrough<Laser>(scan_preprocessed, "x", -range.x(), range.x());
  scan_preprocessed = PointCloudOps::passThrough<Laser>(scan_preprocessed, "y", -range.y(), range.y());

  // (Optional) Remove remoter points
  if (cfg_.remove_backpoints)
    scan_preprocessed = PointCloudOps::filterAngle2D<Laser>(scan_preprocessed, -135.0, 135.0);

  // 4. Transform pointcloud to map frame
  scan_preprocessed = PointCloudOps::applyTransform<Laser>(scan_preprocessed, base2map);

  if (scan_preprocessed->empty())
    return nullptr;
  return scan_preprocessed;
}

void TravEstimationNode::terrainMapping(const pcl::PointCloud<Laser>::Ptr &cloud_input,
                                        const Eigen::Vector3f &sensor_origin,
                                        pcl::PointCloud<Laser>::Ptr &cloud_output) {

  auto cloud_rasterized = mapper_->heightMapping(cloud_input);
  mapper_->raycasting(sensor_origin, cloud_rasterized);
  cloud_output = cloud_rasterized;
}

void TravEstimationNode::updateMapOrigin(const ros::TimerEvent &event) {

  // 1. Get transform matrix using tf tree
  geometry_msgs::TransformStamped base2map;
  if (!tf_.lookupTransform(frame_id_.map_frame, frame_id_.base_frame, base2map))
    return;

  // 2. Update map origin
  auto robot_position =
      grid_map::Position(base2map.transform.translation.x, base2map.transform.translation.y);
  mapper_->moveMapOrigin(robot_position);
}

void TravEstimationNode::publishTravMap(const ros::TimerEvent &event) {

  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(mapper_->getHeightMap(), msg);
  pub_travmap_.publish(msg);
}
} // namespace lesta_ros

int main(int argc, char **argv) {

  ros::init(argc, argv, "trav_estimation_node");
  lesta_ros::TravEstimationNode node;
  ros::spin();

  return 0;
}