/*
 * label_generation_node.h
 *
 *  Created on: Aug 17, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

#include "lesta_ros/core/GlobalMapper.h"
#include "lesta_ros/core/FeatureExtractor.h"
#include "lesta_ros/utils/TransformHandler.h"

namespace lesta_ros {

class LabelGenerationNode {
public:
  struct Config {
    std::string lidarscan_topic;
    double robot_pose_update_rate;
    double map_publish_rate;
    bool remove_backward_points;
    bool debug_mode;
  };

  LabelGenerationNode();
  ~LabelGenerationNode() = default;
  void loadConfig(const ros::NodeHandle &nh);

private:
  // init functions
  void initializeTimers();
  void initializePubSubs();
  void initializeServices();

  void lidarScanCallback(const sensor_msgs::PointCloud2Ptr &msg);
  pcl::PointCloud<Laser>::Ptr preprocessScan(const pcl::PointCloud<Laser>::Ptr &scan_raw,
                                             const geometry_msgs::TransformStamped &sensor2base,
                                             const geometry_msgs::TransformStamped &base2map);
  void terrainMapping(const pcl::PointCloud<Laser>::Ptr &inputcloud,
                      const geometry_msgs::TransformStamped &sensor2map);

  void featureExtraction(const pcl::PointCloud<Laser>::Ptr &scan_raw);

  void updateRobotPose(const ros::TimerEvent &event);
  void publishLabelMap(const ros::TimerEvent &event);

  void toPointCloud2(const grid_map::HeightMap &map, const std::vector<std::string> &layers,
                     const std::unordered_set<grid_map::Index> &grid_indices,
                     sensor_msgs::PointCloud2 &cloud);
  void toMapRegion(const grid_map::HeightMap &map, visualization_msgs::Marker &region);

  ros::NodeHandle nh_;

  // Config
  LabelGenerationNode::Config cfg_;

  // Subscribers
  ros::Subscriber sub_lidarscan_;

  // Publishers
  ros::Publisher pub_labelmap_local_;
  ros::Publisher pub_labelmap_global_;
  ros::Publisher pub_labelmap_region_;

  // Timers
  ros::Timer map_publish_timer_;
  ros::Timer robot_pose_update_timer_;

  // Core objects
  std::unique_ptr<GlobalMapper> mapper_;
  std::unique_ptr<FeatureExtractor> feature_extractor_;
  TransformHandler tf_;
  TransformHandler::FrameID frameID;

  // State variables
  bool lidarscan_received_{false};
};
} // namespace lesta_ros