#pragma once

#include <string>
#include <ros/node_handle.h>

#include "lesta/core/core.h"

namespace lesta_ros::height_mapper {

static HeightMapper::Config loadConfig(const ros::NodeHandle &nh) {

  HeightMapper::Config cfg;
  nh.param<std::string>("estimator_type", cfg.estimator_type, "StatMean");
  nh.param<std::string>("frame_id", cfg.frame_id, "map");
  nh.param<double>("map_length_x", cfg.map_length_x, 10.0);
  nh.param<double>("map_length_y", cfg.map_length_y, 10.0);
  nh.param<double>("grid_resolution", cfg.grid_resolution, 0.1);
  nh.param<double>("min_height_threshold", cfg.min_height, -0.2);
  nh.param<double>("max_height_threshold", cfg.max_height, 1.5);
  return cfg;
}
} // namespace lesta_ros::height_mapper

namespace lesta_ros::global_mapper {

static GlobalMapper::Config loadConfig(const ros::NodeHandle &nh) {

  GlobalMapper::Config cfg;
  nh.param<std::string>("estimator_type", cfg.estimator_type, "StatMean");
  nh.param<std::string>("frame_id", cfg.frame_id, "map");
  nh.param<double>("map_length_x", cfg.map_length_x, 400.0);
  nh.param<double>("map_length_y", cfg.map_length_y, 400.0);
  nh.param<double>("grid_resolution", cfg.grid_resolution, 0.1);
  nh.param<double>("min_height_threshold", cfg.min_height, -0.2);
  nh.param<double>("max_height_threshold", cfg.max_height, 1.5);

  nh.param<std::string>("map_save_dir", cfg.map_save_dir,
                        std::string("/home/") + std::getenv("USER") + "/Downloads");
  return cfg;
}
} // namespace lesta_ros::global_mapper

namespace lesta_ros::feature_extractor {

static FeatureExtractor::Config loadConfig(const ros::NodeHandle &nh) {

  FeatureExtractor::Config cfg;
  nh.param<double>("pca_region_radius", cfg.pca_region_radius, 0.2);

  return cfg;
}
} // namespace lesta_ros::feature_extractor

namespace lesta_ros::label_generator {

static LabelGenerator::Config loadConfig(const ros::NodeHandle &nh) {

  LabelGenerator::Config cfg;
  nh.param<double>("footprint_radius", cfg.footprint_radius, 0.5);
  nh.param<double>("max_acceptable_terrain_step", cfg.max_acceptable_step, 0.1);

  return cfg;
}
} // namespace lesta_ros::label_generator

namespace lesta_ros::traversability_estimator {

static TraversabilityEstimator::Config loadConfig(const ros::NodeHandle &nh) {

  TraversabilityEstimator::Config cfg;
  // TODO: add config parameters
  return cfg;
}
} // namespace lesta_ros::traversability_estimator
