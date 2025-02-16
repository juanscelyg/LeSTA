/*
 * TraversabilityEstimator.h
 *
 *  Created on: Feb 15, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <height_mapping_core/height_mapping_core.h>

class TraversabilityEstimator {
public:
  struct Config {
    // double pca_region_radius;
  } cfg;

  TraversabilityEstimator(const Config &cfg);

  void predict(grid_map::HeightMap &map); // For static map
  void predict(grid_map::HeightMap &map,
               const pcl::PointCloud<Laser>::Ptr &input_scan); // For lifelong estimation

private:
  void layerInit(grid_map::HeightMap &map);
};
