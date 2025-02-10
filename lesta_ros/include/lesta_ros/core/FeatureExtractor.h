/*
 * FeatureExtraction.h
 *
 *  Created on: Feb 07, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <height_mapping_core/height_mapping_core.h>

class FeatureExtractor {
public:
  struct Config {
    double normal_estimation_radius;
  } cfg;

  FeatureExtractor(const Config &cfg);

  void extractFeatures(grid_map::HeightMap &map); // For static map
  void extractFeatures(grid_map::HeightMap &map,
                       const pcl::PointCloud<Laser>::Ptr &input_scan); // For lifelong mapping

private:
  void layerInit(grid_map::HeightMap &map);
};
