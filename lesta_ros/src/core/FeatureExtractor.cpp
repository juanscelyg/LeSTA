/*
 * FeatureExtraction.cpp
 *
 *  Created on: Feb 07, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta_ros/core/FeatureExtractor.h"

FeatureExtractor::FeatureExtractor(const Config &cfg) : cfg(cfg) {}

void FeatureExtractor::layerInit(grid_map::HeightMap &map) {

  // Basic feature layers
  map.addLayer("step");
  map.addLayer("slope");
  map.addLayer("roughness");
  map.addLayer("curvature");
  map.addLayer("variance");

  // Layers for visualization of normal vector
  map.addLayer("normal_x");
  map.addLayer("normal_y");
  map.addLayer("normal_z");

  std::vector<std::string> basic_layers{"step", "slope", "roughness", "curvature", "variance"};
  basic_layers.push_back(grid_map::HeightMap::CoreLayers::ELEVATION);
  // map.setBasicLayers(basic_layers);
}

void FeatureExtractor::extractFeatures(grid_map::HeightMap &map) {

  layerInit(map);
  //
}

void FeatureExtractor::extractFeatures(grid_map::HeightMap &map,
                                       const pcl::PointCloud<Laser>::Ptr &input_scan) {

  layerInit(map);
}
