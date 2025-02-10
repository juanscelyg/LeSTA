/*
 * FeatureExtraction.cpp
 *
 *  Created on: Feb 07, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta_ros/core/FeatureExtractor.h"

FeatureExtractor::FeatureExtractor(const Config &cfg) : cfg(cfg) {
  //
}

void FeatureExtractor::extractFeatures(const grid_map::HeightMap &map) {
  // TODO: Implement feature extraction
}

void FeatureExtractor::extractFeatures(const grid_map::HeightMap &map,
                                       const pcl::PointCloud<Laser>::Ptr &input_scan) {
  // TODO: Implement feature extraction
}
