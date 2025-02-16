/*
 * TraversabilityEstimator.cpp
 *
 *  Created on: Feb 15, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta/core/TraversabilityEstimator.h"

TraversabilityEstimator::TraversabilityEstimator(const Config &cfg) : cfg(cfg) {
  //
}

void TraversabilityEstimator::predict(grid_map::HeightMap &map) {
  //
}

void TraversabilityEstimator::predict(grid_map::HeightMap &map,
                                      const pcl::PointCloud<Laser>::Ptr &input_scan) {
  //
}
