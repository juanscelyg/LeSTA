/*
 * FeatureExtractor.cpp
 *
 *  Created on: Feb 07, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta/core/FeatureExtractor.h"
#include <Eigen/Dense>

FeatureExtractor::FeatureExtractor(const Config &cfg) : cfg(cfg) {}

void FeatureExtractor::layerInit(grid_map::HeightMap &map) {

  // Basic feature layers
  map.addLayer("step");
  map.addLayer("slope");
  map.addLayer("roughness");
  map.addLayer("curvature");

  // Layers for visualization of normal vector
  map.addLayer("normal_x");
  map.addLayer("normal_y");
  map.addLayer("normal_z");
}

void FeatureExtractor::extractFeatures(grid_map::HeightMap &map) {

  layerInit(map);
  // TODO: implement feature extraction for entire map (less efficient)
}

void FeatureExtractor::extractFeatures(grid_map::HeightMap &map,
                                       const pcl::PointCloud<Laser>::Ptr &input_scan) {

  layerInit(map);

  for (const auto &point : input_scan->points) {

    grid_map::Position position(point.x, point.y);
    grid_map::Index index;
    if (!map.getIndex(position, index))
      continue;
    if (map.isEmptyAt(index))
      continue;

    const auto &neighbors = map.getNeighborHeights(index, cfg.pca_region_radius);
    if (neighbors.size() < 4)
      continue;

    // 1. Compute Covariance Matrix
    Eigen::Matrix3d covariance;
    Eigen::Vector3d sum_neighbors(Eigen::Vector3d::Zero());
    Eigen::Matrix3d squared_sum_neighbors(Eigen::Matrix3d::Zero());
    for (const auto &neighbor : neighbors) {
      sum_neighbors += neighbor;
      squared_sum_neighbors.noalias() += neighbor * neighbor.transpose();
    }
    const auto mean_neighbors = sum_neighbors / neighbors.size();
    covariance = squared_sum_neighbors / neighbors.size() - mean_neighbors * mean_neighbors.transpose();

    // Check if covariance matrix is degenerated using trace
    if (covariance.trace() < std::numeric_limits<float>::epsilon())
      continue;

    // Compute Eigenvectors and Eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver;
    solver.computeDirect(covariance, Eigen::DecompositionOptions::ComputeEigenvectors);
    const auto &eigenvectors = solver.eigenvectors();
    const auto &eigenvalues = solver.eigenvalues();

    // Line feature: second eigen value is near zero -> normal is not defined
    if (eigenvalues(1) < 1e-8)
      continue;

    // Check direction of the normal vector and flip the sign towards the user defined direction.
    Eigen::Vector3d normal_vector = eigenvectors.col(0);
    Eigen::Vector3d positive_normal_vector(Eigen::Vector3d::UnitZ());
    if (normal_vector.dot(positive_normal_vector) < 0.0)
      normal_vector *= -1;

    // Calculate step
    auto minMax = std::minmax_element(neighbors.begin(), neighbors.end(),
                                      [](const Eigen::Vector3d &lhs, const Eigen::Vector3d &rhs) {
                                        return lhs(2) < rhs(2); // Compare z-components.
                                      });

    double minZ = (*minMax.first)(2);  // Minimum z-component.
    double maxZ = (*minMax.second)(2); // Maximum z-component.
    map.at("step", index) = maxZ - minZ;

    map.at("slope", index) = std::acos(std::abs(normal_vector(2))) * 180 / M_PI;
    map.at("roughness", index) = std::sqrt(eigenvalues(0));
    map.at("curvature", index) = std::abs(eigenvalues(0) / covariance.trace());
    map.at("normal_x", index) = normal_vector(0);
    map.at("normal_y", index) = normal_vector(1);
    map.at("normal_z", index) = normal_vector(2);
  }
}
