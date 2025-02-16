/*
 * LabelGenerator.cpp
 *
 *  Created on: Feb 10, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "lesta/core/LabelGenerator.h"

LabelGenerator::LabelGenerator(const Config &cfg) : cfg(cfg) {}

void LabelGenerator::layerInit(grid_map::HeightMap &map) {

  map.addLayer("footprint");
  map.addLayer("traversability_label");
}

void LabelGenerator::addFootprint(grid_map::HeightMap &map, grid_map::Position &robot_position) {

  layerInit(map);
  //
  grid_map::CircleIterator iterator(map, robot_position, cfg.footprint_radius);
  for (iterator; !iterator.isPastEnd(); ++iterator) {
    Eigen::Vector3d grid_with_elevation;
    if (!map.getPosition3(grid_map::HeightMap::CoreLayers::ELEVATION, *iterator, grid_with_elevation))
      continue;

    map.at("footprint", *iterator) = 1.0;
    map.at("traversability_label", *iterator) = (float)Traversability::TRAVERSABLE;
  }
}

void LabelGenerator::addObstacles(grid_map::HeightMap &map, pcl::PointCloud<Laser>::Ptr &scan) {

  layerInit(map);

  for (const auto &point : scan->points) {
    grid_map::Position position(point.x, point.y);
    grid_map::Index index;
    if (!map.getIndex(position, index))
      continue;

    // pass if recoreded as traversable
    if (map.at("traversability_label", index) > 0.5)
      continue;

    if (map.isEmptyAt("step", index))
      continue;

    if (map.at("step", index) > cfg.max_acceptable_step)
      map.at("traversability_label", index) = (float)Traversability::NON_TRAVERSABLE;
    else
      map.at("traversability_label", index) = (float)Traversability::UNKNOWN;
  }
}
