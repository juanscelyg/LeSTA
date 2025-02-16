/*
 * LabelGenerator.h
 *
 *  Created on: Feb 07, 2025
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#pragma once

#include <height_mapping_core/height_mapping_core.h>

class LabelGenerator {
public:
  struct Config {
    double footprint_radius;
    double max_acceptable_step;
  } cfg;

  enum class Traversability : int {
    TRAVERSABLE = 1,
    NON_TRAVERSABLE = 0,
    UNKNOWN = -1,
  };

  LabelGenerator(const Config &cfg);

  void addFootprint(grid_map::HeightMap &map, grid_map::Position &robot_position);
  void addObstacles(grid_map::HeightMap &map, pcl::PointCloud<Laser>::Ptr &scan);

private:
  void layerInit(grid_map::HeightMap &map);
};
