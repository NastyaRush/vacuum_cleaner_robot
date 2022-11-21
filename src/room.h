#pragma once
#include "point.h"
#include <string>

// class for Rooms
class Room : public Point {
  float height;
  float width;

public:
  Room(std::string init_name, float x, float y, float init_height,
       float init_width)
      : Point(init_name, x, y), height(init_height), width(init_width) {}
  float getHeight();
  float getWidth();
};