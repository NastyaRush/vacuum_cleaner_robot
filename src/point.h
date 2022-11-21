#pragma once
#include <string>


//class for points 
class Point {
    std::string name;
    float xCoord;
    float yCoord;
public:
    Point(float x, float y) : xCoord(x), yCoord(y) {}
    Point(std::string init_name, float x, float y) : name(init_name), xCoord(x), yCoord(y) {}
	std::string getName();
	float getX();
	float getY();
};