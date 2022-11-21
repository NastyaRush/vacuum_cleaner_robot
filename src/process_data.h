#pragma once
#include "room.h"
#include <fstream>
#include <iostream>
#include <limits.h>
#include <sstream>
#include <tuple>
#include <unistd.h>
#include <vector>

void checkRoomName(unsigned);
std::stringstream readFile(std::string);
std::vector<Room *> getRooms(std::stringstream *, char, char);
std::tuple<bool, unsigned> findRoom(std::vector<Room *> *rooms,
                                    char *goalRoomName);
void showCorrectRoomNames(std::vector<Room *> *rooms);

// convert string to number
template <typename T> T StringToNumber(const std::string &numberAsString) {
  T value;
  std::istringstream stream(numberAsString);
  stream >> value;
  if (stream.fail()) {
    std::runtime_error e(numberAsString);
    throw e;
  }
  return value;
}