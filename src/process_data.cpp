#include "process_data.h"

// check if room's name is not empty
void checkRoomName(unsigned argcNumber) {
  if (argcNumber == 1) {
    std::cout << "Provided room name is empty!\n";
    exit(1);
  }
}

// read file to stream
std::stringstream readFile(std::string filename) {
  std::stringstream stream;
  char buffer[PATH_MAX];
  if (getcwd(buffer, sizeof(buffer)) != NULL) {
    std::ifstream file(buffer + '/' + filename, std::ifstream::binary);
    stream << file.rdbuf();
    if (!stream) {
      printf("No such file in working directory: %s\n", filename.c_str());
      file.close();
    }

  } else {
    std::cout << "getcwd() error: cannot get current working directory\n";
  }
  return stream;
}

// create list of rooms from stream
std::vector<Room *> getRooms(std::stringstream *rooms_stream,
                             char line_delimiter, char value_delimiter) {
  std::vector<Room *> rooms;
  std::string segment;
  std::vector<std::string> seglist;

  while (std::getline(*rooms_stream, segment, line_delimiter)) {
    seglist.push_back(segment);
  }

  for (unsigned i = 1; i < seglist.size(); i++) {

    std::string property_value;
    std::vector<std::string> values_list;
    std::stringstream property_values;
    property_values << seglist[i];
    while (std::getline(property_values, property_value, value_delimiter)) {
      values_list.push_back(property_value);
    }
    Room *room = new Room(values_list[0], StringToNumber<float>(values_list[1]),
                          StringToNumber<float>(values_list[2]),
                          StringToNumber<float>(values_list[3]),
                          StringToNumber<float>(values_list[4]));
    rooms.push_back(room);
  }
  return rooms;
}

std::tuple<bool, unsigned> findRoom(std::vector<Room *> *rooms,
                                    char *roomArgument) {
  bool match = false;
  unsigned room_index;
  // get goal point name
  std::string goalRoomName(roomArgument);

  // iterate through list of avaible rooms
  for (unsigned i = 0; i < rooms->size(); i++) {
    // if provided point is in the list then set coordinates of this point
    if (goalRoomName.compare((*rooms)[i]->getName()) == 0) {
      match = true;
      room_index = i;
    }
  }
  return std::make_tuple(match, room_index);
}

void showCorrectRoomNames(std::vector<Room *> *rooms) {
  std::cout << "Provided point name is not in list of avaible room "
               "names.\nAvaible room names:\n";
  for (unsigned i = 0; i < rooms->size(); i++) {
    std::cout << (*rooms)[i]->getName() << " ";
  }
  std::cout << "\n";
}