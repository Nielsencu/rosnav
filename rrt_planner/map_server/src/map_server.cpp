#include <ros/ros.h>

#include "map_server/mapping.h"


int main(int argc, char** argv) {
  std::string path;
  for(int i =0; i < argc;i++){
      if(i == 1){
        path = argv[i];
      }
  }
  ros::init(argc, argv, "map_server");
  ros::NodeHandle node;
  new map_server::MapServer(&node,path);
}