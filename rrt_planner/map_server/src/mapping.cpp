#include "map_server/mapping.h"

namespace map_server
{

MapServer::MapServer(ros::NodeHandle *nh, std::string path) : 
_nh(nh),
_private_nh("~"),
_path(path),
_window("Map")
{
    parseYAML(); 
    getMap();  
    map_pub_ = _nh->advertise<nav_msgs::OccupancyGrid>("/map", 1);

    ROS_INFO("Mapping node initialized");
    publishMap();
    // while(ros::ok()){
    //     publishMap();
    //     ros::Duration(0.1).sleep();
    //     ros::spinOnce();
    // }
}

MapServer::~MapServer() {
  cv::destroyWindow(_window);
}

void MapServer::parseYAML(){
    YAML::Node config = YAML::LoadFile(_path);
    _image_path = config["image"].as<std::string>();
    _resolution = config["resolution"].as<double>();
    _origin = config["origin"].as<std::vector<double>>();
    _occupied_thresh = config["occupied_thresh"].as<double>();
    _free_thresh = config["free_thresh"].as<double>();
    _negate = config["negate"].as<double>();
}

void MapServer::getMap(){
    _map = cv::imread(_image_path);
    if (_map.empty()){
        std::cerr << "Could not open file" << std::endl;
        return;
    }
    _height = _map.rows;
    _width = _map.cols;
    ROS_INFO("Loaded map from: %s", _image_path.c_str());
}

void MapServer::publishMap(){
    nav_msgs::OccupancyGrid grid;

    grid.info.map_load_time = ros::Time::now();
    grid.header.frame_id = "map";
    grid.header.stamp = ros::Time::now();

    grid.info.width = (unsigned int) _width;
    grid.info.height = (unsigned int) _height;
    grid.info.resolution = _resolution;

    grid.info.origin.position.x = _origin[0];
    grid.info.origin.position.y = _origin[1];
    grid.info.origin.position.z = _origin[2];

    grid.info.origin.orientation.x = 0.0;
    grid.info.origin.orientation.y = 0.0;
    grid.info.origin.orientation.z = 0.0;
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(grid.info.width * grid.info.height);

    int scale = 0;

    for(unsigned int i = 0; i < grid.info.height; i++) {
        for (unsigned int j = 0; j < grid.info.width; j++) {
            double color_sum = 0;
            for(size_t k=0;k<3;k++){
                color_sum += _negate ? _map.at<cv::Vec3b>(i, j).val[k] : 255.0 - _map.at<cv::Vec3b>(i, j).val[k];
            }
            color_sum /= 3.0;
            color_sum /= 255.0;
            unsigned char val;
            if(color_sum > _occupied_thresh){
                val = 100;
            }else if(color_sum < _free_thresh){
                val = 0;
            }else{
                val = scale ? 99 * (val - _free_thresh) / (_occupied_thresh - _free_thresh) : -1;
            }
            grid.data[(grid.info.height -i - 1) * grid.info.width + j] = (char)val;
        }
    } 
    
    cv::imshow(_window, _map);
    cv::waitKey(0);
    
    map_pub_.publish(grid);
    ROS_INFO("Published map!");    
}


}