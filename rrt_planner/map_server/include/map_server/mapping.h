#include <ros/ros.h>

#include <nav_msgs/OccupancyGrid.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <bits/stdc++.h>
#include "yaml-cpp/yaml.h"


namespace map_server{

class MapServer{

    public:

    MapServer(ros::NodeHandle *, std::string);
    ~MapServer();

    private:

    ros::NodeHandle *_nh;
    ros::NodeHandle _private_nh;
    std::string _path;

    cv::Mat _map;

    ros::Publisher map_pub_;
    
    std::string _image_path;
    double _resolution;  
    double _occupied_thresh;
    double _free_thresh;
    double _negate;
    std::vector<double> _origin;

    int _height;
    int _width;

    std::string _window;

    void parseYAML();
    void getMap();
    void publishMap();
    
};


}