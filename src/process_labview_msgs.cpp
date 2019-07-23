#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>

#include "std_msgs/String.h"

#include <string>

std::string _labview_ml_topic = "/ISI/ML_DATA";
std::string _labview_spec_topic = "/ISI/SPEC_DATA";

void strCallback_labview_ml(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Labview machine learning raw data: [%s]", msg->data.c_str());
    std::string raw_data = msg->data;
    std::string type_delimiter = ";";
    std::string data_delimiter = ",";
    int index = raw_data.find(type_delimiter);
    std::string materials_str = raw_data.substr(0, index);
    std::string values_str = raw_data.substr(index, raw_data.end);
    ROS_INFO("Labview result materials: %s", materials_str.c_str());
    ROS_INFO("Labview result values: %s", values_str.c_str());
}

void strCallback_labview_spec(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("Labview spectrum topic raw data: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "process_labview_msgs");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string labview_ml_topic, labview_spec_topic;

    //Get parameters
    if (p_nh.getParam("labview_ml_topic", labview_ml_topic))
    {
        _labview_ml_topic = labview_ml_topic;
        ROS_INFO("labview_ml_topic: %s", _labview_ml_topic.c_str());
    }
    if (p_nh.getParam("labview_spec_topic", labview_spec_topic))
    {
        _labview_spec_topic = labview_spec_topic;
        ROS_INFO("labview_spec_topic: %s", _labview_spec_topic.c_str());
    }

    ros::Subscriber sub_labview_ml = nh.subscribe(_labview_ml_topic, 1000, strCallback_labview_ml);
    ros::Subscriber sub_labview_spec = nh.subscribe(_labview_spec_topic, 1000, strCallback_labview_spec);

    ros::spin();
}
