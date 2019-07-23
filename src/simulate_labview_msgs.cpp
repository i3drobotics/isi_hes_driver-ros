#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>

#include "std_msgs/String.h"

#include <string>

ros::Publisher _pub_labview_ml, _pub_labview_spec;
std::string _labview_ml_topic = "/ISI/ML_DATA";
std::string _labview_spec_topic = "/ISI/SPEC_DATA";
std::string _example_ml_data = "Aluminium_Sulphate,Aluminium_Sulphate,Aluminium_Sulphate,Aluminium_Ammonium_Sulphate,Aluminium_Sulphate;1.00,0.96,0.96,0.96,0.94";
std::string _example_spec_data = "TODO";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulate_labview_msgs");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    std::string labview_ml_topic, labview_spec_topic, example_ml_data, example_spec_data;

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
    if (p_nh.getParam("example_ml_data", example_ml_data))
    {
        _example_ml_data = example_ml_data;
        ROS_INFO("example_ml_data: %s", _example_ml_data.c_str());
    }
    if (p_nh.getParam("example_spec_data", example_spec_data))
    {
        _example_spec_data = example_spec_data;
        ROS_INFO("example_spec_data: %s", _example_spec_data.c_str());
    }

    _pub_labview_ml = nh.advertise<std_msgs::String>(_labview_ml_topic, 1000);
    _pub_labview_spec = nh.advertise<std_msgs::String>(_labview_spec_topic, 1000);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        std_msgs::String msg_labview_ml, msg_labview_spec;

        msg_labview_ml.data = _example_ml_data;
        msg_labview_spec.data = _example_spec_data;

        _pub_labview_ml.publish(msg_labview_ml);
        _pub_labview_spec.publish(msg_labview_spec);

        ros::spinOnce();

        loop_rate.sleep();
    }
}
