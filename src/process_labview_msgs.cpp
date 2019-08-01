#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>

#include <boost/bind.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

//#include <isi_hes_msgs/Spectra.h>
//#include <isi_hes_msgs/Ml.h>

#include "std_msgs/String.h"

#include <string>

typedef message_filters::sync_policies::ApproximateTime<
    std_msgs::String, std_msgs::String, std_msgs::String, std_msgs::String>
    policy_t;

class StringTime {
    public:
    std::string string;
    ros::Time time;
  public:
    StringTime(std::string _string, ros::Time _time){
        string = _string;
        time = _time;
    }
};

StringTime LVWavenumber = StringTime("",ros::Time(0));
StringTime LVIntensity = StringTime("",ros::Time(0));
StringTime LVMl = StringTime("",ros::Time(0));
StringTime LVHeader = StringTime("",ros::Time(0));

void ProcessLVMl(std::string ml_data, ros::Time time){

}

void labviewCb(StringTime wavenumber,StringTime intensity,
                StringTime &ml,StringTime &header)
{
    ros::Duration maxSyncTime(3);

    // Load message times into array
    ros::Time timeArray[4];
    timeArray[0] = wavenumber.time;
    timeArray[1] = intensity.time;
    timeArray[2] = ml.time;
    timeArray[3] = header.time;

    // Find oldest and youngest message in array
    ros::Time tempSmallest = timeArray[0];
    ros::Time tempLargest = timeArray[0];
    for(int i=0; i<4; i++) {
        if(tempSmallest>timeArray[i]) {
            tempSmallest=timeArray[i];
        }
        if(tempLargest>timeArray[i]) {
            tempLargest=timeArray[i];
        }
    }

    // Calcuate the time difference between oldest and youngest message
    ros::Duration timeSync = tempLargest - tempSmallest;
    // Calcuate the time difference of oldest message from now
    ros::Duration timeFromNow = ros::Time::now() - tempLargest;

    //Check messages are in sync with each other and aren't too old
    if (timeFromNow <= maxSyncTime){
        if (timeSync <= maxSyncTime){
            // Message are in sync
            ProcessLVMl(ml.string,ros::Time::now());
        } else {
            // Messages are too far out of sync
            std::cout << timeSync << std::endl;
            ROS_ERROR("messages are not in sync");
        }
    } else {
        // Oldest message is too old from current time
        std::cout << timeFromNow << std::endl;
    }

    /*
    ROS_INFO("Labview machine learning raw data: [%s]", msg->data.c_str());
    std::string raw_data = msg->data;
    std::string type_delimiter = ";";
    std::string data_delimiter = ",";
    int index = raw_data.find(type_delimiter);
    std::string materials_str = raw_data.substr(0, index);
    std::string values_str = raw_data.substr(index);
    ROS_INFO("Labview result materials: %s", materials_str.c_str());
    ROS_INFO("Labview result values: %s", values_str.c_str());
     */
}

void LVwavenumberCb(const std_msgs::String::ConstPtr &msg){
    LVWavenumber = StringTime(msg->data,ros::Time::now());
}

void LVintensityCb(const std_msgs::String::ConstPtr &msg){
    LVIntensity = StringTime(msg->data,ros::Time::now());
}

void LVmlCb(const std_msgs::String::ConstPtr &msg){
    LVMl = StringTime(msg->data,ros::Time::now());
}

void LVheaderCb(const std_msgs::String::ConstPtr &msg){
    LVHeader = StringTime(msg->data,ros::Time::now());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "process_labview_msgs");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    // Publisher creation
    // TODO finish creating these publishers
    ros::Publisher pub_ml = nh.advertise<std_msgs::String>("/isi/isi_hes_ml", 1000);
    ros::Publisher pub_spectra = nh.advertise<std_msgs::String>("/isi/isi_hes_spectra", 1000);

    // Subscribers creation
    ros::Subscriber sub_wavenumber = nh.subscribe("/isi/wavenumber", 1000, LVwavenumberCb);
    ros::Subscriber sub_intensity = nh.subscribe("/isi/intensity", 1000, LVintensityCb);
    ros::Subscriber sub_ml = nh.subscribe("/isi/ml", 1000, LVmlCb);
    ros::Subscriber sub_header = nh.subscribe("/isi/header", 1000, LVheaderCb);

    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        labviewCb(LVWavenumber,LVIntensity,LVMl,LVHeader);
        ros::spinOnce();
        r.sleep();
    }
}
