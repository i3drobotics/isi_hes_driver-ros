#include <ros/ros.h>
#include <ros/console.h>
#include <ros/param.h>

#include <boost/bind.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <isi_hes_msgs/Spectra.h>
#include <isi_hes_msgs/Ml.h>

#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"

#include <string>
#include <stdlib.h> /* atof */

typedef message_filters::sync_policies::ApproximateTime<
    std_msgs::String, std_msgs::String, std_msgs::String, std_msgs::String>
    policy_t;

class StringTime
{
public:
    std::string string;
    ros::Time time;

public:
    StringTime(std::string _string, ros::Time _time)
    {
        string = _string;
        time = _time;
    }
};

template <typename T>
void printVector(const T &t)
{
    std::copy(t.cbegin(), t.cend(), std::ostream_iterator<typename T::value_type>(std::cout, ", "));
}

template <typename T>
void printVectorInVector(const T &t)
{
    std::for_each(t.cbegin(), t.cend(), printVector<typename T::value_type>);
}

bool initialised = false;
bool acquisition = false;

ros::Publisher pub_ml;
ros::Publisher pub_spectra;
ros::Publisher pub_range;
ros::Publisher pub_range_ok;
ros::Publisher pub_data_ok;

StringTime LVWavenumber = StringTime("", ros::Time(0));
StringTime LVIntensity = StringTime("", ros::Time(0));
StringTime LVMl = StringTime("", ros::Time(0));
StringTime LVHeader = StringTime("", ros::Time(0));
std::string parent_frame = "map";   // TODO: As a parameter
std::string sensor_frame = "raman"; // TODO: As a parameter

void delim_string_to_vector(std::string string, std::vector<double> &output_vector, std::string delimiter = ",")
{
    size_t pos = 0;
    std::string token_s;
    // Cycle though each token in string when splitting by ',' delimiter
    while ((pos = string.find(delimiter)) != std::string::npos)
    {
        // Get token by index of delimeter in string
        token_s = string.substr(0, pos);
        // Convert token to double
        double token_d = atof(token_s.c_str());
        // Add token to vector
        output_vector.push_back(token_d);
        // Remove token from original string
        string.erase(0, pos + delimiter.length());
    }
    double token_d = atof(string.c_str());
    output_vector.push_back(token_d);
}

void delim_string_to_vector(std::string string, std::vector<std::string> &output_vector, std::string delimiter = ",")
{
    size_t pos = 0;
    std::string token_s;
    // Cycle though each token in string when splitting by ',' delimiter
    while ((pos = string.find(delimiter)) != std::string::npos)
    {
        // Get token by index of delimeter in string
        token_s = string.substr(0, pos);
        // Add token to vector
        output_vector.push_back(token_s);
        // Remove token from original string
        string.erase(0, pos + delimiter.length());
    }
    output_vector.push_back(string);
}

void get_sample_and_similarity_from_ml_string(std::string input_ml, std::vector<std::string> &output_samples, std::vector<double> &output_similarity)
{
    //Split ml string samples string and similarity string
    std::vector<std::string> sam_sim;
    delim_string_to_vector(input_ml, sam_sim, ";");

    //Split samples string into samples string vector
    std::vector<std::string> samples;
    delim_string_to_vector(sam_sim.at(0), samples, ",");
    //Split similarity string into simiarity double vector
    std::vector<double> similarity;
    delim_string_to_vector(sam_sim.at(1), similarity, ",");

    output_samples = samples;
    output_similarity = similarity;
}

// Listens to tf to know the sensor position at sample time
bool getSensorAndSamplePose(const ros::Time &query_time,
                            geometry_msgs::PoseWithCovariance &sensor_pose,
                            geometry_msgs::PoseWithCovariance &sample_pose)
{
    // TODO: Get the position at sample time and not just current time
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped ts;
    try
    {
        ts = tfBuffer.lookupTransform(parent_frame, sensor_frame, query_time, ros::Duration(2.0));

        // message translationsensor_frame,
        sensor_pose.pose.position.x = ts.transform.translation.x;
        sensor_pose.pose.position.y = ts.transform.translation.y;
        sensor_pose.pose.position.z = ts.transform.translation.z;
        sensor_pose.pose.orientation.x = ts.transform.rotation.x;
        sensor_pose.pose.orientation.y = ts.transform.rotation.y;
        sensor_pose.pose.orientation.z = ts.transform.rotation.z;
        sensor_pose.pose.orientation.w = ts.transform.rotation.w;

        // TODO: Calculate sample pose
        sample_pose = sensor_pose;

        return true;
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return false;
    }
}

void PublishMl(StringTime ml, const geometry_msgs::PoseWithCovariance &sensor_pose, const geometry_msgs::PoseWithCovariance &sample_pose)
{
    isi_hes_msgs::Ml ml_msg;
    ml_msg.header.stamp = ml.time;
    ml_msg.header.frame_id = sensor_frame;

    // Obtain samples and similarity
    std::vector<std::string> samples;
    std::vector<double> similarity;
    get_sample_and_similarity_from_ml_string(ml.string, samples, similarity);
    ml_msg.samples = samples;
    ml_msg.similarity = similarity;
    ml_msg.sensor_pose = sensor_pose;
    ml_msg.sample_pose = sample_pose;
    pub_ml.publish(ml_msg);
}

void PublishSpectra(StringTime wavenumber, StringTime intensity, const geometry_msgs::PoseWithCovariance &sensor_pose, const geometry_msgs::PoseWithCovariance &sample_pose)
{
    isi_hes_msgs::Spectra spectra_msg;
    spectra_msg.header.stamp = wavenumber.time;
    spectra_msg.header.frame_id = sensor_frame;

    //Split wavenumber and intensity strings into vector doubles
    std::vector<double> wavenumber_vals, intensity_vals;
    delim_string_to_vector(wavenumber.string, wavenumber_vals, ",");
    delim_string_to_vector(intensity.string, intensity_vals, ",");
    spectra_msg.wavenumber = wavenumber_vals;
    spectra_msg.intensity = intensity_vals;

    spectra_msg.sensor_pose = sensor_pose;
    spectra_msg.sample_pose = sample_pose;

    pub_spectra.publish(spectra_msg);
}

void labviewCb(StringTime &wavenumber, StringTime &intensity,
               StringTime &ml, StringTime &header)
{
    ros::Duration maxSyncTime(3);

    // Load message times into array
    ros::Time timeArray[4];
    timeArray[0] = wavenumber.time;
    timeArray[1] = intensity.time;
    timeArray[2] = ml.time;
    timeArray[3] = header.time;

    // Find oldest and youngest message in array
    ros::Time tempOldest = timeArray[0];
    ros::Time tempYoungest = timeArray[0];
    int iYoungest = -1;
    int iOldest = -1;
    for (int i = 0; i < 4; i++)
    {
        if (timeArray[i] == ros::Time(0))
        {
            // message hasn't been initalised so return untill all data arives
            return;
        }
        if (tempOldest >= timeArray[i])
        {
            iOldest = i;
            tempOldest = timeArray[i];
        }
        if (tempYoungest >= timeArray[i])
        {
            iYoungest = i;
            tempYoungest = timeArray[i];
        }
    }

    // Deal with initialisation data
    std_msgs::Float64 range_msg;
    std_msgs::Bool range_ok_msg;
    std_msgs::Bool data_ok_msg;

    // TODO:  Get range, range_ok and data_ok from labview

    pub_range.publish(range_msg);
    pub_range_ok.publish(range_ok_msg);
    pub_data_ok.publish(data_ok_msg);

    // Deal with acquisition data only during acquisition
    if (acquisition && range_ok_msg.data && data_ok_msg.data)
    {
        ros::Duration maxSyncTime(3);

        // Load message times into array
        ros::Time timeArray[4];
        timeArray[0] = wavenumber.time;
        timeArray[1] = intensity.time;
        timeArray[2] = ml.time;
        timeArray[3] = header.time;

        // Find oldest and youngest message in array
        ros::Time tempOldest = timeArray[0];
        ros::Time tempYoungest = timeArray[0];
        bool isInitalised = false;
        for (int i = 0; i < 4; i++)
        {
            if (timeArray[i] == ros::Time(0))
            {
                // message hasn't been initalised so return untill all data arives
                return;
            }
            if (tempOldest > timeArray[i])
            {
                tempOldest = timeArray[i];
            }
            if (tempYoungest > timeArray[i])
            {
                tempYoungest = timeArray[i];
            }
        }

        // Calcuate the time difference between oldest and youngest message
        ros::Duration timeSync = tempYoungest - tempOldest;
        // Calcuate the time difference of oldest message from now
        ros::Duration timeFromNow = ros::Time::now() - tempYoungest;

        //Check messages are in sync with each other and aren't too old
        if (timeFromNow <= maxSyncTime)
        {
            if (timeSync <= maxSyncTime)
            {
                // Message are in sync
                // Obtain sensor and sample poses
                geometry_msgs::PoseWithCovariance sen_pose;
                geometry_msgs::PoseWithCovariance sam_pose;
                getSensorAndSamplePose(ros::Time().now(), sen_pose, sam_pose); // NOTE:  should be sync time
                PublishMl(ml, sen_pose, sam_pose);
                PublishSpectra(wavenumber, intensity, sen_pose, sam_pose);
            }
            else
            {
                // Messages are too far out of sync
                std::cout << timeSync << std::endl;
                ROS_ERROR("isi hes string messages are not in sync");
                return;
            }
        }
        else
        {
            // Oldest message is too old from current time
            std::cout << timeFromNow << std::endl;
            std::cout << "Youngest: t-" << tempYoungest << ", i-" << iYoungest << std::endl;
            ROS_ERROR("isi hes string messages are too old");
            return;
        }
    }
}

void LVwavenumberCb(const std_msgs::String::ConstPtr &msg)
{
    //ROS_INFO("Wavenumber received");
    LVWavenumber = StringTime(msg->data, ros::Time::now());
}

void LVintensityCb(const std_msgs::String::ConstPtr &msg)
{
    //ROS_INFO("Intensity received");
    LVIntensity = StringTime(msg->data, ros::Time::now());
}

void LVmlCb(const std_msgs::String::ConstPtr &msg)
{
    //ROS_INFO("ML received");
    LVMl = StringTime(msg->data, ros::Time::now());
}

void LVheaderCb(const std_msgs::String::ConstPtr &msg)
{
    //ROS_INFO("Header received");
    LVHeader = StringTime(msg->data, ros::Time::now());
}

bool InitialiseCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Initialising LabView Process");
    // TODO:  Add Labview comms
    initialised = true;
    return true;
}
bool AcquireCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Acquiring data");
    // TODO:  Add Labview comms
    acquisition = true;
    return true;
}
bool StopCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
    ROS_INFO("Stopping LabView Process");
    // TODO:  Add Labview comms
    acquisition = false;
    initialised = false;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "process_labview_msgs");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    // Publisher creation
    pub_range = nh.advertise<std_msgs::Float64>("/isi_hes/range", 10);
    pub_range_ok = nh.advertise<std_msgs::Bool>("/isi_hes/range_ok", 10);
    pub_data_ok = nh.advertise<std_msgs::Bool>("/isi_hes/data_ok", 10);
    // acquisition
    pub_ml = nh.advertise<isi_hes_msgs::Ml>("/isi_hes/isi_hes_ml", 1000);
    pub_spectra = nh.advertise<isi_hes_msgs::Spectra>("/isi_hes/isi_hes_spectra", 1000);

    // Subscribers creation
    ros::Subscriber sub_wavenumber = nh.subscribe("/isi_hes/wavenumber", 1000, LVwavenumberCb);
    ros::Subscriber sub_intensity = nh.subscribe("/isi_hes/intensity", 1000, LVintensityCb);
    ros::Subscriber sub_ml = nh.subscribe("/isi_hes/ml", 1000, LVmlCb);
    ros::Subscriber sub_header = nh.subscribe("/isi_hes/header", 1000, LVheaderCb);

    // Services creation
    ros::ServiceServer ser_init = nh.advertiseService("/isi_hes/initialise", InitialiseCb);
    ros::ServiceServer ser_acquire = nh.advertiseService("/isi_hes/acquire", AcquireCb);
    ros::ServiceServer ser_stop = nh.advertiseService("/isi_hes/stop", StopCb);

    ros::Rate r(1); // 10 hz
    while (ros::ok())
    {
        if (initialised)
        {
            labviewCb(LVWavenumber, LVIntensity, LVMl, LVHeader);
            ros::spinOnce();
            r.sleep();
        }
    }
}
