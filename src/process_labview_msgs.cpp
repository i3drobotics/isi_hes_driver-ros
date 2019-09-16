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

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>

#include <string>
#include <stdlib.h> /* atof */

bool range_state_, check_state_ = false;

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

class LVCommands
{
public:
    LVCommands(ros::NodeHandle *nh)
    {   
        //Initailise services
        ser_wait_ = nh->advertiseService("/isi_hes/wait", &LVCommands::WaitCb, this);
        ser_range_ = nh->advertiseService("/isi_hes/get_range", &LVCommands::RangeCb, this);
        ser_check_ = nh->advertiseService("/isi_hes/check", &LVCommands::CheckCb, this);
        ser_acquire_ = nh->advertiseService("/isi_hes/acquire", &LVCommands::AcquireCb, this);
        ser_stop_ = nh->advertiseService("/isi_hes/stop", &LVCommands::StopCb, this);

        //Initialise publisher
        pub_LVCommand = nh->advertise<std_msgs::Int32>("isi_hes/command", 10);
    }

private:
    //Define command variables
    const int LV_COMMAND_WAIT = 1; //Not used (For labview state machine)
    const int LV_COMMAND_RANGE = 2;
    const int LV_COMMAND_CHECK = 3;
    const int LV_COMMAND_ACQR = 4;
    const int LV_COMMAND_STOP = 5;
    const int LV_COMMAND_LASER_ON = 6;  //Not used (For labview state machine)
    const int LV_COMMAND_LASER_OFF = 7; //Not used (For labview state machine)

    ros::Publisher pub_LVCommand;
    ros::ServiceServer ser_wait_, ser_range_, ser_check_, ser_acquire_, ser_stop_;

    bool send_LVCommand(int command)
    {
        std_msgs::Int32 cmd_msg;
        if (command >= 1 && command <= 7)
        {
            cmd_msg.data = command;
            pub_LVCommand.publish(cmd_msg);
            return true;
        }
        else
        {
            ROS_ERROR("Invalid ISI probe command. MUST be 1-7.");
            return false;
        }
    }
    bool WaitCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO("Requesting wait for command in ISI HES Probe");
        // Send range command to ISI Probe labview
        send_LVCommand(LV_COMMAND_WAIT);
        return true;
    }
    bool RangeCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO("Requesting distance of ISI HES Probe");
        // Send range command to ISI Probe labview
        send_LVCommand(LV_COMMAND_RANGE);
        // Small sleep for labview to register command
        ros::Duration(0.5).sleep();
        // Send wait command to set ISI Probe labview state after function call
        send_LVCommand(LV_COMMAND_WAIT);
        return true;
    }
    bool CheckCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO("Requesting check of valid position/state of ISI HES Probe");
        // Send check command to ISI Probe labview
        send_LVCommand(LV_COMMAND_CHECK);
        // Small sleep for labview to register command
        ros::Duration(0.5).sleep();
        // Send wait command to set ISI Probe labview state after function call
        send_LVCommand(LV_COMMAND_WAIT);
        return true;
    }
    bool AcquireCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        //if (range_state_ && check_state_)
        if (true)
        {
            ROS_INFO("Requesting acquisition of ISI HES Probe data");
            // Send acquire command to ISI Probe labview
            send_LVCommand(LV_COMMAND_ACQR);
            // Small sleep for labview to register command
            ros::Duration(0.5).sleep();
            // Send wait command to set ISI Probe labview state after function call
            send_LVCommand(LV_COMMAND_WAIT);
        } else {
            ROS_ERROR("MUST run range and check test before acquire!");
            return false;
        }
        return true;
    }
    bool StopCb(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
    {
        ROS_INFO("Requesting stop ISI HES Probe Process");
        // Send stop command to ISI Probe labview
        send_LVCommand(LV_COMMAND_STOP);
        // Small sleep for labview to register command
        ros::Duration(0.5).sleep();
        // Send wait command to set ISI Probe labview state after function call
        send_LVCommand(LV_COMMAND_WAIT);
        return true;
    }
};

class LVControl
{
public:
    LVControl(ros::NodeHandle *nh, std::string parent_frame = "map", std::string sensor_frame = "raman")
    {
        ROS_INFO("Starting ISI LabView process node...");
        parent_frame_ = parent_frame;
        sensor_frame_ = sensor_frame;

        // Publisher creation
        pub_ml_ = nh->advertise<isi_hes_msgs::Ml>("/isi_hes/ml", 1000);
        pub_spectra_ = nh->advertise<isi_hes_msgs::Spectra>("/isi_hes/spectra", 1000);

        // Subscribers creation
        sub_wavenumber_ = nh->subscribe("/isi_hes/wavenumber_raw", 1000, &LVControl::LVwavenumberCb, this);
        sub_intensity_ = nh->subscribe("/isi_hes/intensity_raw", 1000, &LVControl::LVintensityCb, this);
        sub_ml_ = nh->subscribe("/isi_hes/ml_raw", 1000, &LVControl::LVmlCb, this);
        sub_header_ = nh->subscribe("/isi_hes/header_raw", 1000, &LVControl::LVheaderCb, this);
        sub_range_ = nh->subscribe("/isi_hes/distance_raw", 1000, &LVControl::LVRangeCb, this);
        ROS_INFO("ISI LabView process node ready. You can now start the labview program.");
    }

private:
    ros::Subscriber sub_wavenumber_, sub_intensity_, sub_ml_, sub_header_, sub_range_;

    const int LV_STATE_UNKNOWN = -1;
    const int LV_STATE_READY = 0;
    const int LV_STATE_RANGING = 1;
    const int LV_STATE_CHECKING = 2;
    const int LV_STATE_ACQUIRING = 3;

    int LV_STATE_ = LV_STATE_UNKNOWN;

    StringTime LVWavenumber_ = StringTime("", ros::Time(0));
    StringTime LVIntensity_ = StringTime("", ros::Time(0));
    StringTime LVMl_ = StringTime("", ros::Time(0));
    StringTime LVHeader_ = StringTime("", ros::Time(0));
    StringTime LVRange_ = StringTime("", ros::Time(0));
    std::string parent_frame_ = "map";   // TODO: As a parameter
    std::string sensor_frame_ = "hes_probe_base_link"; // TODO: As a parameter
    std::string sample_frame_ = "hes_probe_standoff"; // TODO: As a paramter
    ros::Publisher pub_ml_, pub_spectra_, pub_range_;

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
            ts = tfBuffer.lookupTransform(parent_frame_, sensor_frame_, query_time, ros::Duration(2.0));

            // message translationsensor_frame,
            sensor_pose.pose.position.x = ts.transform.translation.x;
            sensor_pose.pose.position.y = ts.transform.translation.y;
            sensor_pose.pose.position.z = ts.transform.translation.z;
            sensor_pose.pose.orientation.x = ts.transform.rotation.x;
            sensor_pose.pose.orientation.y = ts.transform.rotation.y;
            sensor_pose.pose.orientation.z = ts.transform.rotation.z;
            sensor_pose.pose.orientation.w = ts.transform.rotation.w;

            ts = tfBuffer.lookupTransform(parent_frame_, sample_frame_, query_time, ros::Duration(2.0));
            // Calculate sample pose
            sample_pose.pose.position.x = ts.transform.translation.x;
            sample_pose.pose.position.y = ts.transform.translation.y;
            sample_pose.pose.position.z = ts.transform.translation.z;
            sample_pose.pose.orientation.x = ts.transform.rotation.x;
            sample_pose.pose.orientation.y = ts.transform.rotation.y;
            sample_pose.pose.orientation.z = ts.transform.rotation.z;
            sample_pose.pose.orientation.w = ts.transform.rotation.w;

            return true;
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return false;
        }
        catch (...){
            ROS_ERROR("Failed to create transfrom in isi hes node");
            return false;
        }
    }

    void PublishMl(StringTime ml, const geometry_msgs::PoseWithCovariance &sensor_pose, const geometry_msgs::PoseWithCovariance &sample_pose)
    {
      ROS_INFO("Sending machine learning result");
        isi_hes_msgs::Ml ml_msg;
        ml_msg.header.stamp = ml.time;
        ml_msg.header.frame_id = sensor_frame_;

        // Obtain samples and similarity
        std::vector<std::string> samples;
        std::vector<double> similarity;
        get_sample_and_similarity_from_ml_string(ml.string, samples, similarity);
        ml_msg.samples = samples;
        ml_msg.similarity = similarity;
        ml_msg.sensor_pose = sensor_pose;
        ml_msg.sample_pose = sample_pose;
        pub_ml_.publish(ml_msg);
    }

    void PublishSpectra(StringTime wavenumber, StringTime intensity, const geometry_msgs::PoseWithCovariance &sensor_pose, const geometry_msgs::PoseWithCovariance &sample_pose)
    {
      ROS_INFO("Sending spectra");
        isi_hes_msgs::Spectra spectra_msg;
        spectra_msg.header.stamp = wavenumber.time;
        spectra_msg.header.frame_id = sensor_frame_;

        //Split wavenumber and intensity strings into vector doubles
        std::vector<double> wavenumber_vals, intensity_vals;
        delim_string_to_vector(wavenumber.string, wavenumber_vals, ",");
        delim_string_to_vector(intensity.string, intensity_vals, ",");
        spectra_msg.wavenumber = wavenumber_vals;
        spectra_msg.intensity = intensity_vals;

        spectra_msg.sensor_pose = sensor_pose;
        spectra_msg.sample_pose = sample_pose;

        pub_spectra_.publish(spectra_msg);
    }

    bool checkStringTimeSync(float maxSyncTime, std::vector<StringTime> string_time_vector)
    {
        ros::Duration maxSyncDuration(maxSyncTime);

        int vector_size = string_time_vector.size();

        // Load message times into array
        ros::Time timeArray[vector_size];
        for (int i = 0; i < vector_size; i++)
        {
            timeArray[i] = string_time_vector[i].time;
        }

        // Find oldest and youngest message in array
        ros::Time tempOldest = timeArray[0];
        ros::Time tempYoungest = timeArray[0];
        int iYoungest = -1;
        int iOldest = -1;
        for (int i = 0; i < vector_size; i++)
        {
            if (timeArray[i] == ros::Time(0))
            {
                // message hasn't been initalised so return untill all data arives
                return false;
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

        // Calcuate the time difference between oldest and youngest message
        ros::Duration timeSync = tempYoungest - tempOldest;
        // Calcuate the time difference of oldest message from now
        ros::Duration timeFromNow = ros::Time::now() - tempYoungest;

        //Check messages are in sync with each other and aren't too old
        if (timeFromNow <= maxSyncDuration)
        {
            if (timeSync <= maxSyncDuration)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    void labviewCb(StringTime &wavenumber, StringTime &intensity,
                   StringTime &ml, StringTime &header)
    {
        // Deal with acquisition data only during acquisition
        //if (range_state_ && check_state_)
        if (true)
        {
            // Message are in sync
            // Obtain sensor and sample poses
            ROS_INFO("ISI HES data messages received. Publishing in sucsink topics /ml and /");
            geometry_msgs::PoseWithCovariance sen_pose;
            geometry_msgs::PoseWithCovariance sam_pose;
            getSensorAndSamplePose(ros::Time().now(), sen_pose, sam_pose); // NOTE:  should be sync time
            PublishMl(ml, sen_pose, sam_pose);
            PublishSpectra(wavenumber, intensity, sen_pose, sam_pose);
        } else {
            ROS_ERROR("MUST run range and check test before acquire!");
        }
    }

    void LVRangeCb(const std_msgs::String::ConstPtr &msg)
    {
        std::vector<std::string> rangeVector;
        delim_string_to_vector(msg->data, rangeVector, ";");
        ROS_INFO("Range response: %s",msg->data.c_str());
        if (rangeVector.size() > 0){
            std::string range_status = rangeVector[0];
            bool isValid = true;
            if (range_status == "out_of_range"){
                range_state_ = true; //TODO CHANGE THIS BACK TO FALSE
            } else if (range_status == "in_range"){
                range_state_ = true;
            } else if (range_status == "check_failed"){
                check_state_ = true; //TODO CHANGE THIS BACK TO FALSE
            } else if (range_status == "check_passed"){
                check_state_ = true;
            } else {
                isValid = false;
                ROS_ERROR("Invalid range status in message (/isi_hes/distance)");
            }
            if (isValid){
                std::string range_value_s = rangeVector[0];
                double range_value = atof(range_value_s.c_str());
            }
        }
    }

    void LVwavenumberCb(const std_msgs::String::ConstPtr &msg)
    {
        ROS_INFO("Wavenumber received");
        LVWavenumber_ = StringTime(msg->data, ros::Time::now());

        std::vector<StringTime> LVMessages {LVWavenumber_,LVIntensity_,LVMl_,LVHeader_};
        if (checkStringTimeSync(3,LVMessages))
        {
            labviewCb(LVWavenumber_, LVIntensity_, LVMl_, LVHeader_);
        }
    }

    void LVintensityCb(const std_msgs::String::ConstPtr &msg)
    {
        ROS_INFO("Intensity received");
        LVIntensity_ = StringTime(msg->data, ros::Time::now());

        std::vector<StringTime> LVMessages {LVWavenumber_,LVIntensity_,LVMl_,LVHeader_};
        if (checkStringTimeSync(3,LVMessages))
        {
            labviewCb(LVWavenumber_, LVIntensity_, LVMl_, LVHeader_);
        }
    }

    void LVmlCb(const std_msgs::String::ConstPtr &msg)
    {
        ROS_INFO("ML received");
        LVMl_ = StringTime(msg->data, ros::Time::now());

        std::vector<StringTime> LVMessages {LVWavenumber_,LVIntensity_,LVMl_,LVHeader_};
        if (checkStringTimeSync(3,LVMessages))
        {
            labviewCb(LVWavenumber_, LVIntensity_, LVMl_, LVHeader_);
        }
    }

    void LVheaderCb(const std_msgs::String::ConstPtr &msg)
    {
        ROS_INFO("Header received");
        LVHeader_ = StringTime(msg->data, ros::Time::now());

        std::vector<StringTime> LVMessages {LVWavenumber_,LVIntensity_,LVMl_,LVHeader_};
        if (checkStringTimeSync(3,LVMessages))
        {
            labviewCb(LVWavenumber_, LVIntensity_, LVMl_, LVHeader_);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "process_labview_msgs");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    LVCommands LV_commands = LVCommands(&nh);
    LVControl LV_control = LVControl(&nh);

    ros::Rate r(1); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
    }
}
