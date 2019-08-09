#include "ros/ros.h"
#include "std_msgs/Bool.h"

void pubTrigger(ros::Publisher bool_pub, bool en){
    std_msgs::Bool bool_msg;
    bool_msg.data = en;
    bool_pub.publish(bool_msg);
}

int main (int argc, char ** argv){
    ros::init(argc, argv, "simulate_trigger_labview");
    ros::NodeHandle nh;
    ros::NodeHandle p_nh("~");

    ros::Publisher trigger_pub = nh.advertise<std_msgs::Bool>("/isi_hes/trigger", 1, true);

    ros::Rate r(100);

    while (ros::ok()){
        std::cout << "Press any key to trigger...";
        std::cin.ignore();
        
        //trigger on then off (to simulate a button press)
        std::cout << "Trigger: True" << std::endl;
        pubTrigger(trigger_pub,true);
        ros::spinOnce();
        r.sleep();

        std::cout << "Trigger: False" << std::endl;
        pubTrigger(trigger_pub,false);
        ros::spinOnce();
        r.sleep();
    }
}