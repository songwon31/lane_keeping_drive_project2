#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "lidar_test/Obstacle.h"

lidar_test::Obstacle msg;
float distance[505] = {0, };

void lidar_callback(const sensor_msgs::LaserScan& scan_in)
{
    msg.obstacle = false;
    msg.right = 0;
    msg.left = 0;
    int8_t front_count = 0;
    int8_t left_count = 0;
    int8_t right_count = 0;
    for (int i; i < 505; i++)
    {
        if ((i<40 || i>=475) && (scan_in.ranges[i] <= 0.6 && scan_in.ranges[i] > 0.0))
        {
            front_count += 1;
        }
        else if (i < 125)
        {
            left_count++;
        }
        else if (375 < i)
        {
            right_count++;
        }

        if (front_count > 3)
        {
            msg.obstacle = true;
        }
    }
    msg.left = left_count;
    msg.right = right_count;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_test", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, &lidar_callback);
    ros::Publisher lidar_pub = nh.advertise<lidar_test::Obstacle>("/obstacle", 10);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        lidar_pub.publish(msg);
        loop_rate.sleep();                                                                                                                                                                              
    }

    return 0;
}
