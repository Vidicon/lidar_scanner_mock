#include <ros/ros.h>
#include "std_msgs/Float32.h"

#include "sensor_msgs/LaserScan.h"

ros::Publisher g_scan_pub;

float g_SetAngle = 0;
int g_seq = 0;

void sendScan()
{
  sensor_msgs::LaserScan scanMsg;

  scanMsg.header.frame_id = "Lidar";
  scanMsg.header.seq = g_seq++;
  scanMsg.header.stamp = ros::Time::now();

  scanMsg.angle_increment = (M_PI / 180);
  scanMsg.angle_min = -M_PI;
  scanMsg.angle_max = M_PI;

  scanMsg.range_max = 6.0;
  scanMsg.range_min = 0.1;

  for (int i = 0; i < 360; i++)
  {
    float random = ((float)rand()) / (float)RAND_MAX;
    scanMsg.ranges.push_back(i * 0.01 + (random * 0.1));
    scanMsg.intensities.push_back(i);
  }

  g_scan_pub.publish(scanMsg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_mock");
  ros::NodeHandle n;

  g_scan_pub = n.advertise<sensor_msgs::LaserScan>("scan", 1000);

  float angle = 0;

  ros::Rate loop_rate(5);
  while (ros::ok())
  {
    sendScan();

    ros::spinOnce();
    loop_rate.sleep();
  }
}