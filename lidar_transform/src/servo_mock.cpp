#include <ros/ros.h>
#include "std_msgs/Float32.h"

ros::Publisher g_trueAngle_pub;

float g_SetAngle = 0;

void sendAngle(float angle)
{
  std_msgs::Float32 AngMsg;
  AngMsg.data = angle;
  g_trueAngle_pub.publish(AngMsg);
}

void angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
  const float noise = (M_PI / 360);  // 0.5deg
  float random = ((float)rand()) / (float)RAND_MAX;
  g_SetAngle = msg->data + ((random * noise) - (noise / 2));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "servo_mock");
  ros::NodeHandle n;

  ros::Subscriber angleSubscriber = n.subscribe("setAngle", 1000, angleCallback);
  g_trueAngle_pub = n.advertise<std_msgs::Float32>("trueAngle", 1000);

  float trueAngle = 0;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    float angleError = g_SetAngle - trueAngle;

    const float changeRate = 0.1;

    if (angleError > changeRate)
    {
      angleError = changeRate;
    }

    if (angleError < -changeRate)
    {
      angleError = -changeRate;
    }

    trueAngle += angleError;

    sendAngle(trueAngle);

    ros::spinOnce();
    loop_rate.sleep();
  }
}