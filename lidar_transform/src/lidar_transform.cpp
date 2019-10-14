#include <ros/ros.h>

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"
#include "sensor_msgs/ChannelFloat32.h"
#include "std_msgs/Float32.h"

ros::Publisher g_scan3d_pub;
ros::Publisher g_setAngle_pub;

float g_currentAngle = 0.0;
int g_seq = 0;
bool requestNextAngle = true;
float g_setAngle = 0;

sensor_msgs::ChannelFloat32 g_channelIntensity;
std::vector<geometry_msgs::Point32> g_pointcloudPoints;

void sendPointCloud()
{
  sensor_msgs::PointCloud scan3dMsg;
  scan3dMsg.header.seq = g_seq++;
  scan3dMsg.header.stamp = ros::Time::now();
  scan3dMsg.header.frame_id = "scan_3d";

  scan3dMsg.points = g_pointcloudPoints;
  scan3dMsg.channels.clear();
  scan3dMsg.channels.push_back(g_channelIntensity);

  g_scan3d_pub.publish(scan3dMsg);
}

geometry_msgs::Point32 angularDistanceToPoint(float pan, float tilt, float distance)
{
  tilt = M_PI - tilt;
  geometry_msgs::Point32 point;
  point.x = -(sin(pan) * distance);  // invert to miror image.
  point.y = cos(pan) * sin(tilt) * distance;
  point.z = cos(tilt) * cos(pan) * distance;
  return point;
}

void clearPointcloud()
{
  g_channelIntensity.values.clear();
  g_pointcloudPoints.clear();
}

void addScanToPointcloud(float tilt, sensor_msgs::LaserScan scan)
{
  uint16_t pointCount = -1;
  for (float i = scan.angle_min; i < scan.angle_max; i += scan.angle_increment)
  {
    pointCount++;  // 0 to 359

    if (!(i < (M_PI / 2) || i > M_PI + (M_PI / 2)))  // only use front 180 deg of the sensor
      continue;

    if (!(scan.ranges[pointCount] > 0.01 && scan.ranges[pointCount] < 3))  // only use scan data more than 10 cm and 3
                                                                           // m. only needed because my lidar node is
                                                                           // bad.
      continue;

    g_pointcloudPoints.push_back(angularDistanceToPoint(
        i, tilt, scan.ranges[pointCount]));  // calculede 3d point from tilt angle, pan angle and distance.
    g_channelIntensity.values.push_back(scan.intensities[pointCount]);
  }
}

void sendAngle(float angle)
{
  std_msgs::Float32 AngMsg;
  AngMsg.data = angle;
  g_setAngle_pub.publish(AngMsg);
}

void angleCallback(const std_msgs::Float32::ConstPtr& msg)
{
  g_currentAngle = msg->data;
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  sensor_msgs::LaserScan scan = (sensor_msgs::LaserScan)*msg;

  if (g_currentAngle > g_setAngle + (M_PI / 180) || g_currentAngle < g_setAngle - (M_PI / 180))
    return;

  addScanToPointcloud(g_currentAngle, scan);
  sendPointCloud();
  requestNextAngle = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidar_transformer");
  ros::NodeHandle n;

  ros::Subscriber scanSubscriber = n.subscribe("scan", 1000, scanCallback);
  ros::Subscriber angleSubscriber = n.subscribe("trueAngle", 1000, angleCallback);
  g_scan3d_pub = n.advertise<sensor_msgs::PointCloud>("scan3d", 1000);
  g_setAngle_pub = n.advertise<std_msgs::Float32>("setAngle", 1000);

  g_channelIntensity.name = "intensity";

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    if (requestNextAngle)
    {
      requestNextAngle = false;
      g_setAngle += (M_PI / 180);

      if (g_setAngle > M_PI)
        g_setAngle = 0;

      sendAngle(g_setAngle);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
