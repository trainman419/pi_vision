#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcl");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);

  PointCloud::Ptr msg (new PointCloud);
  msg->header.frame_id = "/world";
  msg->height = 1;
  msg->width = 3;
  msg->points.push_back (pcl::PointXYZ(1.0, 1.0, 1.0));
  msg->points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));
  msg->points.push_back (pcl::PointXYZ(2.0, 3.0, 1.0));
 
  ros::Rate loop_rate(4);
  while (nh.ok())
    {
      msg->header.stamp = ros::Time::now ();
      pub.publish (msg);
      ros::spinOnce ();
      loop_rate.sleep ();
    }
}
