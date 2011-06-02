#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZ PointT;

int main(int argc, char** argv)
{
  ros::init (argc, argv, "pub_pcd");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<PointCloud> ("points2", 1);
  PointCloud::Ptr msg (new PointCloud);

  PointCloud::Ptr cloud (new PointCloud);
  PointCloud::Ptr cloud_filtered (new PointCloud);

  //  sensor_msgs::PointCloud2::Ptr cloud (new sensor_msgs::PointCloud2 ()), cloud_filtered (new sensor_msgs::PointCloud2 ());

  pcl::PCDReader reader;

  // Read in the cloud data
  reader.read ("data/table_scene_mug_stereo_textured.pcd", *cloud);
  ROS_INFO ("PointCloud before filtering: %d data points (%s).", cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str ());

  pcl::VoxelGrid<PointT> vox;
  vox.setInputCloud (cloud);
  vox.setLeafSize (0.01, 0.01, 0.01);
  vox.filter (*cloud_filtered);

  ROS_INFO ("PointCloud after filtering: %d data points (%s).", cloud_filtered->width * cloud_filtered->height, pcl::getFieldsList (*cloud_filtered).c_str ());

  msg->header.frame_id = "/world";
  //msg->height = 1;
  //msg->width = cloud->points.size;
  msg->points = cloud_filtered->points;

  ros::Rate loop_rate(1);
  while (nh.ok())
    {
      msg->header.stamp = ros::Time::now ();
      pub.publish (msg);
      ros::spinOnce ();
      loop_rate.sleep ();
    }
}
