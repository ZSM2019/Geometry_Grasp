/*#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

 
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}*/
/***********************************************************
关于使用sensor_msgs/PointCloud2，
***********************************************************/

#include <ros/ros.h>
// PCL 的相关的头文件
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//滤波的头文件
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
//申明发布器
ros::Publisher pub;
 //回调函数
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)  //特别注意的是这里面形参的数据格式
{
 // 声明存储原始数据与滤波后的数据的点云的 格式
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;    //原始的点云的数据格式
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); 
  pcl::PCLPointCloud2 cloud_filtered;     //存储滤波后的数据格式
  pcl::PCLPointCloud2::Ptr cloud_filtered2 (new pcl::PCLPointCloud2);
 // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZRGBA>());
  // 转化为PCL中的点云的数据格式
  pcl_conversions::toPCL(*input, *cloud);
  //pcl_conversions::toPCL(*input, cloud2);
  pcl::fromPCLPointCloud2 (*cloud, *cloud2);
  pcl::PointCloud<pcl::PointXYZ> cloud1; 

  pcl::io::savePCDFile<pcl::PointXYZRGBA>("/home/zsm/catkin_ws/src/rospcl/src/rgbtest222.pcd",*cloud2);
// pcl::io::savePCDFileASCII("/home/zsm/catkin_ws/src/rospcl/src/222.pcd",*cloud);

  // 进行一个滤波处理
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;   //实例化滤波
  sor.setInputCloud (cloudPtr);     //设置输入的滤波
  sor.setLeafSize (0.01, 0.01, 0.01);   //设置体素网格的大小
  sor.filter (*cloud_filtered2);      //存储滤波后的点云
  
  pcl::fromPCLPointCloud2 (*cloud_filtered2, *cloud2); 

  pcl::io::savePCDFile("/home/zsm/catkin_ws/src/rospcl/src/rgbtest111.pcd",*cloud_filtered2);

  viewer->addPointCloud(cloud2, "xxx");
 // viewer->addPointCloud(cloud2, xxx);
  viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
  viewer->spinOnce();
  viewer->removePointCloud("xxx");
  
  // 再将滤波后的点云的数据格式转换为ROS下的数据格式发布出去
  sensor_msgs::PointCloud2 output;   //声明的输出的点云的格式
  pcl_conversions::fromPCL(cloud_filtered, output);    //第一个参数是输入，后面的是输出

  //发布命令
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // 初始化 ROS节点
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;   //声明节点的名称

  // 为接受点云数据创建一个订阅节点
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  //创建ROS的发布节点
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // 回调
 // ros::spin ();
 // viewer->setBackgroundColor (1, 1, 1); //背景
  //viewer->addPointCloud(cloud2, "bunny");
  while(ros::ok())
  {
    ros::spinOnce();
  }

}
