/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#include <ros/ros.h>
#include <sstream>
#include "robot_self_filter/self_see_filter.h"
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

namespace robot_self_filter
{
class SelfFilter
{
public:

  SelfFilter(void): nh_("~"), subscribing_(false)
  {
    nh_.param<std::string>("sensor_frame", sensor_frame_, std::string());
    nh_.param("use_rgb", use_rgb_, false);
    nh_.param("max_queue_size", max_queue_size_, 10);
    if (use_rgb_) 
    {
      self_filter_rgb_ = new filters::SelfFilter<pcl::PointXYZRGB>(nh_);
    }
    else 
    {
      self_filter_ = new filters::SelfFilter<pcl::PointXYZ>(nh_);
    }
    ros::SubscriberStatusCallback connect_cb
      = boost::bind( &SelfFilter::connectionCallback, this, _1);
    
    if (use_rgb_) 
    {
      self_filter_rgb_->getSelfMask()->getLinkNames(frames_);
    }
    else 
    {
      self_filter_->getSelfMask()->getLinkNames(frames_);
    }
    pointCloudPublisher_ = root_handle_.advertise<sensor_msgs::PointCloud2>("cloud_out", 1,
                                                                            connect_cb, connect_cb);
  }
    
  ~SelfFilter(void)
  {
    if (self_filter_) 
    {
      delete self_filter_;
    }
    if (self_filter_rgb_)
    {
      delete self_filter_rgb_;
    }
  }
    
private:

  void connectionCallback(const ros::SingleSubscriberPublisher& pub)
  {
    if (pointCloudPublisher_.getNumSubscribers() > 0) {
      if (!subscribing_) {
        subscribe();
        subscribing_ = true;
      }
    }
    else {
      if (subscribing_) {
        unsubscribe();
        subscribing_ = false;
      }
    }
  }
  
  void subscribe() {
    if(frames_.empty())
    {
      ROS_DEBUG("No valid frames have been passed into the self filter. Using a callback that will just forward scans on.");
      no_filter_sub_ = root_handle_.subscribe<sensor_msgs::PointCloud2>("cloud_in", 1, boost::bind(&SelfFilter::noFilterCallback, this, _1));
    }
    else
    {
      ROS_DEBUG("Valid frames were passed in. We'll filter them.");
      sub_.subscribe(root_handle_, "cloud_in", max_queue_size_);
      mn_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(sub_, tf_, "", max_queue_size_));
      mn_->setTargetFrames(frames_);
      mn_->registerCallback(boost::bind(&SelfFilter::cloudCallback, this, _1));
    }
  }

  void unsubscribe() {
    if (frames_.empty()) {
      no_filter_sub_.shutdown();
    }
    else {
      sub_.unsubscribe();
    }
  }

  void noFilterCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud){
    pointCloudPublisher_.publish(cloud);
    ROS_DEBUG("Self filter publishing unfiltered frame");
  }
    
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud2) 
  {
    ROS_DEBUG("Got pointcloud that is %f seconds old", (ros::Time::now() - cloud2->header.stamp).toSec());
    std::vector<int> mask;
    ros::WallTime tm = ros::WallTime::now();

    
    sensor_msgs::PointCloud2 out2;
    int input_size = 0;
    int output_size = 0;
    if (use_rgb_)
    {
      typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud2, *cloud);
      pcl::PointCloud<pcl::PointXYZRGB> out;
      self_filter_rgb_->updateWithSensorFrame(*cloud, out, sensor_frame_);
      pcl::toROSMsg(out, out2);
      out2.header.stamp = cloud2->header.stamp;
      input_size = cloud->points.size();
      output_size = out.points.size();
    }
    else
    {
      typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*cloud2, *cloud);
      pcl::PointCloud<pcl::PointXYZ> out;
      self_filter_->updateWithSensorFrame(*cloud, out, sensor_frame_);
      pcl::toROSMsg(out, out2);
      out2.header.stamp = cloud2->header.stamp;
      input_size = cloud->points.size();
      output_size = out.points.size();
    }
      
    double sec = (ros::WallTime::now() - tm).toSec();
    pointCloudPublisher_.publish(out2);
    ROS_DEBUG("Self filter: reduced %d points to %d points in %f seconds", input_size, output_size, sec);

  }
  
  tf::TransformListener                                 tf_;
  //tf::MessageNotifier<robot_self_filter::PointCloud>           *mn_;
  ros::NodeHandle                                       nh_, root_handle_;

  boost::shared_ptr<tf::MessageFilter<sensor_msgs::PointCloud2> >          mn_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;

  filters::SelfFilter<pcl::PointXYZ> *self_filter_;
  filters::SelfFilter<pcl::PointXYZRGB> *self_filter_rgb_;
  std::string sensor_frame_;
  bool use_rgb_;
  bool subscribing_;
  std::vector<std::string> frames_;
  
  ros::Publisher                                        pointCloudPublisher_;
  ros::Subscriber                                       no_filter_sub_;
  int max_queue_size_;
};
}
    
int main(int argc, char **argv)
{
  ros::init(argc, argv, "self_filter");
  
  ros::NodeHandle nh("~");
  robot_self_filter::SelfFilter s;
  ros::spin();
    
  return 0;
}
