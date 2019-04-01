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

#include <cstdio>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "robot_self_filter/self_mask.h"

namespace robot_self_filter
{

class TestSelfFilter
{
public:

    TestSelfFilter(void) 
    {
	id_ = 1;
	vmPub_ = nodeHandle_.advertise<visualization_msgs::Marker>("visualization_marker", 10240);
	std::vector<robot_self_filter::LinkInfo> links;
	robot_self_filter::LinkInfo li;
        li.name="base_link";
        li.padding = .05;
        li.scale = 1.0;
        links.push_back(li);
	sf_ = new robot_self_filter::SelfMask(tf_, links);
    }

    ~TestSelfFilter(void)
    {
	delete sf_;
    }
    
    void gotIntersection(const Eigen::Vector3d &pt)
    {
	sendPoint(pt.x(), pt.y(), pt.z());
    }
    
    void sendPoint(double x, double y, double z)
    {
	visualization_msgs::Marker mk;

	mk.header.stamp = ros::Time::now();

	mk.header.frame_id = "base_link";

	mk.ns = "test_self_filter";
	mk.id = id_++;
	mk.type = visualization_msgs::Marker::SPHERE;
	mk.action = visualization_msgs::Marker::ADD;
	mk.pose.position.x = x;
	mk.pose.position.y = y;
	mk.pose.position.z = z;
	mk.pose.orientation.w = 1.0;

	mk.scale.x = mk.scale.y = mk.scale.z = 0.01;

	mk.color.a = 1.0;
	mk.color.r = 1.0;
	mk.color.g = 0.04;
	mk.color.b = 0.04;
	
	mk.lifetime = ros::Duration(10);
	
	vmPub_.publish(mk);
    }

    void run(void)
    {
        Cloud in;
        CloudModifier modifier(in);
        in.header.stamp = ros::Time::now();
        in.header.frame_id = "base_link";

        const size_t N = 500000;
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(N);
        CloudIter x_it(in, "x");
        CloudIter y_it(in, "y");
        CloudIter z_it(in, "z");
        for (size_t i = 0; i < N; ++i, ++x_it, ++y_it, ++z_it)
        {
            *x_it = float(uniform(1.5));
            *y_it = float(uniform(1.5));
            *z_it = float(uniform(1.5));
        }

	for (unsigned int i = 0 ; i < 1000 ; ++i)
	{
	    ros::Duration(0.001).sleep();
	    ros::spinOnce();
	}
	
	ros::WallTime tm = ros::WallTime::now();
	std::vector<int> mask;
	sf_->maskIntersection(in, "laser_tilt_mount_link", 0.01, mask, boost::bind(&TestSelfFilter::gotIntersection, this, _1) );
	//	sf_->maskContainment(in, mask);
	printf("%f points per second\n", (double)N/(ros::WallTime::now() - tm).toSec());

	
	int k = 0;
	for (unsigned int i = 0 ; i < mask.size() ; ++i)
	{
	    //	    bool v = sf_->getMaskContainment(in.points[i].x, in.points[i].y, in.points[i].z);
	    //	    if (v != mask[i]) 
	    //		ROS_ERROR("Mask does not match");	    
	    if (mask[i] != robot_self_filter::INSIDE) continue;
	    //	    sendPoint(in.points[i].x, in.points[i].y, in.points[i].z);
	    k++;
	}
	
	ros::spin();	
    }

protected:
    
    double uniform(double magnitude)
    {
	return (2.0 * drand48() - 1.0) * magnitude;
    }

    tf::TransformListener             tf_;
    robot_self_filter::SelfMask      *sf_;
    ros::Publisher                    vmPub_;
    ros::NodeHandle                   nodeHandle_;        
    int                               id_;
};
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_self_filter");

    robot_self_filter::TestSelfFilter t;
    sleep(1);
    t.run();
    
    return 0;
}

    
    
