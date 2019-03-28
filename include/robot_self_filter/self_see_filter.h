/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROBOT_SELF_FILTER_SELF_SEE_FILTER_H_
#define ROBOT_SELF_FILTER_SELF_SEE_FILTER_H_

#include <filters/filter_base.h>
#include <robot_self_filter/self_mask.h>
#include <ros/console.h>

namespace robot_self_filter
{

/** \brief A filter to remove parts of the robot seen in a pointcloud
 *
 */

class SelfFilter: public filters::FilterBase<sensor_msgs::PointCloud2>
{
    
public:
  /** \brief Construct the filter */
  SelfFilter(ros::NodeHandle nh) : nh_(nh)
  {
    nh_.param<double>("min_sensor_dist", min_sensor_dist_, 0.01);
    double default_padding, default_scale;
    nh_.param<double>("self_see_default_padding", default_padding, .01);
    nh_.param<double>("self_see_default_scale", default_scale, 1.0);
    nh_.param<bool>("keep_organized", keep_organized_, false);
    std::vector<robot_self_filter::LinkInfo> links;	
    std::string link_names;
    
    if(!nh_.hasParam("self_see_links")) {
      ROS_WARN("No links specified for self filtering.");
    } else {     

      XmlRpc::XmlRpcValue ssl_vals;;
      
      nh_.getParam("self_see_links", ssl_vals);
      if(ssl_vals.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_WARN("Self see links need to be an array");
        
      } else {
        if(ssl_vals.size() == 0) {
          ROS_WARN("No values in self see links array");
        } else {
          for(int i = 0; i < ssl_vals.size(); i++) {
            robot_self_filter::LinkInfo li;
            
            if(ssl_vals[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
              ROS_WARN("Self see links entry %d is not a structure.  Stopping processing of self see links",i);
              break;
            }
            if(!ssl_vals[i].hasMember("name")) {
              ROS_WARN("Self see links entry %d has no name.  Stopping processing of self see links",i);
              break;
            } 
            li.name = std::string(ssl_vals[i]["name"]);
            if(!ssl_vals[i].hasMember("padding")) {
              ROS_DEBUG("Self see links entry %d has no padding.  Assuming default padding of %g",i,default_padding);
              li.padding = default_padding;
            } else {
              li.padding = ssl_vals[i]["padding"];
            }
            if(!ssl_vals[i].hasMember("scale")) {
              ROS_DEBUG("Self see links entry %d has no scale.  Assuming default scale of %g",i,default_scale);
              li.scale = default_scale;
            } else {
              li.scale = ssl_vals[i]["scale"];
            }
            links.push_back(li);
          }
        }      
      }
    }
    sm_ = new robot_self_filter::SelfMask(tf_, links);
    if (!sensor_frame_.empty())
      ROS_INFO("Self filter is removing shadow points for sensor in frame '%s'. Minimum distance to sensor is %f.", sensor_frame_.c_str(), min_sensor_dist_);
  }
    
  /** \brief Destructor to clean up
   */
  virtual ~SelfFilter(void)
  {
    delete sm_;
  }
  
  virtual bool configure(void)
  {
    // keep only the points that are outside of the robot
    // for testing purposes this may be changed to true
    nh_.param("invert", invert_, false);
    
    if (invert_)
      ROS_INFO("Inverting filter output");
	
    return true;
  }

  bool updateWithSensorFrame(const Cloud& data_in, Cloud& data_out, const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out);
  }
    
  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const Cloud& data_in, Cloud& data_out)
  {
    std::vector<int> keep(num_points(data_in));
    if(sensor_frame_.empty()) {
      sm_->maskContainment(data_in, keep);
    } else {
      sm_->maskIntersection(data_in, sensor_frame_, min_sensor_dist_, keep);
    }	
    fillResult(data_in, keep, data_out);
    return true;
  }

  bool updateWithSensorFrame(const Cloud& data_in, Cloud& data_out, Cloud& data_diff, const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out, data_diff);
  }

  /** \brief Update the filter and return the data seperately
   * \param data_in T array with length width
   * \param data_out T array with length width
   */
  virtual bool update(const Cloud& data_in, Cloud& data_out, Cloud& data_diff)
  {
    std::vector<int> keep(num_points(data_in));
    if(sensor_frame_.empty()) {
      sm_->maskContainment(data_in, keep);
    } else {
      sm_->maskIntersection(data_in, sensor_frame_, min_sensor_dist_, keep);
    }
    fillResult(data_in, keep, data_out);
    fillDiff(data_in, keep, data_diff);
    return true;
  }

  void fillDiff(const Cloud& data_in, const std::vector<int> &keep, Cloud& data_out)
  {
    std::vector<int> remove = keep;
    for (std::vector<int>::iterator it = remove.begin(); it != remove.begin(); ++it)
    {
      *it = (*it != OUTSIDE);
    }
    fillResult(data_in, remove, data_out);
  }

  void fillResult(const Cloud& data_in, const std::vector<int> &keep, Cloud& data_out)
  {
    const size_t np = num_points(data_in);

    if (keep_organized_)
    {
      // Start from a copy, set x, y, z to nan for points filtered out.
      data_out = data_in;
      CloudIter x_it(data_out, "x");
      CloudIter y_it(data_out, "y");
      CloudIter z_it(data_out, "z");
      for (size_t i = 0; i < np; ++i, ++x_it, ++y_it, ++z_it)
      {
        if (keep[i] != robot_self_filter::OUTSIDE) {
          *x_it = *y_it = *z_it = std::numeric_limits<float>::quiet_NaN();
        }
      }
      return;
    }

    // Iterate over the points and copy corresponding data chunks.
    data_out.header = data_in.header;
    data_out.fields = data_in.fields;
    data_out.point_step = data_in.point_step;
    data_out.height = 1;
    data_out.width = 0;
    data_out.data.resize(0);
    data_out.data.reserve(data_in.data.size());
    for (size_t i = 0; i < np; ++i)
    {
      if (keep[i] == robot_self_filter::OUTSIDE)
      {
        size_t from = (i / data_in.width) * data_in.row_step + (i % data_in.width) * data_in.point_step;
        size_t to = from + data_in.point_step;
        data_out.data.insert(data_out.data.end(), data_in.data.begin() + from, data_in.data.begin() + to);
        data_out.width++;
      }
    }
    data_out.row_step = data_out.width * data_out.point_step;
  }

  virtual bool updateWithSensorFrame(const std::vector<Cloud> &data_in, std::vector<Cloud> &data_out, const std::string& sensor_frame)
  {
    sensor_frame_ = sensor_frame;
    return update(data_in, data_out);
  }
  
  virtual bool update(const std::vector<Cloud> &data_in, std::vector<Cloud> &data_out)
  {
    bool result = true;
    data_out.resize(data_in.size());
    for (unsigned int i = 0 ; i < data_in.size() ; ++i)
      if (!update(data_in[i], data_out[i]))
        result = false;
    return true;
  }

  robot_self_filter::SelfMask* getSelfMask() {
    return sm_;
  }

  void setSensorFrame(const std::string& frame) {
    sensor_frame_ = frame;
  }
    
protected:
    
  tf::TransformListener tf_;
  robot_self_filter::SelfMask* sm_;
  
  ros::NodeHandle nh_;
  bool invert_;
  std::string sensor_frame_;
  double min_sensor_dist_;
  bool keep_organized_;
  
};

}

#endif //ROBOT_SELF_FILTER_SELF_SEE_FILTER_H_
