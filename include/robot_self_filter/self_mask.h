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

#ifndef ROBOT_SELF_FILTER_SELF_MASK_
#define ROBOT_SELF_FILTER_SELF_MASK_

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <robot_self_filter/bodies.h>
#include <tf/transform_listener.h>
#include <boost/bind.hpp>
#include <boost/filesystem.hpp>
#include <string>
#include <vector>

#include <urdf/model.h>
#include <resource_retriever/retriever.h>

namespace robot_self_filter
{

    /** \brief The possible values of a mask computed for a point */
    enum
    {
	INSIDE = 0,
	OUTSIDE = 1,
	SHADOW = 2,
    };

struct LinkInfo
{
  std::string name;
  double padding;
  double scale;
};
    
    static inline tf::Transform urdfPose2TFTransform(const urdf::Pose &pose)
    {
      return tf::Transform(tf::Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w),
			   tf::Vector3(pose.position.x, pose.position.y, pose.position.z));
    }

    static shapes::Shape* constructShape(const urdf::Geometry *geom)
    {
	ROS_ASSERT(geom);
	
	shapes::Shape *result = NULL;
	switch (geom->type)
	{
	case urdf::Geometry::SPHERE:
	    result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
	    break;	
	case urdf::Geometry::BOX:
	    {
		urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
		result = new shapes::Box(dim.x, dim.y, dim.z);
	    }
	    break;
	case urdf::Geometry::CYLINDER:
	    result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius,
					  dynamic_cast<const urdf::Cylinder*>(geom)->length);
	    break;
	case urdf::Geometry::MESH:
	    {
		const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
		if (!mesh->filename.empty())
		{
		    resource_retriever::Retriever retriever;
		    resource_retriever::MemoryResource res;
		    bool ok = true;
		    
		    try
		    {
			res = retriever.get(mesh->filename);
		    }
		    catch (resource_retriever::Exception& e)
		    {
			ROS_ERROR("%s", e.what());
			ok = false;
		    }
		    
		    if (ok)
		    {
			if (res.size == 0)
			    ROS_WARN("Retrieved empty mesh for resource '%s'", mesh->filename.c_str());
			else
			{
			    boost::filesystem::path model_path(mesh->filename);
			    std::string ext = model_path.extension().string();
			    if (ext == ".dae" || ext == ".DAE") {
			      result = shapes::createMeshFromBinaryDAE(mesh->filename.c_str());
			    }
			    else {
			      result = shapes::createMeshFromBinaryStlData(reinterpret_cast<char*>(res.data.get()), res.size);
			    }
			    if (result == NULL)
				ROS_ERROR("Failed to load mesh '%s'", mesh->filename.c_str());
			}
		    }
		}
		else
		    ROS_WARN("Empty mesh filename");
	    }
	    
	    break;
	default:
	    ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
	    break;
	}
	
	return result;
    }

    /** \brief Computing a mask for a pointcloud that states which points are inside the robot
     *
     */
    template <typename PointT>
    class SelfMask
    {	
    protected:
	
	struct SeeLink
	{
	    SeeLink(void)
	    {
		body = unscaledBody = NULL;
	    }
	    
	    std::string   name;
	    bodies::Body *body;
	    bodies::Body *unscaledBody;
	    tf::Transform   constTransf;
	    double        volume;
	};
	
	struct SortBodies
	{
	    bool operator()(const SeeLink &b1, const SeeLink &b2)
	    {
		return b1.volume > b2.volume;
	    }
	};
	
    public:
	typedef pcl::PointCloud<PointT> PointCloud;

	/** \brief Construct the filter */
	SelfMask(tf::TransformListener &tf, const std::vector<LinkInfo> &links) : tf_(tf)
	{
	    configure(links);
	}
	
	/** \brief Destructor to clean up
	 */
	~SelfMask(void)
	{
	    freeMemory();
	}
	
	/** \brief Compute the containment mask (INSIDE or OUTSIDE) for a given pointcloud. If a mask element is INSIDE, the point
	    is inside the robot. The point is outside if the mask element is OUTSIDE.
	 */
	void maskContainment(const PointCloud& data_in, std::vector<int> &mask)
        {
          mask.resize(data_in.points.size());
          if (bodies_.empty())
            std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
          else
          {
            std_msgs::Header header = pcl_conversions::fromPCL(data_in.header);
            assumeFrame(header);
            maskAuxContainment(data_in, mask);
          }

        }

	/** \brief Compute the intersection mask for a given
	    pointcloud. If a mask element can have one of the values
	    INSIDE, OUTSIDE or SHADOW. If the value is SHADOW, the
	    point is on a ray behind the robot and should not have
	    been seen. If the mask element is INSIDE, the point is
	    inside the robot. The sensor frame is specified to obtain
	    the origin of the sensor. A callback can be registered for
	    the first intersection point on each body.
	 */
	void maskIntersection(const PointCloud& data_in, const std::string &sensor_frame, const double min_sensor_dist,
			      std::vector<int> &mask, const boost::function<void(const tf::Vector3&)> &intersectionCallback = NULL)
        {
          mask.resize(data_in.points.size());
          if (bodies_.empty()) {
            std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
          }
          else
          {
            std_msgs::Header header = pcl_conversions::fromPCL(data_in.header);
            assumeFrame(header, sensor_frame, min_sensor_dist);
            if (sensor_frame.empty())
              maskAuxContainment(data_in, mask);
            else
              maskAuxIntersection(data_in, mask, intersectionCallback);
          }
          
        }
        
        
	/** \brief Compute the intersection mask for a given pointcloud. If a mask
	    element can have one of the values INSIDE, OUTSIDE or SHADOW. If the value is SHADOW,
	    the point is on a ray behind the robot and should not have
	    been seen. If the mask element is INSIDE, the point is inside
	    the robot. The origin of the sensor is specified as well.
	 */
	void maskIntersection(const PointCloud& data_in, const tf::Vector3 &sensor_pos, const double min_sensor_dist,
			      std::vector<int> &mask, const boost::function<void(const tf::Vector3&)> &intersectionCallback = NULL)
        {
          mask.resize(data_in.points.size());
          if (bodies_.empty())
            std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
          else
          {
            std_msgs::Header header = pcl_conversions::fromPCL(data_in.header);
            assumeFrame(header, sensor_pos, min_sensor_dist);
            maskAuxIntersection(data_in, mask, intersectionCallback);
          }

        }
	
	/** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
	 *   The frame in which the sensor is located is optional */
	void assumeFrame(const std_msgs::Header& header)
        {
              const unsigned int bs = bodies_.size();
    
    // place the links in the assumed frame 
    for (unsigned int i = 0 ; i < bs ; ++i)
    {
      std::string err;
      if(!tf_.waitForTransform(header.frame_id, bodies_[i].name, header.stamp, ros::Duration(.1), ros::Duration(.01), &err)) {
        ROS_ERROR("WaitForTransform timed out from %s to %s after 100ms.  Error string: %s", bodies_[i].name.c_str(), header.frame_id.c_str(), err.c_str());
        
      } 
      
      // find the transform between the link's frame and the pointcloud frame
      tf::StampedTransform transf;
      try
      {
        tf_.lookupTransform(header.frame_id, bodies_[i].name, header.stamp, transf);
      }
      catch(tf::TransformException& ex)
      {
        transf.setIdentity();
        ROS_ERROR("Unable to lookup transform from %s to %s. Exception: %s", bodies_[i].name.c_str(), header.frame_id.c_str(), ex.what());	
      }
      
      // set it for each body; we also include the offset specified in URDF
      bodies_[i].body->setPose(transf * bodies_[i].constTransf);
      bodies_[i].unscaledBody->setPose(transf * bodies_[i].constTransf);
    }
    
    computeBoundingSpheres();

        }
	
	
        /** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
	 *  Also specify which possition to assume for the sensor (frame is not needed) */
	void assumeFrame(const std_msgs::Header& header, const tf::Vector3 &sensor_pos, const double min_sensor_dist)
        {
          assumeFrame(header);
          sensor_pos_ = sensor_pos;
          min_sensor_dist_ = min_sensor_dist;
        }

	/** \brief Assume subsequent calls to getMaskX() will be in the frame passed to this function.
	 *   The frame in which the sensor is located is optional */
	void assumeFrame(const std_msgs::Header& header, const std::string &sensor_frame, const double min_sensor_dist)
        {
          assumeFrame(header);

          std::string err;
          if(!tf_.waitForTransform(header.frame_id, sensor_frame, header.stamp, ros::Duration(.1), ros::Duration(.01), &err)) {
            ROS_ERROR("WaitForTransform timed out from %s to %s after 100ms.  Error string: %s", sensor_frame.c_str(), header.frame_id.c_str(), err.c_str());
            sensor_pos_.setValue(0, 0, 0);
          } 

          //transform should be there
          // compute the origin of the sensor in the frame of the cloud
          try
          {
            tf::StampedTransform transf;
            tf_.lookupTransform(header.frame_id, sensor_frame, header.stamp, transf);
            sensor_pos_ = transf.getOrigin();
          }
          catch(tf::TransformException& ex)
          {
            sensor_pos_.setValue(0, 0, 0);
            ROS_ERROR("Unable to lookup transform from %s to %s.  Exception: %s", sensor_frame.c_str(), header.frame_id.c_str(), ex.what());
          }
  
          min_sensor_dist_ = min_sensor_dist;

        }
	
        /** \brief Get the containment mask (INSIDE or OUTSIDE) value for an individual point. No
	    setup is performed, assumeFrame() should be called before use */
	int  getMaskContainment(const tf::Vector3 &pt) const
        {
          const unsigned int bs = bodies_.size();
          int out = OUTSIDE;
          for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
            if (bodies_[j].body->containsPoint(pt))
              out = INSIDE;
          return out;
        }
	
	/** \brief Get the containment mask (INSIDE or OUTSIDE) value for an individual point. No
	    setup is performed, assumeFrame() should be called before use */
	int  getMaskContainment(double x, double y, double z) const
        {
          return getMaskContainment(tf::Vector3(x, y, z));
        }
	
	/** \brief Get the intersection mask (INSIDE, OUTSIDE or
	    SHADOW) value for an individual point. No setup is
	    performed, assumeFrame() should be called before use */
	int  getMaskIntersection(double x, double y, double z, const boost::function<void(const tf::Vector3&)> &intersectionCallback = NULL) const
        {
          return getMaskIntersection(tf::Vector3(x, y, z), intersectionCallback);
        }
	
	/** \brief Get the intersection mask (INSIDE, OUTSIDE or
	    SHADOW) value for an individual point. No setup is
	    performed, assumeFrame() should be called before use */
	int  getMaskIntersection(const tf::Vector3 &pt, const boost::function<void(const tf::Vector3&)> &intersectionCallback = NULL) const
        {
          const unsigned int bs = bodies_.size();

          // we first check is the point is in the unscaled body. 
          // if it is, the point is definitely inside
          int out = OUTSIDE;
          for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
            if (bodies_[j].unscaledBody->containsPoint(pt))
              out = INSIDE;
    
          if (out == OUTSIDE)
          {

            // we check it the point is a shadow point 
            tf::Vector3 dir(sensor_pos_ - pt);
            tfScalar  lng = dir.length();
            if (lng < min_sensor_dist_)
              out = INSIDE;
            else
            {
              dir /= lng;
	    
              std::vector<tf::Vector3> intersections;
              for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
		if (bodies_[j].body->intersectsRay(pt, dir, &intersections, 1))
		{
                  if (dir.dot(sensor_pos_ - intersections[0]) >= 0.0)
                  {
                    if (intersectionCallback)
                      intersectionCallback(intersections[0]);
                    out = SHADOW;
                  }
		}
	    
              // if it is not a shadow point, we check if it is inside the scaled body
              for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
		if (bodies_[j].body->containsPoint(pt))
                  out = INSIDE;
            }
          }
          return out;

        }
	
	/** \brief Get the set of link names that have been instantiated for self filtering */
	void getLinkNames(std::vector<std::string> &frames) const
        {
          for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
            frames.push_back(bodies_[i].name);
        }
	
    protected:

	/** \brief Free memory. */
	void freeMemory(void)
        {
          for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
            {
              if (bodies_[i].body)
                delete bodies_[i].body;
              if (bodies_[i].unscaledBody)
                delete bodies_[i].unscaledBody;
            }
          
          bodies_.clear();
        }


	/** \brief Configure the filter. */
	bool configure(const std::vector<LinkInfo> &links)
        {
          // in case configure was called before, we free the memory
          freeMemory();
          sensor_pos_.setValue(0, 0, 0);
    
          std::string content;
          boost::shared_ptr<urdf::Model> urdfModel;

          if (nh_.getParam("robot_description", content))
          {
            urdfModel = boost::shared_ptr<urdf::Model>(new urdf::Model());
            if (!urdfModel->initString(content))
            {
              ROS_ERROR("Unable to parse URDF description!");
              return false;
            }
          }
          else
          {
            ROS_ERROR("Robot model not found! Did you remap 'robot_description'?");
            return false;
          }
          
          std::stringstream missing;
    
          // from the geometric model, find the shape of each link of interest
          // and create a body from it, one that knows about poses and can 
          // check for point inclusion
          for (unsigned int i = 0 ; i < links.size() ; ++i)
          {
            const urdf::Link *link = urdfModel->getLink(links[i].name).get();
            if (!link)
            {
              missing << " " << links[i].name;
              continue;
            }
            
            if (!(link->collision && link->collision->geometry))
            {
              ROS_WARN("No collision geometry specified for link '%s'", links[i].name.c_str());
              continue;
            }
	
            shapes::Shape *shape = constructShape(link->collision->geometry.get());
	
            if (!shape)
            {
              ROS_ERROR("Unable to construct collision shape for link '%s'", links[i].name.c_str());
              continue;
            }
	
            SeeLink sl;
            sl.body = bodies::createBodyFromShape(shape);

            if (sl.body)
            {
              sl.name = links[i].name;
              
              // collision models may have an offset, in addition to what TF gives
              // so we keep it around
              sl.constTransf = urdfPose2TFTransform(link->collision->origin);
              
              sl.body->setScale(links[i].scale);
              sl.body->setPadding(links[i].padding);
              ROS_INFO_STREAM("Self see link name " <<  links[i].name << " padding " << links[i].padding);
              sl.volume = sl.body->computeVolume();
              sl.unscaledBody = bodies::createBodyFromShape(shape);
              bodies_.push_back(sl);
            }
            else
              ROS_WARN("Unable to create point inclusion body for link '%s'", links[i].name.c_str());
	
            delete shape;
          }
    
          if (missing.str().size() > 0)
            ROS_WARN("Some links were included for self mask but they do not exist in the model:%s", missing.str().c_str());
    
          if (bodies_.empty())
            ROS_WARN("No robot links will be checked for self mask");
    
          // put larger volume bodies first -- higher chances of containing a point
          std::sort(bodies_.begin(), bodies_.end(), SortBodies());
    
          bspheres_.resize(bodies_.size());
          bspheresRadius2_.resize(bodies_.size());

          for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
            ROS_DEBUG("Self mask includes link %s with volume %f", bodies_[i].name.c_str(), bodies_[i].volume);
    
          //ROS_INFO("Self filter using %f padding and %f scaling", padd, scale);

          return true; 

        }
	
	/** \brief Compute bounding spheres for the checked robot links. */
	void computeBoundingSpheres(void)
        {
          const unsigned int bs = bodies_.size();
          for (unsigned int i = 0 ; i < bs ; ++i)
          {
            bodies_[i].body->computeBoundingSphere(bspheres_[i]);
            bspheresRadius2_[i] = bspheres_[i].radius * bspheres_[i].radius;
          }
        }

	
	/** \brief Perform the actual mask computation. */
	void maskAuxContainment(const PointCloud& data_in, std::vector<int> &mask)
        {
          const unsigned int bs = bodies_.size();
          const unsigned int np = data_in.points.size();
    
          // compute a sphere that bounds the entire robot
          bodies::BoundingSphere bound;
          bodies::mergeBoundingSpheres(bspheres_, bound);	  
          tfScalar radiusSquared = bound.radius * bound.radius;
    
          // we now decide which points we keep
          //#pragma omp parallel for schedule(dynamic) 
          for (int i = 0 ; i < (int)np ; ++i)
          {
            tf::Vector3 pt = tf::Vector3(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
            int out = OUTSIDE;
            if (bound.center.distance2(pt) < radiusSquared)
              for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
		if (bodies_[j].body->containsPoint(pt))
                  out = INSIDE;
	
            mask[i] = out;
          }
        }

	/** \brief Perform the actual mask computation. */
	void maskAuxIntersection(const PointCloud& data_in, std::vector<int> &mask, const boost::function<void(const tf::Vector3&)> &callback)
        {
          const unsigned int bs = bodies_.size();
          const unsigned int np = data_in.points.size();
    
          // compute a sphere that bounds the entire robot
          bodies::BoundingSphere bound;
          bodies::mergeBoundingSpheres(bspheres_, bound);	  
          tfScalar radiusSquared = bound.radius * bound.radius;

          //std::cout << "Testing " << np << " points\n";

          // we now decide which points we keep
          //#pragma omp parallel for schedule(dynamic) 
          for (int i = 0 ; i < (int)np ; ++i)
          {
            bool print = false;
            //if(i%100 == 0) print = true;
            tf::Vector3 pt = tf::Vector3(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
            int out = OUTSIDE;

            // we first check is the point is in the unscaled body. 
            // if it is, the point is definitely inside
            if (bound.center.distance2(pt) < radiusSquared)
              for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
                if (bodies_[j].unscaledBody->containsPoint(pt)) {
                  if(print)
                    std::cout << "Point " << i << " in unscaled body part " << bodies_[j].name << std::endl;
                  out = INSIDE;
                }

            // if the point is not inside the unscaled body,
            if (out == OUTSIDE)
            {
              // we check it the point is a shadow point 
              tf::Vector3 dir(sensor_pos_ - pt);
              tfScalar  lng = dir.length();
              if (lng < min_sensor_dist_) {
		out = INSIDE;
                //std::cout << "Point " << i << " less than min sensor distance away\n";
              }
              else
              {		
		dir /= lng;

		std::vector<tf::Vector3> intersections;
		for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j) {
                  if (bodies_[j].body->intersectsRay(pt, dir, &intersections, 1))
                  {
                    if (dir.dot(sensor_pos_ - intersections[0]) >= 0.0)
                    {
                      if (callback)
                        callback(intersections[0]);
                      out = SHADOW;
                      if(print) std::cout << "Point " << i << " shadowed by body part " << bodies_[j].name << std::endl;
                    }
                  }
		}
		// if it is not a shadow point, we check if it is inside the scaled body
		if (out == OUTSIDE && bound.center.distance2(pt) < radiusSquared)
                  for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
                    if (bodies_[j].body->containsPoint(pt)) {
                      if(print) std::cout << "Point " << i << " in scaled body part " << bodies_[j].name << std::endl;
                      out = INSIDE;
                    }
              }
            }
            mask[i] = out;
          }
        }
	
	tf::TransformListener              &tf_;
	ros::NodeHandle                     nh_;
	
	tf::Vector3                           sensor_pos_;
	double                              min_sensor_dist_;
	
	std::vector<SeeLink>                bodies_;
	std::vector<double>                 bspheresRadius2_;
	std::vector<bodies::BoundingSphere> bspheres_;
	
    };
    
}

#endif
