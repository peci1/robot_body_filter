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

#include "robot_self_filter/bodies.h"
#include <LinearMath/btConvexHull.h>
// #include <BulletCollision/CollisionShapes/btTriangleShape.h>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace robot_self_filter
{
bodies::Body* bodies::createBodyFromShape(const shapes::Shape *shape)
{
    Body *body = NULL;
    
    if (shape)
	switch (shape->type)
	{
	case shapes::BOX:
	    body = new bodies::Box(shape);
	    break;
	case shapes::SPHERE:
	    body = new bodies::Sphere(shape);
	    break;
	case shapes::CYLINDER:
	    body = new bodies::Cylinder(shape);
	    break;
	case shapes::MESH:
	    body = new bodies::ConvexMesh(shape);
	    break;
	default:
	    std::cerr << "Creating body from shape: Unknown shape type" << shape->type << std::endl;
	    break;
	}
    
    return body;
}

void bodies::mergeBoundingSpheres(const std::vector<BoundingSphere> &spheres, BoundingSphere &mergedSphere)
{
    if (spheres.empty())
    {
	mergedSphere.center.setValue(tfScalar(0), tfScalar(0), tfScalar(0));
	mergedSphere.radius = 0.0;
    }
    else
    {
	mergedSphere = spheres[0];
	for (unsigned int i = 1 ; i < spheres.size() ; ++i)
	{
	    if (spheres[i].radius <= 0.0)
		continue;
	    double d = spheres[i].center.distance(mergedSphere.center);
	    if (d + mergedSphere.radius <= spheres[i].radius)
	    {
		mergedSphere.center = spheres[i].center;
		mergedSphere.radius = spheres[i].radius;
	    }
	    else
		if (d + spheres[i].radius > mergedSphere.radius)
		{
		    tf::Vector3 delta = mergedSphere.center - spheres[i].center;
		    mergedSphere.radius = (delta.length() + spheres[i].radius + mergedSphere.radius)/2.0;
		    mergedSphere.center = delta.normalized() * (mergedSphere.radius - spheres[i].radius) + spheres[i].center;
		}
	}
    }
}

namespace bodies
{
    static const double ZERO = 1e-9;
    
    /** \brief Compute the square of the distance between a ray and a point 
	Note: this requires 'dir' to be normalized */
    static inline double distanceSQR(const tf::Vector3& p, const tf::Vector3& origin, const tf::Vector3& dir)
    {
	tf::Vector3 a = p - origin;
	double d = dir.dot(a);
	return a.length2() - d * d;
    }
    
    namespace detail
    {
	
	// temp structure for intersection points (used for ordering them)
	struct intersc
	{
	    intersc(const tf::Vector3 &_pt, const double _tm) : pt(_pt), time(_tm) {}
	    
	    tf::Vector3 pt;
	    double    time;
	};
	
	// define order on intersection points
	struct interscOrder
	{
	    bool operator()(const intersc &a, const intersc &b) const
	    {
		return a.time < b.time;
	    }
	};
    }
}

bool bodies::Sphere::containsPoint(const tf::Vector3 &p, bool verbose) const 
{
    return (m_center - p).length2() < m_radius2;
}

void bodies::Sphere::useDimensions(const shapes::Shape *shape) // radius
{
    m_radius = static_cast<const shapes::Sphere*>(shape)->radius;
}

void bodies::Sphere::updateInternalData(void)
{
    m_radiusU = m_radius * m_scale + m_padding;
    m_radius2 = m_radiusU * m_radiusU;
    m_center = m_pose.getOrigin();
}

double bodies::Sphere::computeVolume(void) const
{
    return 4.0 * M_PI * m_radiusU * m_radiusU * m_radiusU / 3.0;
}

void bodies::Sphere::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusU;
}

bool bodies::Sphere::intersectsRay(const tf::Vector3& origin, const tf::Vector3& dir, std::vector<tf::Vector3> *intersections, unsigned int count) const
{
    if (distanceSQR(m_center, origin, dir) > m_radius2) return false;
    
    bool result = false;
    
    tf::Vector3 cp = origin - m_center;
    double dpcpv = cp.dot(dir);
    
    tf::Vector3 w = cp - dpcpv * dir;
    tf::Vector3 Q = m_center + w;
    double x = m_radius2 - w.length2();
    
    if (fabs(x) < ZERO)
    { 
	w = Q - origin;
	double dpQv = w.dot(dir);
	if (dpQv > ZERO)
	{
	    if (intersections)
		intersections->push_back(Q);
	    result = true;
	}
    } else 
	if (x > 0.0)
	{    
	    x = sqrt(x);
	    w = dir * x;
	    tf::Vector3 A = Q - w;
	    tf::Vector3 B = Q + w;
	    w = A - origin;
	    double dpAv = w.dot(dir);
	    w = B - origin;
	    double dpBv = w.dot(dir);
	    
	    if (dpAv > ZERO)
	    {	
		result = true;
		if (intersections)
		{
		    intersections->push_back(A);
		    if (count == 1)
			return result;
		}
	    }
	    
	    if (dpBv > ZERO)
	    {
		result = true;
		if (intersections)
		    intersections->push_back(B);
	    }
	}
    return result;
}

bool bodies::Cylinder::containsPoint(const tf::Vector3 &p, bool verbose) const 
{
    tf::Vector3 v = p - m_center;		
    double pH = v.dot(m_normalH);
    
    if (fabs(pH) > m_length2)
	return false;
    
    double pB1 = v.dot(m_normalB1);
    double remaining = m_radius2 - pB1 * pB1;
    
    if (remaining < 0.0)
	return false;
    else
    {
	double pB2 = v.dot(m_normalB2);
	return pB2 * pB2 < remaining;
    }		
}

void bodies::Cylinder::useDimensions(const shapes::Shape *shape) // (length, radius)
{
    m_length = static_cast<const shapes::Cylinder*>(shape)->length;
    m_radius = static_cast<const shapes::Cylinder*>(shape)->radius;
}

void bodies::Cylinder::updateInternalData(void)
{
    m_radiusU = m_radius * m_scale + m_padding;
    m_radius2 = m_radiusU * m_radiusU;
    m_length2 = m_scale * m_length / 2.0 + m_padding;
    m_center = m_pose.getOrigin();
    m_radiusBSqr = m_length2 * m_length2 + m_radius2;
    m_radiusB = sqrt(m_radiusBSqr);
    
    const tf::Matrix3x3& basis = m_pose.getBasis();
    m_normalB1 = basis.getColumn(0);
    m_normalB2 = basis.getColumn(1);
    m_normalH  = basis.getColumn(2);
    
    double tmp = -m_normalH.dot(m_center);
    m_d1 = tmp + m_length2;
    m_d2 = tmp - m_length2;
}

double bodies::Cylinder::computeVolume(void) const
{
    return 2.0 * M_PI * m_radius2 * m_length2;
}

void bodies::Cylinder::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

bool bodies::Cylinder::intersectsRay(const tf::Vector3& origin, const tf::Vector3& dir, std::vector<tf::Vector3> *intersections, unsigned int count) const
{
    if (distanceSQR(m_center, origin, dir) > m_radiusBSqr) return false;

    std::vector<detail::intersc> ipts;
    
    // intersect bases
    double tmp = m_normalH.dot(dir);
    if (fabs(tmp) > ZERO)
    {
	double tmp2 = -m_normalH.dot(origin);
	double t1 = (tmp2 - m_d1) / tmp;
	
	if (t1 > 0.0)
	{
	    tf::Vector3 p1(origin + dir * t1);
	    tf::Vector3 v1(p1 - m_center);
	    v1 = v1 - m_normalH.dot(v1) * m_normalH;
	    if (v1.length2() < m_radius2 + ZERO)
	    {
		if (intersections == NULL)
		    return true;
		
		detail::intersc ip(p1, t1);
		ipts.push_back(ip);
	    }
	}
	
	double t2 = (tmp2 - m_d2) / tmp;
	if (t2 > 0.0)
	{
	    tf::Vector3 p2(origin + dir * t2);
	    tf::Vector3 v2(p2 - m_center);
	    v2 = v2 - m_normalH.dot(v2) * m_normalH;
	    if (v2.length2() < m_radius2 + ZERO)
	    {
		if (intersections == NULL)
		    return true;
		
		detail::intersc ip(p2, t2);
		ipts.push_back(ip);
	    }
	}
    }
    
    if (ipts.size() < 2)
    {
	// intersect with infinite cylinder
	tf::Vector3 VD(m_normalH.cross(dir));
	tf::Vector3 ROD(m_normalH.cross(origin - m_center));
	double a = VD.length2();
	double b = 2.0 * ROD.dot(VD);
	double c = ROD.length2() - m_radius2;
	double d = b * b - 4.0 * a * c;
	if (d > 0.0 && fabs(a) > ZERO)
	{
	    d = sqrt(d);
	    double e = -a * 2.0;
	    double t1 = (b + d) / e;
	    double t2 = (b - d) / e;
	    
	    if (t1 > 0.0)
	    {
		tf::Vector3 p1(origin + dir * t1);
		tf::Vector3 v1(m_center - p1);
		
		if (fabs(m_normalH.dot(v1)) < m_length2 + ZERO)
		{
		    if (intersections == NULL)
			return true;
		    
		    detail::intersc ip(p1, t1);
		    ipts.push_back(ip);
		}
	    }
	    
	    if (t2 > 0.0)
	    {
		tf::Vector3 p2(origin + dir * t2);    
		tf::Vector3 v2(m_center - p2);
		
		if (fabs(m_normalH.dot(v2)) < m_length2 + ZERO)
		{
		    if (intersections == NULL)
			return true;
		    detail::intersc ip(p2, t2);
		    ipts.push_back(ip);
		}
	    }
	}
    }
    
    if (ipts.empty())
	return false;

    std::sort(ipts.begin(), ipts.end(), detail::interscOrder());
    const unsigned int n = count > 0 ? std::min<unsigned int>(count, ipts.size()) : ipts.size();
    for (unsigned int i = 0 ; i < n ; ++i)
	intersections->push_back(ipts[i].pt);
    
    return true;
}

bool bodies::Box::containsPoint(const tf::Vector3 &p, bool verbose) const 
{
  /*  if(verbose)
      fprintf(stderr,"Actual: %f,%f,%f \nDesired: %f,%f,%f \nTolerance:%f,%f,%f\n",p.x(),p.y(),p.z(),m_center.x(),m_center.y(),m_center.z(),m_length2,m_width2,m_height2);*/
    tf::Vector3 v = p - m_center;
    double pL = v.dot(m_normalL);
    
    if (fabs(pL) > m_length2)
	return false;
    
    double pW = v.dot(m_normalW);
    
    if (fabs(pW) > m_width2)
	return false;
    
    double pH = v.dot(m_normalH);
    
    if (fabs(pH) > m_height2)
	return false;
    
    return true;
}

void bodies::Box::useDimensions(const shapes::Shape *shape) // (x, y, z) = (length, width, height)
{
    const double *size = static_cast<const shapes::Box*>(shape)->size;
    m_length = size[0];
    m_width  = size[1];
    m_height = size[2];
}

void bodies::Box::updateInternalData(void) 
{
    double s2 = m_scale / 2.0;
    m_length2 = m_length * s2 + m_padding;
    m_width2  = m_width * s2 + m_padding;
    m_height2 = m_height * s2 + m_padding;
    
    m_center  = m_pose.getOrigin();
    
    m_radius2 = m_length2 * m_length2 + m_width2 * m_width2 + m_height2 * m_height2;
    m_radiusB = sqrt(m_radius2);

    const tf::Matrix3x3& basis = m_pose.getBasis();
    m_normalL = basis.getColumn(0);
    m_normalW = basis.getColumn(1);
    m_normalH = basis.getColumn(2);

    const tf::Vector3 tmp(m_normalL * m_length2 + m_normalW * m_width2 + m_normalH * m_height2);
    m_corner1 = m_center - tmp;
    m_corner2 = m_center + tmp;
}

double bodies::Box::computeVolume(void) const
{
    return 8.0 * m_length2 * m_width2 * m_height2;
}

void bodies::Box::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

bool bodies::Box::intersectsRay(const tf::Vector3& origin, const tf::Vector3& dir, std::vector<tf::Vector3> *intersections, unsigned int count) const
{  
    if (distanceSQR(m_center, origin, dir) > m_radius2) return false;

    double t_near = -INFINITY;
    double t_far  = INFINITY;
    
    for (int i = 0; i < 3; i++)
    {
	const tf::Vector3 &vN = i == 0 ? m_normalL : (i == 1 ? m_normalW : m_normalH);
	double dp = vN.dot(dir);
	
	if (fabs(dp) > ZERO)
	{
	    double t1 = vN.dot(m_corner1 - origin) / dp;
	    double t2 = vN.dot(m_corner2 - origin) / dp;
	    
	    if (t1 > t2)
		std::swap(t1, t2);
	    
	    if (t1 > t_near)
		t_near = t1;
	    
	    if (t2 < t_far)
		t_far = t2;
	    
	    if (t_near > t_far)
		return false;
	    
	    if (t_far < 0.0)
		return false;
	}
	else
	{
	    if (i == 0)
	    {
		if ((std::min(m_corner1.y(), m_corner2.y()) > origin.y() ||
		     std::max(m_corner1.y(), m_corner2.y()) < origin.y()) && 
		    (std::min(m_corner1.z(), m_corner2.z()) > origin.z() ||
		     std::max(m_corner1.z(), m_corner2.z()) < origin.z()))
		    return false;
	    }
	    else
	    {
		if (i == 1)
		{
		    if ((std::min(m_corner1.x(), m_corner2.x()) > origin.x() ||
			 std::max(m_corner1.x(), m_corner2.x()) < origin.x()) && 
			(std::min(m_corner1.z(), m_corner2.z()) > origin.z() ||
			 std::max(m_corner1.z(), m_corner2.z()) < origin.z()))
			return false;
		}
		else
		    if ((std::min(m_corner1.x(), m_corner2.x()) > origin.x() ||
			 std::max(m_corner1.x(), m_corner2.x()) < origin.x()) && 
			(std::min(m_corner1.y(), m_corner2.y()) > origin.y() ||
			 std::max(m_corner1.y(), m_corner2.y()) < origin.y()))
			return false;
	    }
	}
    }
    
    if (intersections)
    {
	if (t_far - t_near > ZERO)
	{
	    intersections->push_back(t_near * dir + origin);
	    if (count > 1)
		intersections->push_back(t_far  * dir + origin);
	}
	else
	    intersections->push_back(t_far * dir + origin);
    }
    
    return true;
}
/*
bool bodies::Mesh::containsPoint(const tf::Vector3 &p) const
{
    // compute all intersections
    std::vector<tf::Vector3> pts;
    intersectsRay(p, tf::Vector3(0, 0, 1), &pts);
    
    // if we have an odd number of intersections, we are inside
    return pts.size() % 2 == 1;
}

namespace bodies
{
    namespace detail
    {
	// callback for bullet 
	class myTriangleCallback : public btTriangleCallback
	{
	public:
	    
	    myTriangleCallback(bool keep_triangles) : keep_triangles_(keep_triangles)
	    {
		found_intersections = false;
	    }
	    
	    virtual void processTriangle(tf::Vector3 *triangle, int partId, int triangleIndex)
	    {
		found_intersections = true;
		if (keep_triangles_)
		    triangles.push_back(btTriangleShape(triangle[0], triangle[1], triangle[2]));
		/// \todo figure out if scaling & margins are used for this triangle
	    }
	    
	    bool                         found_intersections;
	    std::vector<btTriangleShape> triangles;
	    
	private:
	    
	    bool                         keep_triangles_;
	};

    }
}

bool bodies::Mesh::intersectsRay(const tf::Vector3& origin, const tf::Vector3& dir, std::vector<tf::Vector3> *intersections, unsigned int count) const
{
    if (m_btMeshShape)
    {
	
	if (distanceSQR(m_center, origin, dir) > m_radiusBSqr) return false;
	
	// transform the ray into the coordinate frame of the mesh
	tf::Vector3 orig(m_iPose * origin);
	tf::Vector3 dr(m_iPose.getBasis() * dir);
	
	// find intersection with AABB
	tfScalar maxT = 0.0;
	
	if (fabs(dr.x()) > ZERO)
	{
	    tfScalar t = (m_aabbMax.x() - orig.x()) / dr.x();
	    if (t < 0.0)
		t = (m_aabbMin.x() - orig.x()) / dr.x();
	    if (t > maxT)
		maxT = t;
	}
	if (fabs(dr.y()) > ZERO)
	{
	    tfScalar t = (m_aabbMax.y() - orig.y()) / dr.y();
	    if (t < 0.0)
		t = (m_aabbMin.y() - orig.y()) / dr.y();
	    if (t > maxT)
		maxT = t;
	}
	if (fabs(dr.z()) > ZERO)
	{
	    tfScalar t = (m_aabbMax.z() - orig.z()) / dr.z();
	    if (t < 0.0)
		t = (m_aabbMin.z() - orig.z()) / dr.z();
	    if (t > maxT)
		maxT = t;
	}
	
	// the farthest point where we could have an intersection id orig + dr * maxT
	
	detail::myTriangleCallback cb(intersections != NULL);
	m_btMeshShape->performRaycast(&cb, orig, orig + dr * maxT);
	if (cb.found_intersections)
	{
	    if (intersections)
	    {
		std::vector<detail::intersc> intpt;
		for (unsigned int i = 0 ; i < cb.triangles.size() ; ++i)
		{
		    tf::Vector3 normal;
		    cb.triangles[i].calcNormal(normal);
		    tfScalar dv = normal.dot(dr);
		    if (fabs(dv) > 1e-3)
		    {
			double t = (normal.dot(cb.triangles[i].getVertexPtr(0)) - normal.dot(orig)) / dv;			
			// here we use the input origin & direction, since the transform is linear
			detail::intersc ip(origin + dir * t, t);
			intpt.push_back(ip);
		    }
		}
		std::sort(intpt.begin(), intpt.end(), detail::interscOrder());
		const unsigned int n = count > 0 ? std::min<unsigned int>(count, intpt.size()) : intpt.size();
		for (unsigned int i = 0 ; i < n ; ++i)
		    intersections->push_back(intpt[i].pt);
	    }
	    return true;	    
	}
	else
	    return false;
    }
    else
	return false;
}   

void bodies::Mesh::useDimensions(const shapes::Shape *shape)
{  
    const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);
    if (m_btMeshShape)
	delete m_btMeshShape;
    if (m_btMesh)
	delete m_btMesh;
    
    m_btMesh = new btTriangleMesh();
    const unsigned int nt = mesh->triangleCount / 3;
    for (unsigned int i = 0 ; i < nt ; ++i)
    {
	const unsigned int i3 = i *3;
	const unsigned int v1 = 3 * mesh->triangles[i3];
	const unsigned int v2 = 3 * mesh->triangles[i3 + 1];
	const unsigned int v3 = 3 * mesh->triangles[i3 + 2];
	m_btMesh->addTriangle(tf::Vector3(mesh->vertices[v1], mesh->vertices[v1 + 1], mesh->vertices[v1 + 2]),
			      tf::Vector3(mesh->vertices[v2], mesh->vertices[v2 + 1], mesh->vertices[v2 + 2]),
			      tf::Vector3(mesh->vertices[v3], mesh->vertices[v3 + 1], mesh->vertices[v3 + 2]), true);
    }
    
    m_btMeshShape = new btBvhTriangleMeshShape(m_btMesh, true);
}

void bodies::Mesh::updateInternalData(void) 
{
    if (m_btMeshShape)
    {
	m_btMeshShape->setLocalScaling(tf::Vector3(m_scale, m_scale, m_scale));
	m_btMeshShape->setMargin(m_padding);
    }
    m_iPose = m_pose.inverse();
    
    tf::Transform id;
    id.setIdentity();
    m_btMeshShape->getAabb(id, m_aabbMin, m_aabbMax);

    tf::Vector3 d = (m_aabbMax - m_aabbMin) / 2.0;

    m_center = m_pose.getOrigin() + d;
    m_radiusBSqr = d.length2();
    m_radiusB = sqrt(m_radiusBSqr);
    
    /// \todo check if the AABB computation includes padding & scaling; if not, include it
}

double bodies::Mesh::computeVolume(void) const
{
    if (m_btMeshShape)
    {
	// approximation; volume of AABB at identity transform
	tf::Vector3 d = m_aabbMax - m_aabbMin;
	return d.x() * d.y() * d.z();
    }
    else
	return 0.0;
}

void bodies::Mesh::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

*/

bool bodies::ConvexMesh::containsPoint(const tf::Vector3 &p, bool verbose) const
{
    if (m_boundingBox.containsPoint(p))
    {
	tf::Vector3 ip(m_iPose * p);
	ip = m_meshCenter + (ip - m_meshCenter) * m_scale;
	return isPointInsidePlanes(ip);
    }
    else
	return false;
}

void bodies::ConvexMesh::useDimensions(const shapes::Shape *shape)
{  
    const shapes::Mesh *mesh = static_cast<const shapes::Mesh*>(shape);

    double maxX = -INFINITY, maxY = -INFINITY, maxZ = -INFINITY;
    double minX =  INFINITY, minY =  INFINITY, minZ  = INFINITY;
    
    for(unsigned int i = 0; i < mesh->vertexCount ; ++i)
    {
	double vx = mesh->vertices[3 * i    ];
	double vy = mesh->vertices[3 * i + 1];
	double vz = mesh->vertices[3 * i + 2];
	
	if (maxX < vx) maxX = vx;
	if (maxY < vy) maxY = vy;
	if (maxZ < vz) maxZ = vz;
	
	if (minX > vx) minX = vx;
	if (minY > vy) minY = vy;
	if (minZ > vz) minZ = vz;
    }
    
    if (maxX < minX) maxX = minX = 0.0;
    if (maxY < minY) maxY = minY = 0.0;
    if (maxZ < minZ) maxZ = minZ = 0.0;
    
    shapes::Box *box_shape = new shapes::Box(maxX - minX, maxY - minY, maxZ - minZ);
    m_boundingBox.setDimensions(box_shape);
    delete box_shape;
    
    m_boxOffset.setValue((minX + maxX) / 2.0, (minY + maxY) / 2.0, (minZ + maxZ) / 2.0);
    
    m_planes.clear();
    m_triangles.clear();
    m_vertices.clear();
    m_meshRadiusB = 0.0;
    m_meshCenter.setValue(tfScalar(0), tfScalar(0), tfScalar(0));

    btVector3 *vertices = new btVector3[mesh->vertexCount];
    for(unsigned int i = 0; i < mesh->vertexCount ; ++i)
    {
	vertices[i].setX(mesh->vertices[3 * i    ]);
	vertices[i].setY(mesh->vertices[3 * i + 1]);
	vertices[i].setZ(mesh->vertices[3 * i + 2]);
    }
    
    HullDesc hd(QF_TRIANGLES, mesh->vertexCount, vertices);
    HullResult hr;
    HullLibrary hl;
    if (hl.CreateConvexHull(hd, hr) == QE_OK)
    {
	//	std::cout << "Convex hull has " << hr.m_OutputVertices.size() << " vertices (down from " << mesh->vertexCount << "), " << hr.mNumFaces << " faces" << std::endl;

	m_vertices.reserve(hr.m_OutputVertices.size());
	tf::Vector3 sum(0, 0, 0);	
	
	for (int j = 0 ; j < hr.m_OutputVertices.size() ; ++j)
	{
            tf::Vector3 vector3_tmp(tf::Vector3(hr.m_OutputVertices[j][0],hr.m_OutputVertices[j][1],hr.m_OutputVertices[j][2]));
	    m_vertices.push_back(vector3_tmp);
	    sum = sum + vector3_tmp;
	}
	
	m_meshCenter = sum / (double)(hr.m_OutputVertices.size());
	for (unsigned int j = 0 ; j < m_vertices.size() ; ++j)
	{
	    double dist = m_vertices[j].distance2(m_meshCenter);
	    if (dist > m_meshRadiusB)
		m_meshRadiusB = dist;
	}
	m_meshRadiusB = sqrt(m_meshRadiusB);
	
	m_triangles.reserve(hr.m_Indices.size());
	for (unsigned int j = 0 ; j < hr.mNumFaces ; ++j)
	{
	    const tf::Vector3 p1(hr.m_OutputVertices[hr.m_Indices[j * 3    ]][0],hr.m_OutputVertices[hr.m_Indices[j * 3    ]][1],hr.m_OutputVertices[hr.m_Indices[j * 3    ]][2]);
	    const tf::Vector3 p2(hr.m_OutputVertices[hr.m_Indices[j * 3 + 1]][0],hr.m_OutputVertices[hr.m_Indices[j * 3 + 1]][1],hr.m_OutputVertices[hr.m_Indices[j * 3 + 1]][2]);
	    const tf::Vector3 p3(hr.m_OutputVertices[hr.m_Indices[j * 3 + 2]][0],hr.m_OutputVertices[hr.m_Indices[j * 3 + 2]][1],hr.m_OutputVertices[hr.m_Indices[j * 3 + 2]][2]);
	    
	    tf::Vector3 edge1 = (p2 - p1);
	    tf::Vector3 edge2 = (p3 - p1);
	    
	    edge1.normalize();
	    edge2.normalize();

	    tf::Vector3 planeNormal = edge1.cross(edge2);
	    
	    if (planeNormal.length2() > tfScalar(1e-6))
	    {
		planeNormal.normalize();
		tf::tfVector4 planeEquation(planeNormal.getX(), planeNormal.getY(), planeNormal.getZ(), -planeNormal.dot(p1));

		unsigned int behindPlane = countVerticesBehindPlane(planeEquation);
		if (behindPlane > 0)
		{
		    tf::tfVector4 planeEquation2(-planeEquation.getX(), -planeEquation.getY(), -planeEquation.getZ(), -planeEquation.getW());
		    unsigned int behindPlane2 = countVerticesBehindPlane(planeEquation2);
		    if (behindPlane2 < behindPlane)
		    {
			planeEquation.setValue(planeEquation2.getX(), planeEquation2.getY(), planeEquation2.getZ(), planeEquation2.getW());
			behindPlane = behindPlane2;
		    }
		}
		
		//		if (behindPlane > 0)
		//		    std::cerr << "Approximate plane: " << behindPlane << " of " << m_vertices.size() << " points are behind the plane" << std::endl;
		
		m_planes.push_back(planeEquation);

		m_triangles.push_back(hr.m_Indices[j * 3 + 0]);
		m_triangles.push_back(hr.m_Indices[j * 3 + 1]);
		m_triangles.push_back(hr.m_Indices[j * 3 + 2]);
	    }
	}
    }
    else
    	std::cerr << "Unable to compute convex hull.";
    
    hl.ReleaseResult(hr);    
    delete[] vertices;
    
}

void bodies::ConvexMesh::updateInternalData(void) 
{
    tf::Transform pose = m_pose;
    pose.setOrigin(m_pose * m_boxOffset);
    m_boundingBox.setPose(pose);
    m_boundingBox.setPadding(m_padding);
    m_boundingBox.setScale(m_scale);
    
    m_iPose = m_pose.inverse();
    m_center = m_pose * m_meshCenter;
    m_radiusB = m_meshRadiusB * m_scale + m_padding;
    m_radiusBSqr = m_radiusB * m_radiusB;

    m_scaledVertices.resize(m_vertices.size());
    for (unsigned int i = 0 ; i < m_vertices.size() ; ++i)
    {
	tf::Vector3 v(m_vertices[i] - m_meshCenter);
	tfScalar l = v.length();
	m_scaledVertices[i] = m_meshCenter + v * (m_scale + (l > ZERO ? m_padding / l : 0.0));
    }
}

void bodies::ConvexMesh::computeBoundingSphere(BoundingSphere &sphere) const
{
    sphere.center = m_center;
    sphere.radius = m_radiusB;
}

bool bodies::ConvexMesh::isPointInsidePlanes(const tf::Vector3& point) const
{
    unsigned int numplanes = m_planes.size();
    for (unsigned int i = 0 ; i < numplanes ; ++i)
    {
	const tf::tfVector4& plane = m_planes[i];
	tfScalar dist = plane.dot(point) + plane.getW() - m_padding - tfScalar(1e-6);
	if (dist > tfScalar(0))
	    return false;
    }
    return true;
}

unsigned int bodies::ConvexMesh::countVerticesBehindPlane(const tf::tfVector4& planeNormal) const
{
    unsigned int numvertices = m_vertices.size();
    unsigned int result = 0;
    for (unsigned int i = 0 ; i < numvertices ; ++i)
    {
	tfScalar dist = planeNormal.dot(m_vertices[i]) + planeNormal.getW() - tfScalar(1e-6);
	if (dist > tfScalar(0))
	    result++;
    }
    return result;
}

double bodies::ConvexMesh::computeVolume(void) const
{
    double volume = 0.0;
    for (unsigned int i = 0 ; i < m_triangles.size() / 3 ; ++i)
    {
	const tf::Vector3 &v1 = m_vertices[m_triangles[3*i + 0]];
        const tf::Vector3 &v2 = m_vertices[m_triangles[3*i + 1]];
	const tf::Vector3 &v3 = m_vertices[m_triangles[3*i + 2]];
	volume += v1.x()*v2.y()*v3.z() + v2.x()*v3.y()*v1.z() + v3.x()*v1.y()*v2.z() - v1.x()*v3.y()*v2.z() - v2.x()*v1.y()*v3.z() - v3.x()*v2.y()*v1.z();
    }
    return fabs(volume)/6.0;
}

bool bodies::ConvexMesh::intersectsRay(const tf::Vector3& origin, const tf::Vector3& dir, std::vector<tf::Vector3> *intersections, unsigned int count) const
{
    if (distanceSQR(m_center, origin, dir) > m_radiusBSqr) return false;
    if (!m_boundingBox.intersectsRay(origin, dir)) return false;
    
    // transform the ray into the coordinate frame of the mesh
    tf::Vector3 orig(m_iPose * origin);
    tf::Vector3 dr(m_iPose.getBasis() * dir);
    
    std::vector<detail::intersc> ipts;
    
    bool result = false;
    
    // for each triangle 
    const unsigned int nt = m_triangles.size() / 3;
    for (unsigned int i = 0 ; i < nt ; ++i)
    {
	tfScalar tmp = m_planes[i].dot(dr);
	if (fabs(tmp) > ZERO)
	{
	    double t = -(m_planes[i].dot(orig) + m_planes[i].getW()) / tmp;
	    if (t > 0.0)
	    {
		const int i3 = 3 * i;
		const int v1 = m_triangles[i3 + 0];
		const int v2 = m_triangles[i3 + 1];
		const int v3 = m_triangles[i3 + 2];
		
		const tf::Vector3 &a = m_scaledVertices[v1];
		const tf::Vector3 &b = m_scaledVertices[v2];
		const tf::Vector3 &c = m_scaledVertices[v3];
		
		tf::Vector3 cb(c - b);
		tf::Vector3 ab(a - b);
		
		// intersection of the plane defined by the triangle and the ray
		tf::Vector3 P(orig + dr * t);
		
		// check if it is inside the triangle
		tf::Vector3 pb(P - b);
		tf::Vector3 c1(cb.cross(pb));
		tf::Vector3 c2(cb.cross(ab));
		if (c1.dot(c2) < 0.0)
		    continue;
		
		tf::Vector3 ca(c - a);
		tf::Vector3 pa(P - a);
		tf::Vector3 ba(-ab);
		
		c1 = ca.cross(pa);
		c2 = ca.cross(ba);
		if (c1.dot(c2) < 0.0)
		    continue;
		
		c1 = ba.cross(pa);
		c2 = ba.cross(ca);
		
		if (c1.dot(c2) < 0.0)
		    continue;
		
		result = true;
		if (intersections)
		{
		    detail::intersc ip(origin + dir * t, t);
		    ipts.push_back(ip);
		}
		else
		    break;
	    }
	}
    }

    if (intersections)
    {
	std::sort(ipts.begin(), ipts.end(), detail::interscOrder());
	const unsigned int n = count > 0 ? std::min<unsigned int>(count, ipts.size()) : ipts.size();
	for (unsigned int i = 0 ; i < n ; ++i)
	    intersections->push_back(ipts[i].pt);
    }
    
    return result;
}
  
}
