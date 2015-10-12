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

#include "robot_self_filter/shapes.h"
namespace robot_self_filter
{

shapes::Shape* shapes::cloneShape(const shapes::Shape *shape)
{
    shapes::Shape *result = NULL;
    switch (shape->type)
    {
    case SPHERE:
	result = new Sphere(static_cast<const Sphere*>(shape)->radius);
	break;
    case CYLINDER:
	result = new Cylinder(static_cast<const Cylinder*>(shape)->radius, static_cast<const Cylinder*>(shape)->length);
	break;
    case BOX:
	result = new Box(static_cast<const Box*>(shape)->size[0], static_cast<const Box*>(shape)->size[1], static_cast<const Box*>(shape)->size[2]);
	break;
    case MESH:
	{
	    const Mesh *src = static_cast<const Mesh*>(shape);
	    Mesh *dest = new Mesh(src->vertexCount, src->triangleCount);
	    unsigned int n = 3 * src->vertexCount;
	    for (unsigned int i = 0 ; i < n ; ++i)
		dest->vertices[i] = src->vertices[i];
	    n = 3 * src->triangleCount;
	    for (unsigned int i = 0 ; i < n ; ++i)
	    {
		dest->triangles[i] = src->triangles[i];
		dest->normals[i] = src->normals[i];
	    }
	    result = dest;
	}
	break;
    default:
	break;
    }
    return result;
}

shapes::StaticShape* shapes::cloneShape(const shapes::StaticShape *shape)
{
    shapes::StaticShape *result = NULL;
    switch (shape->type)
    {
    case PLANE:
	result = new Plane(static_cast<const Plane*>(shape)->a, static_cast<const Plane*>(shape)->b, 
			   static_cast<const Plane*>(shape)->c, static_cast<const Plane*>(shape)->d);
	break;
    default:
	break;
    }
    
    return result;
}

}
