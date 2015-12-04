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

#include <cstdio>
#include <cmath>
#include <algorithm>
#include <set>
#include "robot_self_filter/shapes.h"
#include <resource_retriever/retriever.h>
#include <ros/assert.h>
#include <tinyxml.h>
#if defined(ASSIMP_UNIFIED_HEADER_NAMES)
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>
#else
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>
#include <assimp/IOStream.h>
#include <assimp/IOSystem.h>
#endif


// \author Ioan Sucan ;  based on stl_to_mesh 
namespace robot_self_filter
{
namespace shapes
{

    // Assimp wrappers
    class ResourceIOStream : public Assimp::IOStream
    {
    public:
	ResourceIOStream(const resource_retriever::MemoryResource& res)
	: res_(res)
	, pos_(res.data.get())
	{}

	~ResourceIOStream()
	{}

	size_t Read(void* buffer, size_t size, size_t count)
	{
	  size_t to_read = size * count;
	  if (pos_ + to_read > res_.data.get() + res_.size)
	  {
	    to_read = res_.size - (pos_ - res_.data.get());
	  }

	  memcpy(buffer, pos_, to_read);
	  pos_ += to_read;

	  return to_read;
	}

	size_t Write( const void* buffer, size_t size, size_t count) { ROS_BREAK(); return 0; }

	aiReturn Seek( size_t offset, aiOrigin origin)
	{
	  uint8_t* new_pos = 0;
	  switch (origin)
	  {
	  case aiOrigin_SET:
	    new_pos = res_.data.get() + offset;
	    break;
	  case aiOrigin_CUR:
	    new_pos = pos_ + offset; // TODO is this right?  can offset really not be negative
	    break;
	  case aiOrigin_END:
	    new_pos = res_.data.get() + res_.size - offset; // TODO is this right?
	    break;
	  default:
	    ROS_BREAK();
	  }

	  if (new_pos < res_.data.get() || new_pos > res_.data.get() + res_.size)
	  {
	    return aiReturn_FAILURE;
	  }

	  pos_ = new_pos;
	  return aiReturn_SUCCESS;
	}

	size_t Tell() const
	{
	  return pos_ - res_.data.get();
	}

	size_t FileSize() const
	{
	  return res_.size;
	}

	void Flush() {}

    private:
	resource_retriever::MemoryResource res_;
	uint8_t* pos_;
    };

  
    class ResourceIOSystem : public Assimp::IOSystem
    {
    public:
      ResourceIOSystem()
      {
      }

      ~ResourceIOSystem()
      {
      }

      // Check whether a specific file exists
      bool Exists(const char* file) const
      {
	// Ugly -- two retrievals where there should be one (Exists + Open)
	// resource_retriever needs a way of checking for existence
	// TODO: cache this
	resource_retriever::MemoryResource res;
	try
	{
	  res = retriever_.get(file);
	}
	catch (resource_retriever::Exception& e)
	{
	  return false;
	}

	return true;
      }

      // Get the path delimiter character we'd like to see
      char getOsSeparator() const
      {
	return '/';
      }

      // ... and finally a method to open a custom stream
      Assimp::IOStream* Open(const char* file, const char* mode = "rb")
      {
	// Ugly -- two retrievals where there should be one (Exists + Open)
	// resource_retriever needs a way of checking for existence
	resource_retriever::MemoryResource res;
	try
	{
	  res = retriever_.get(file);
	}
	catch (resource_retriever::Exception& e)
	{
	  return 0;
	}

	return new ResourceIOStream(res);
      }

      void Close(Assimp::IOStream* stream);

    private:
      mutable resource_retriever::Retriever retriever_;
    };

    void ResourceIOSystem::Close(Assimp::IOStream* stream)
    {
      delete stream;
    }

    float getMeshUnitRescale(const std::string& resource_path)
    {
      static std::map<std::string, float> rescale_cache;

      // Try to read unit to meter conversion ratio from mesh. Only valid in Collada XML formats. 
      TiXmlDocument xmlDoc;
      float unit_scale(1.0);
      resource_retriever::Retriever retriever;
      resource_retriever::MemoryResource res;
      try
      {
	res = retriever.get(resource_path);
      }
      catch (resource_retriever::Exception& e)
      {
	ROS_ERROR("%s", e.what());
	return unit_scale;
      }
  
      if (res.size == 0)
      {
	return unit_scale;
      }


      // Use the resource retriever to get the data.
      const char * data = reinterpret_cast<const char * > (res.data.get());
      xmlDoc.Parse(data);

      // Find the appropriate element if it exists
      if(!xmlDoc.Error())
      {
	TiXmlElement * colladaXml = xmlDoc.FirstChildElement("COLLADA");
	if(colladaXml)
	{
	  TiXmlElement *assetXml = colladaXml->FirstChildElement("asset");
	  if(assetXml)
	  {
	    TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
	    if (unitXml && unitXml->Attribute("meter"))
	    {
	      // Failing to convert leaves unit_scale as the default.
	      if(unitXml->QueryFloatAttribute("meter", &unit_scale) != 0)
		ROS_WARN_STREAM("getMeshUnitRescale::Failed to convert unit element meter attribute to determine scaling. unit element: "
				<< *unitXml);
	    }
	  }
	}
      }
      return unit_scale;
    }

    std::vector<tf::Vector3> getVerticesFromAssimpNode(const aiScene* scene, const aiNode* node, const float scale)
    {
	std::vector<tf::Vector3> vertices;
	if (!node)
	{
	  return vertices;
	}
	
	aiMatrix4x4 transform = node->mTransformation;
	aiNode *pnode = node->mParent;
	while (pnode)
	{
	  // Don't convert to y-up orientation, which is what the root node in
	  // Assimp does
	  if (pnode->mParent != NULL)
	    transform = pnode->mTransformation * transform;
	  pnode = pnode->mParent;
	}
	
	aiMatrix3x3 rotation(transform);
	aiMatrix3x3 inverse_transpose_rotation(rotation);
	inverse_transpose_rotation.Inverse();
	inverse_transpose_rotation.Transpose();
	
	for (uint32_t i = 0; i < node->mNumMeshes; i++)
	{
	  aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];
	  // Add the vertices
	  for (uint32_t j = 0; j < input_mesh->mNumVertices; j++)
	  {
	    aiVector3D p = input_mesh->mVertices[j];
	    p *= transform;
	    p *= scale;
	    tf::Vector3 v(p.x, p.y, p.z);
	    vertices.push_back(v);
	  }
	}
	
	for (uint32_t i=0; i < node->mNumChildren; ++i)
	{
	  std::vector<tf::Vector3> sub_vertices = getVerticesFromAssimpNode(scene,node->mChildren[i], scale);
	  // Add vertices
	  for (size_t j = 0; j < sub_vertices.size(); j++) {
	    vertices.push_back(sub_vertices[j]);
	  }
	}
	return vertices;
    }
  
    shapes::Mesh* meshFromAssimpScene(const std::string& name, const aiScene* scene)
    {
      if (!scene->HasMeshes())
      {
	ROS_ERROR("No meshes found in file [%s]", name.c_str());
	return NULL;
      }
      
      float scale = getMeshUnitRescale(name);

      std::vector<tf::Vector3> vertices = getVerticesFromAssimpNode(scene, scene->mRootNode, scale);
      
      return createMeshFromVertices(vertices);
    }
  
    namespace detail
    {
	struct myVertex
	{
	    tf::Vector3    point;
	    unsigned int index;
	};
	
	struct ltVertexValue
	{
	    bool operator()(const myVertex &p1, const myVertex &p2) const
	    {
		const tf::Vector3 &v1 = p1.point;
		const tf::Vector3 &v2 = p2.point;
		if (v1.getX() < v2.getX())
		    return true;
		if (v1.getX() > v2.getX())
		    return false;
		if (v1.getY() < v2.getY())
		    return true;
		if (v1.getY() > v2.getY())
		    return false;
		if (v1.getZ() < v2.getZ())
		    return true;
		return false;
	    }
	};
	
	struct ltVertexIndex
	{
	    bool operator()(const myVertex &p1, const myVertex &p2) const
	    {
		return p1.index < p2.index;
	    }
	};
    }
    
    shapes::Mesh* createMeshFromVertices(const std::vector<tf::Vector3> &vertices, const std::vector<unsigned int> &triangles)
    {
	unsigned int nt = triangles.size() / 3;
	shapes::Mesh *mesh = new shapes::Mesh(vertices.size(), nt);
	for (unsigned int i = 0 ; i < vertices.size() ; ++i)
	{
	    mesh->vertices[3 * i    ] = vertices[i].getX();
	    mesh->vertices[3 * i + 1] = vertices[i].getY();
	    mesh->vertices[3 * i + 2] = vertices[i].getZ();
	}
	
	std::copy(triangles.begin(), triangles.end(), mesh->triangles);
	
	// compute normals 
	for (unsigned int i = 0 ; i < nt ; ++i)
	{
	    tf::Vector3 s1 = vertices[triangles[i * 3    ]] - vertices[triangles[i * 3 + 1]];
	    tf::Vector3 s2 = vertices[triangles[i * 3 + 1]] - vertices[triangles[i * 3 + 2]];
	    tf::Vector3 normal = s1.cross(s2);
	    normal.normalize();
	    mesh->normals[3 * i    ] = normal.getX();
	    mesh->normals[3 * i + 1] = normal.getY();
	    mesh->normals[3 * i + 2] = normal.getZ();
	}
	return mesh;
    }
    
    shapes::Mesh* createMeshFromVertices(const std::vector<tf::Vector3> &source)
    {
	if (source.size() < 3)
	    return NULL;
	
	std::set<detail::myVertex, detail::ltVertexValue> vertices;
	std::vector<unsigned int>                         triangles;
	
	for (unsigned int i = 0 ; i < source.size() / 3 ; ++i)
	{
	    // check if we have new vertices
	    detail::myVertex vt;
	    
	    vt.point = source[3 * i];
	    std::set<detail::myVertex, detail::ltVertexValue>::iterator p1 = vertices.find(vt);
	    if (p1 == vertices.end())
	    {
		vt.index = vertices.size();
		vertices.insert(vt);		    
	    }
	    else
		vt.index = p1->index;
	    triangles.push_back(vt.index);		
	    
	    vt.point = source[3 * i + 1];
	    std::set<detail::myVertex, detail::ltVertexValue>::iterator p2 = vertices.find(vt);
	    if (p2 == vertices.end())
	    {
		vt.index = vertices.size();
		vertices.insert(vt);		    
	    }
	    else
		vt.index = p2->index;
	    triangles.push_back(vt.index);		
	    
	    vt.point = source[3 * i + 2];
	    std::set<detail::myVertex, detail::ltVertexValue>::iterator p3 = vertices.find(vt);
	    if (p3 == vertices.end())
	    {
		vt.index = vertices.size();
		vertices.insert(vt);		    
	    }
	    else
		vt.index = p3->index;

	    triangles.push_back(vt.index);
	}
	
	// sort our vertices
	std::vector<detail::myVertex> vt;
	vt.insert(vt.begin(), vertices.begin(), vertices.end());
	std::sort(vt.begin(), vt.end(), detail::ltVertexIndex());
	
	// copy the data to a mesh structure 
	unsigned int nt = triangles.size() / 3;
	
	shapes::Mesh *mesh = new shapes::Mesh(vt.size(), nt);
	for (unsigned int i = 0 ; i < vt.size() ; ++i)
	{
	    mesh->vertices[3 * i    ] = vt[i].point.getX();
	    mesh->vertices[3 * i + 1] = vt[i].point.getY();
	    mesh->vertices[3 * i + 2] = vt[i].point.getZ();
	}
	
	std::copy(triangles.begin(), triangles.end(), mesh->triangles);
	
	// compute normals 
	for (unsigned int i = 0 ; i < nt ; ++i)
	{
	    tf::Vector3 s1 = vt[triangles[i * 3    ]].point - vt[triangles[i * 3 + 1]].point;
	    tf::Vector3 s2 = vt[triangles[i * 3 + 1]].point - vt[triangles[i * 3 + 2]].point;
	    tf::Vector3 normal = s1.cross(s2);
	    normal.normalize();
	    mesh->normals[3 * i    ] = normal.getX();
	    mesh->normals[3 * i + 1] = normal.getY();
	    mesh->normals[3 * i + 2] = normal.getZ();
	}
	
	return mesh;
    }

    shapes::Mesh* createMeshFromBinaryStlData(const char *data, unsigned int size)
    {
	const char* pos = data;
	pos += 80; // skip the 80 byte header
	
	unsigned int numTriangles = *(unsigned int*)pos;
	pos += 4;
	
	// make sure we have read enough data
	if ((long)(50 * numTriangles + 84) <= size)
	{
	    std::vector<tf::Vector3> vertices;
	    
	    for (unsigned int currentTriangle = 0 ; currentTriangle < numTriangles ; ++currentTriangle)
	    {
		// skip the normal
		pos += 12;
		
		// read vertices 
		tf::Vector3 v1(0,0,0);
		tf::Vector3 v2(0,0,0);
		tf::Vector3 v3(0,0,0);
		
		v1.setX(*(float*)pos);
		pos += 4;
		v1.setY(*(float*)pos);
		pos += 4;
		v1.setZ(*(float*)pos);
		pos += 4;
		
		v2.setX(*(float*)pos);
		pos += 4;
		v2.setY(*(float*)pos);
		pos += 4;
		v2.setZ(*(float*)pos);
		pos += 4;
		
		v3.setX(*(float*)pos);
		pos += 4;
		v3.setY(*(float*)pos);
		pos += 4;
		v3.setZ(*(float*)pos);
		pos += 4;
		
		// skip attribute
		pos += 2;
		
		vertices.push_back(v1);
		vertices.push_back(v2);
		vertices.push_back(v3);
	    }
	    
	    return createMeshFromVertices(vertices);
	}
	
	return NULL;
    }

    shapes::Mesh* createMeshFromBinaryDAE(const char* filename)
    {
      std::string resource_path(filename);
      Assimp::Importer importer;
      importer.SetIOHandler(new ResourceIOSystem());
      const aiScene* scene = importer.ReadFile(resource_path, aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords|aiProcess_FlipUVs);
      if (!scene)
      {
        ROS_ERROR("Could not load resource [%s]: %s", resource_path.c_str(), importer.GetErrorString());
        return NULL;
      }
      return meshFromAssimpScene(resource_path, scene);
    }
  
    shapes::Mesh* createMeshFromBinaryStl(const char *filename)
    {
	FILE* input = fopen(filename, "r");
	if (!input)
	    return NULL;
	
	fseek(input, 0, SEEK_END);
	long fileSize = ftell(input);
	fseek(input, 0, SEEK_SET);
	
	char* buffer = new char[fileSize];
	size_t rd = fread(buffer, fileSize, 1, input);
	
	fclose(input);
	
	shapes::Mesh *result = NULL;
	
	if (rd == 1)
    	    result = createMeshFromBinaryStlData(buffer, fileSize);
	
	delete[] buffer;
	
	return result;
    }
    
}
}
