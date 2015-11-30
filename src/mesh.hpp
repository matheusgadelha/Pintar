#ifndef MESH_H_
#define MESH_H_

#define _USE_MATH_DEFINES

#include <utility>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <iostream>

#undef Success
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <boost/progress.hpp>

#include "object.hpp"

namespace Pintar
{

void printVector3f( Eigen::Vector4f v )
{
	std::cout << v[0] << " " << v[1] << " " << v[2] << std::endl;
}

template <typename T>
class Mesh : public Object
{
	public:

		Mesh() : Object(), 
			isRawVerticesAlloc(false){}

		~Mesh()
		{
			delete [] raw_vertices;
		}

		void generateMesh (std::vector<Eigen::Vector3f> _vertices, std::vector<int> _indices)
		{
			vertices	= _vertices;
			indices		= _indices;
			adjustBarycenter();
			computeRawVertices();
			computeAdjacencyMatrix( vertices.size() );
			createFaces();
		}

		void createFaces()
		{
			for( size_t i=0; i<indices.size(); i+=3 )
			{
				std::vector<int> face;
				face.push_back(indices[i]);
				face.push_back(indices[i+1]);
				face.push_back(indices[i+2]);
				
				mFaces.push_back( face );
			}
		}

		void computeAdjacencyMatrix( unsigned int size )
		{
			mAdjacency = Eigen::SparseMatrix<char>( size, size );
			mAdjacency.reserve(Eigen::VectorXi::Constant(size,6));
			
			std::cout << "Computing Adjacency Matrix...";
			boost::progress_display show_progress(indices.size() );

			for( size_t i=0; i<indices.size(); i+=3 )
			{
				mAdjacency.coeffRef(indices[i],indices[i+1]) = 1;
				mAdjacency.coeffRef(indices[i+1],indices[i]) = 1;

				mAdjacency.coeffRef(indices[i],indices[i+2]) = 1;
				mAdjacency.coeffRef(indices[i+2],indices[i]) = 1;

				mAdjacency.coeffRef(indices[i+1],indices[i+2]) = 1;
				mAdjacency.coeffRef(indices[i+2],indices[i+1]) = 1;

				show_progress+=3;
			}
			std::cout << "Done." << std::endl;
		}

		void allocateRawVertices()
		{
			if (!isRawVerticesAlloc)
			{
				raw_vertices		= new T[indices.size()*3];
				raw_vertices_size	= indices.size()*3;
				isRawVerticesAlloc	= true;
			}
		}

		void computeRawVertices()
		{
			allocateRawVertices();
			for (int i=0; i<indices.size(); ++i)
			{
				raw_vertices[3*i]	= vertices[indices[i]][0];
				raw_vertices[3*i+1]	= vertices[indices[i]][1];
				raw_vertices[3*i+2]	= vertices[indices[i]][2];
			}
		}

		void adjustBarycenter()
		{
			Eigen::Vector3f barycenter;
			for( size_t i=0; i<vertices.size(); ++i )
			{
				barycenter += vertices[i];
			}
			barycenter =  barycenter / (float)vertices.size();

			for( size_t i=0; i<vertices.size(); ++i )
			{
				vertices[i] -= barycenter;
			}
		}

		std::vector<Eigen::Vector3f> vertices;
		T* raw_vertices;
		size_t raw_vertices_size;
		std::vector<int> indices;
		std::vector< std::vector<int> > mFaces;
		std::vector< Eigen::Vector3f > mFaceNormals;
		bool isRawVerticesAlloc;

		Eigen::SparseMatrix<char> mAdjacency;
};

typedef Mesh<float> StandardMesh;

}

#endif
