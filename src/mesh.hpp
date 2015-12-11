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

const std::string sep = "\n-----\n";

template <typename T>
class Mesh : public Object
{
	public:

		Mesh() : Object(), 
			isRawDataAlloc(false), baseColor(1.0f,0.5f,0.0f){}

		~Mesh()
		{
			delete [] raw_data;
		}

		void generateMesh (const std::vector<Eigen::Vector3f>& _vertices, 
				const std::vector<int>& _indices)
		{
			vertices	= _vertices;
			indices		= _indices;
			adjustBarycenter();
//			computeAdjacencyMatrix( vertices.size() );
			createFaces();
			createFaceNormals();
			computeVertexNormals();
			computeColors();
			computeRawData();

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

		Eigen::Vector3f getVertex( int idx )
		{
			return vertices[idx];
		}

		void createFaceNormals()
		{
			for( size_t i=0; i<mFaces.size(); ++i )
			{
				Eigen::Vector3f v0v1 = getVertex(mFaces[i][1]) - getVertex(mFaces[i][0]);
				Eigen::Vector3f v0v2 = getVertex(mFaces[i][2]) - getVertex(mFaces[i][0]);
				mFaceNormals.push_back( v0v1.cross(v0v2).normalized() );
				mFaceAreas.push_back( v0v1.cross(v0v2).squaredNorm() );
			}
		}

		void computeVertexNormals()
		{
			std::cout << "Computing normals...";
			boost::progress_display show_progress( vertices.size() );
			for( size_t i=0; i<vertices.size(); ++i )
			{
				std::vector<int> oneRingFaces = getOneRingFaces(i);
				float sumAreas = sumOfFaceAreas( oneRingFaces );
				Eigen::Vector3f vnormal(0,0,0);
				for( size_t j = 0; j<oneRingFaces.size(); ++j )
				{
					vnormal += (1.0f/sumAreas) * mFaceAreas[oneRingFaces[j]] *
						mFaceNormals[oneRingFaces[j]];
				}
				mVertexNormals.push_back( vnormal.normalized() );

				++show_progress;
			}
			std::cout << "Done." << std::endl;
		}

		float sumOfFaceAreas( std::vector<int> faces_idx )
		{
			float result = 0.0f;
			for( size_t i=0; i<faces_idx.size(); ++i )
			{
				result += mFaceAreas[faces_idx[i]];
			}
			return result;
		}

		void computeColors()
		{
			for( size_t i=0; i<vertices.size(); ++i )
			{
				mVertexColors.push_back( baseColor );
			}
		}

		std::vector<int> getOneRingFaces( int idx )
		{
			std::vector<int> result;
			for( size_t i=0; i<mFaces.size(); ++i )
			{
				if( std::find( mFaces[i].begin(), mFaces[i].end(), idx ) != mFaces[i].end() )
				{
					result.push_back( i );
				}
			}
			return result;
		}

		std::vector<int> getOneRingVertices( int idx )
		{
			std::vector<int> result;
			for( int i=0; i<mFaces.size(); ++i )
			{
				if( std::find( mFaces[i].begin(), mFaces[i].end(), idx ) != mFaces[i].end() )
				{
					result.insert( result.end(), mFaces[i].begin(), mFaces[i].end() );
				}
			}
			std::sort( result.begin(), result.end() );
			result.erase( std::unique( result.begin(), result.end() ), result.end());
			return result;
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

		void allocateRawData()
		{
			if (!isRawDataAlloc)
			{
				raw_data		= new T[indices.size()*3*3];
				raw_data_size	= indices.size()*3*3;
				isRawDataAlloc	= true;
			}
		}

		void computeRawData()
		{
			allocateRawData();
			for (int i=0; i<indices.size(); ++i)
			{
				raw_data[3*i]	= vertices[indices[i]][0];
				raw_data[3*i+1]	= vertices[indices[i]][1];
				raw_data[3*i+2]	= vertices[indices[i]][2];
			}
			
			for (int i=0; i<indices.size(); ++i)
			{
				raw_data[3*i + indices.size()*3]	= mVertexNormals[indices[i]][0];
				raw_data[3*i+1 + indices.size()*3]	= mVertexNormals[indices[i]][1];
				raw_data[3*i+2 + indices.size()*3]	= mVertexNormals[indices[i]][2];
			}
			
			for (int i=0; i<indices.size(); ++i)
			{
				raw_data[3*i + indices.size()*3*2]		= mVertexColors[indices[i]][0];
				raw_data[3*i+1 + indices.size()*3*2]	= mVertexColors[indices[i]][1];
				raw_data[3*i+2 + indices.size()*3*2]	= mVertexColors[indices[i]][2];
			}
		}

		Eigen::Vector3f barycenter()
		{
			Eigen::Vector3f b(0,0,0);
			for( size_t i=0; i<vertices.size(); ++i )
			{
				b += vertices[i];
			}
			return b/ (float)vertices.size();
		}

		void adjustBarycenter()
		{
			Eigen::Vector3f b = barycenter();
			for( size_t i=0; i<vertices.size(); ++i )
			{
				vertices[i] -= b;
			}
		}

		std::vector<Eigen::Vector3f> vertices;
		T* raw_data;
		size_t raw_data_size;
		std::vector<int> indices;
		std::vector< std::vector<int> > mFaces;
		std::vector< Eigen::Vector3f > mFaceNormals;
		std::vector< Eigen::Vector3f > mVertexNormals;
		std::vector< Eigen::Vector3f > mVertexColors;
		std::vector<float> mFaceAreas;
		bool isRawDataAlloc;
		Eigen::Vector3f baseColor;

		Eigen::SparseMatrix<char> mAdjacency;
};

typedef Mesh<float> StandardMesh;

}

#endif
