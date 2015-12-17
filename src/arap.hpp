#ifndef ARAP_H_
#define ARAP_H_

#define _USE_MATH_DEFINES

#include <utility>
#include <vector>
#include <deque>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <iostream>
#include <cassert>

#undef Success
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <boost/progress.hpp>

#include "mesh.hpp"

namespace Pintar
{

class ARAPDeform
{
	public:
		ARAPDeform( StandardMesh* mesh )
		{
			mMesh = mesh;
			computeWeights();
		}

		std::vector<int> intersection( std::vector<int>& a, std::vector<int>& b )
		{
			std::vector<int> result;
			std::sort(a.begin(), a.end());
			std::sort(b.begin(), b.end());

			std::set_intersection( a.begin(), a.end(), b.begin(), b.end(), 
					std::back_inserter( result ));

			return result;
		}

		void computeWeights()
		{
			for( size_t v = 0; v < mMesh->oneRings.size(); ++v )
			{
				std::vector<float> ringWeights;
//				std::cout << mMesh->oneRings[v].size() << std::endl;
				for( size_t e = 0; e < mMesh->oneRings[v].size(); ++e )
				{
					size_t ov = mMesh->oneRings[v][e];
					std::vector<int> ovring = mMesh->oneRings[ov];
					std::vector<int> vring = mMesh->oneRings[v];

					std::vector<int> inter = intersection( vring, ovring );
					assert( inter.size() == 2 );

					Eigen::Vector3f p = mMesh->vertices[v];
					Eigen::Vector3f p1 = mMesh->vertices[inter[0]];
					Eigen::Vector3f p2 = mMesh->vertices[ov];
					Eigen::Vector3f p3 = mMesh->vertices[inter[1]];

					float w = (p-p1).dot(p2-p1) / (p-p1).cross(p2-p1).norm() + 
						(p-p3).dot(p2-p3) / (p-p3).cross(p2-p3).norm();
					w = w / 2.0f;

					ringWeights.push_back( w );
				}
				weights.push_back( ringWeights );
			}
		}

	private:
		StandardMesh* mMesh;
		std::vector< std::vector<float> > weights;
};

}

#endif

