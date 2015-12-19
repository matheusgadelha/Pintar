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
#include <Eigen/SparseCholesky>
#include <Eigen/SVD>

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
			for( size_t i=0; i<mMesh->vertices.size(); ++i )
			{
				meshVertices.push_back( mMesh->vertices[i].cast<double>() );
			}
			isControlVertex = std::vector<bool>( mMesh->vertices.size(), false );
			isFixedVertex = std::vector<bool>( mMesh->vertices.size(), false );
			clearData();

			laplacianComputed = false;
		}

		void clearData()
		{
			rotations.clear();
			pline.clear();
			b.clear();

			rotations.resize( mMesh->vertices.size(), Eigen::Matrix3d::Identity() );
			pline.resize( 3, Eigen::VectorXd( mMesh->vertices.size() ));
			b.resize( 3, Eigen::VectorXd( mMesh->vertices.size() ));
		}

		void setVertexPosition( int vid, Eigen::Vector3f newPos )
		{
			isControlVertex[vid] = true;
			meshVertices[vid] = newPos.cast<double>();
		}

		void setControlVertex( int vid )
		{
			isControlVertex[vid] = true;
			laplacianComputed = false;
		}

		void unsetControlVertex( int vid )
		{
			isControlVertex[vid] = false;
			laplacianComputed = false;
		}

		void setFixedVertex( int vid )
		{
			isFixedVertex[vid] = true;
			laplacianComputed = false;
		}

		void unsetFixedVertex( int vid )
		{
			isFixedVertex[vid] = false;
			laplacianComputed = false;
		}

		void translateControlVertices( Eigen::Vector3f t )
		{
			for( size_t i=0; i<isControlVertex.size(); ++i )
			{
				if( isControlVertex[i] )
				{
//					std::cout << meshVertices[i] << sep;
					meshVertices[i] += t.cast<double>();
//					std::cout << meshVertices[i] << sep;
				}
			}
			applyDeformation();
		}

		void clearFixedVertices()
		{
			for( size_t i=0; i<isFixedVertex.size(); ++i)
				isFixedVertex[i] = false;
		}

		void clearControlVertices()
		{
			for( size_t i=0; i<isControlVertex.size(); ++i)
				isControlVertex[i] = false;
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
				std::vector<double> ringWeights;
				for( size_t e = 0; e < mMesh->oneRings[v].size(); ++e )
				{
					size_t ov = mMesh->oneRings[v][e];
					std::vector<int> ovring = mMesh->oneRingFaces[ov];
					std::vector<int> vring = mMesh->oneRingFaces[v];

					std::vector<int> inter = intersection( vring, ovring );
//					std::cout << v << " " << ov << std::endl;
//					std::cout << inter.size() << std::endl;

					assert( inter.size() == 2 );
					std::vector<int> sharedvs;
					double w;
					if( inter.size() == 2 )
					{
						sharedvs.push_back( mMesh->sharedVertexOnFace(v, ov, inter[0]) );
						assert(sharedvs[0] != -1);
						sharedvs.push_back( mMesh->sharedVertexOnFace(v, ov, inter[1]) );
						assert(sharedvs[1] != -1);

						Eigen::Vector3d p = mMesh->vertices[v].cast<double>();
						Eigen::Vector3d p1 = mMesh->vertices[sharedvs[0]].cast<double>();
						Eigen::Vector3d p2 = mMesh->vertices[ov].cast<double>();
						Eigen::Vector3d p3 = mMesh->vertices[sharedvs[1]].cast<double>();

						w = (p-p1).dot(p2-p1) / (p-p1).cross(p2-p1).norm() + 
							(p-p3).dot(p2-p3) / (p-p3).cross(p2-p3).norm();
						w = w / 2.0f;
					} 
					else if( inter.size() == 1 )
					{
						sharedvs.push_back( mMesh->sharedVertexOnFace(v, ov, inter[0]) );
						assert(sharedvs[0] != -1);

						Eigen::Vector3d p = mMesh->vertices[v].cast<double>();
						Eigen::Vector3d p1 = mMesh->vertices[sharedvs[0]].cast<double>();
						Eigen::Vector3d p2 = mMesh->vertices[ov].cast<double>();

						w = (p-p1).dot(p2-p1) / (p-p1).cross(p2-p1).norm();
						w = w / 2.0f;
					}

					ringWeights.push_back( w );
				}

//				if( v==1 )std::cout << std::endl;
				weights.push_back( ringWeights );
			}
		}

		void preProcess()
		{
			computeWeights();
			clearData();

			laplacian = Eigen::SparseMatrix<double>( 
					mMesh->vertices.size(), mMesh->vertices.size() );

			for( size_t v=0; v < mMesh->oneRings.size(); ++v )
			{
				double w = 0.0f;
//				mMesh->changeVertexPos(v,meshVertices[v].cast<float>());
				if( !(isFixedVertex[v] || isControlVertex[v]) )
				{
					for( size_t itRing = 0; itRing < mMesh->oneRings[v].size(); ++itRing )
					{
						int adjV = mMesh->oneRings[v][itRing];
						w += weights[v][itRing];
						laplacian.coeffRef( v, adjV ) = -weights[v][itRing];
					}
				}
				else 
				{
					w = 1.0f;
				}

				laplacian.coeffRef(v,v) = w;
			}

//			std::cout << laplacian << sep;

			laplacianT = laplacian.transpose();
			solver.compute( laplacianT * laplacian );

			laplacianComputed = true;
		}

		void applyDeformation()
		{
			clearData();
			computeWeights();
			if( !laplacianComputed ) preProcess();
			for( int it=0; it<10; ++it )
			{
				for( size_t vid = 0; vid < mMesh->vertices.size(); ++vid )
				{
					Eigen::Vector3d p = meshVertices[vid];

					if( !(isFixedVertex[vid] || isControlVertex[vid]) )
					{
						p = Eigen::Vector3d::Zero();
						for( size_t itRing=0; itRing < mMesh->oneRings[vid].size(); ++itRing)
						{
							size_t adjV = mMesh->oneRings[vid][itRing];
							Eigen::Vector3d eij = mMesh->vertices[vid].cast<double>() - 
								mMesh->vertices[adjV].cast<double>();
							Eigen::Vector3d RiRjE = (rotations[vid]+rotations[adjV])*eij;

							p+=RiRjE * (weights[vid][itRing] / 2.0f);
						}
					}
					
					for( int i=0; i<3; ++i) b[i][vid] = p[i];

				}
				
//				for( size_t i=0; i<mMesh->vertices.size(); ++i)
//					std::cout << b[0][i] << " " << b[1][i] << " " << b[2][i] << sep;

				for( int i=0; i<3; ++i) 
					pline[i] = solver.solve(laplacianT * b[i]);

				if( it > 0 ) computeRotations();
			}

			for( size_t vid = 0; vid < mMesh->vertices.size(); ++vid )
			{
				meshVertices[vid] = Eigen::Vector3d(
						pline[0][vid], pline[1][vid], pline[2][vid]);

				mMesh->changeVertexPos(vid, meshVertices[vid].cast<float>());
			}
		}

		void computeRotations()
		{
			Eigen::Matrix3d id = Eigen::Matrix3d::Identity();

			for( size_t vid=0; vid<meshVertices.size(); ++vid )
			{
				int ringSize = mMesh->oneRings[vid].size();
				int degree = 0;

				Eigen::MatrixXd P(3, ringSize), Pline(3,ringSize);

				for( size_t itRing = 0; itRing < mMesh->oneRings[vid].size(); ++itRing )
				{
					int adjV = mMesh->oneRings[vid][itRing];

					P.col(degree) = (mMesh->vertices[vid].cast<double>() - 
							mMesh->vertices[adjV].cast<double>()) * weights[vid][itRing];

					Pline.col(degree++)=Eigen::Vector3d( pline[0][vid], pline[1][vid], pline[2][vid])
						-Eigen::Vector3d( pline[0][adjV], pline[1][adjV], pline[2][adjV]);
				}

				Eigen::MatrixXd S = P * Pline.transpose();

				Eigen::JacobiSVD<Eigen::MatrixXd> svd(S, Eigen::ComputeThinU|Eigen::ComputeThinV);
				Eigen::MatrixXd V = svd.matrixV();
				Eigen::MatrixXd Ut = svd.matrixU().transpose();

				//Trick to guarantee rotation, not reflection.
				id(2,2) = (V*Ut).determinant();
				rotations[vid] = (V*id*Ut);
				
//				std::cout << rotations[vid] << sep;
				
//				std::cout << P << sep;
//				std::cout << Pline << sep;
			}

		}

		std::vector<bool> isFixedVertex;
		std::vector<bool> isControlVertex;

	private:
		StandardMesh* mMesh;

		Eigen::SparseMatrix<double> laplacian;
		Eigen::SparseMatrix<double> laplacianT;
		Eigen::SimplicialLLT<Eigen::SparseMatrix<double> > solver;

		std::vector<Eigen::Vector3d> meshVertices;

		std::vector< std::vector<double> > weights;
		std::vector<Eigen::Matrix3d> rotations;
		std::vector<Eigen::VectorXd> pline;
		std::vector<Eigen::VectorXd> b;

		bool laplacianComputed;
};

}

#endif

