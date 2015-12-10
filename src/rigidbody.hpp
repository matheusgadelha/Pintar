#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <GL/OOGL.hpp>

#include <iostream>
#include <algorithm>
#include <string>
#include <fstream>
#include <cmath>
#undef Success
#include <Eigen/Dense>

#include "object.hpp"
#include "mesh.hpp"

namespace Pintar
{

struct Force
{
	Force( Eigen::Vector3f v, Eigen::Vector3f p ) : vec(v), pos(p){}
	Force( Eigen::Vector3f v ) : vec(v), pos(0,1.5f,0){}
	Eigen::Vector3f vec;
	Eigen::Vector3f pos;
};

class RigidBody
{
	public:
		RigidBody( StandardMesh* b ) : body(b), mass(1.0f)
		{
			position = body->getPosition();
			orientation = body->getOrientation();
			velocity = Eigen::Vector3f(0,0,0);
			acceleration = Eigen::Vector3f(0,0,0);
			angularVelocity = Eigen::Quaternionf::Identity();
			angularAcceleration = Eigen::Vector3f(0,0,0);

			computeMomentOfInertia();
		}

		Eigen::Vector3f resultingForce()
		{
			Eigen::Vector3f result(0,0,0);
			for( size_t i=0; i<appliedForces.size(); ++i )
				result = result + appliedForces[i].vec;

			return result;
		}

		void applyForce( Force f )
		{
			appliedForces.push_back( f );
		}

		void simulateEuler( float dt )
		{
			Eigen::Vector3f rf = resultingForce();
			acceleration = rf / mass;
			position = position + dt*velocity;
			velocity = velocity + dt*acceleration;

			angularAcceleration = dt*worldSpaceMOI().inverse() * totalTorque();
			angularVelocity = Eigen::Quaternionf( 0, angularAcceleration[0]/2, 
					angularAcceleration[1]/2,
					angularAcceleration[2]/2) * orientation;

			orientation = angularVelocity*orientation;

			body->setPosition( position );
			body->setRotation( orientation );
			clearForces();
		}

		Eigen::Vector3f totalTorque()
		{
			Eigen::Vector3f barycenter = body->barycenter();
			Eigen::Vector3f torque(0,0,0);

			for( size_t i=0; i<appliedForces.size(); ++i )
			{
				torque = torque + (appliedForces[i].pos - barycenter).cross(appliedForces[i].vec);
			}
			return torque;
		}

		Eigen::Matrix3f angularVelocityMatrix( Eigen::Vector3f w )
		{
			Eigen::Matrix3f result = Eigen::Matrix3f::Zero();
			result(0,1) = -w[2];
			result(0,2) = w[1];
			result(1,0) = w[2];
			result(1,2) = -w[0];
			result(2,0) = -w[1];
			result(2,1) = w[0];
			
			return result;
		}

		void computeMomentOfInertia()
		{
			Eigen::Matrix3f result = Eigen::Matrix3f::Zero();
			Eigen::Vector3f barycenter = body->barycenter();

			for( size_t i=0; i<body->vertices.size(); ++i )
			{
				Eigen::Vector3f r = body->vertices[i] - barycenter;
				result(0,0) += r[1]*r[1] + r[2]*r[2];
				result(0,1) += -r[0]*r[1];
				result(0,2) += -r[0]*r[2];
				result(1,1) += r[0]*r[0] + r[2]*r[2];
				result(1,0) += -r[1]*r[0];
				result(1,2) += -r[1]*r[2];
				result(2,2) += r[0]*r[0] + r[1]*r[1];
				result(2,0) += -r[2]*r[0];
				result(2,1) += -r[2]*r[1];
			}

			momentOfInertia = mass*result;
		}

		Eigen::Matrix3f worldSpaceMOI()
		{
			return orientation.matrix() * momentOfInertia * orientation.matrix().transpose();
		}

		void clearForces(){ appliedForces.clear(); }

		Eigen::Vector3f position;
		Eigen::Quaternionf orientation;
		Eigen::Vector3f velocity;
		Eigen::Quaternionf angularVelocity;
		Eigen::Vector3f acceleration;
		Eigen::Vector3f angularAcceleration;
		std::vector<Force> appliedForces;
		Eigen::Matrix3f momentOfInertia;

		StandardMesh* body;
		
		float mass;
};

}


#endif

