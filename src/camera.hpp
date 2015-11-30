#ifndef CAMERA_H
#define CAMERA_H

#include <GL/OOGL.hpp>

#include <iostream>
#include <algorithm>
#include <string>
#include <fstream>
#include <cmath>
#undef Success
#include <Eigen/Dense>

#include "object.hpp"

namespace Pintar
{

class Camera : public Object
{
	public:
		Camera() : Object(){
			mView = Eigen::Matrix4f::Identity();
			mProjection = Eigen::Matrix4f::Identity();
			lookAt(Eigen::Vector3f(0,0,-1), Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,1,0));
			setPerspectiveProjection(3.14/3.0f,4.0f/3.0,0.1f, 1000.0f);
		}

		void lookAt(
				const Eigen::Vector3f& position, 
				const Eigen::Vector3f& target, 
				const Eigen::Vector3f& up)
		{
			Eigen::Matrix3f R;
			Eigen::Matrix4f viewMatrix;

			R.col(2) = (target-position).normalized();
			R.col(0) = up.cross(R.col(2)).normalized();
			R.col(1) = R.col(2).cross(R.col(0));

			Eigen::Quaternionf qr(R);
			setRotation( qr );
			setPosition( position );
		}

		void setPerspectiveProjection(float fovY, float aspect, float near, float far)
		{
			mNearPlane = near;
			mFarPlane = far;

			Eigen::Matrix4f projectionMatrix = Eigen::Matrix4f::Identity();
			float theta = fovY*0.5;
			float range = far - near;
			float invtan = 1./tan(theta);

			projectionMatrix(0,0) = invtan / aspect;
			projectionMatrix(1,1) = invtan;
			projectionMatrix(2,2) = -(near + far) / range;
			projectionMatrix(3,2) = -1;
			projectionMatrix(2,3) = -2 * near * far / range;
			projectionMatrix(3,3) = 0;

			mProjection = projectionMatrix;
		}

		Eigen::Matrix4f view()
		{
			Eigen::Matrix3f rotation = transform().topLeftCorner<3,3>();
			Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

			view.topLeftCorner<3,3>() = rotation.transpose();
			view.topRightCorner<3,1>() = -transform().topRightCorner<3,1>();
//			view.topRightCorner<3,1>() = -mPosition;
//			std::cout << view << "\n---\n";

			mView = view;
			return view;
		}

		Eigen::Matrix4f projection()
		{
			return mProjection;
		}

		Eigen::Vector3f cameraToWorld( Eigen::Vector2f p )
		{
			Eigen::Vector4f ps( p[0], p[1], mNearPlane, 1 );
			Eigen::Matrix4f invProjView = (mProjection*mView).inverse();
			Eigen::Vector4f result4 =  invProjView*ps;

			return Eigen::Vector3f(result4[0], result4[1], result4[2])/result4[3];
		}

	private:
		Eigen::Matrix4f mView;
		Eigen::Matrix4f mProjection;
		float mNearPlane;
		float mFarPlane;
};

class ArcballCamera : public Camera
{
	public:
		ArcballCamera() : Camera(), radius(1.0f){}

		void move( Eigen::Vector2f x1, Eigen::Vector2f x2 )
		{
			Eigen::Vector3f w1 = cameraToWorld(x1);
			Eigen::Vector3f w2 = cameraToWorld(x2);

			w1[2] = sqrt( radius*radius - w1[1]*w1[1] - w1[0]*w1[0] );
			w2[2] = sqrt( radius*radius - w2[1]*w1[1] - w2[0]*w2[0] );

			w1.normalize();
			w2.normalize();

			Eigen::Vector3f rotAxis = w1.cross(w2);
			float angle = acos( std::min(1.0f, w1.dot(w2)) );
			Eigen::Quaternionf rot;
			rot = Eigen::AngleAxis<float>(angle,rotAxis);
			
			Eigen::Vector3f newPos = rot.matrix()*mPosition;
			lookAt( newPos, getParentPosition(), Eigen::Vector3f(0,0,1));
		}

		float radius;
};

}

#endif

