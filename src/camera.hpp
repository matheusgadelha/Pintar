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
			setPerspectiveProjection(3.14/3.0f,4.0f/3.0,0.3, 1000.0f);
		}

		void lookAt(
				const Eigen::Vector3f& position, 
				const Eigen::Vector3f& target, 
				const Eigen::Vector3f& up)
		{
			Eigen::Matrix3f R;
			Eigen::Matrix4f viewMatrix;

			R.col(2) = -(target-position).normalized();
			R.col(0) = up.cross(R.col(2)).normalized();
			R.col(1) = R.col(2).cross(R.col(0));

			Eigen::Quaternionf qr(R);
			setRotation( qr );
			setPosition( position );
		}

		void lookAt(
				const Eigen::Vector3f& target,
				const Eigen::Vector3f& up)
		{
			lookAt( getLocalPosition(), target, up );
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

		void setOrthographicProjections( float size, float aspect, float near, float far )
		{
			mNearPlane = near;
			mFarPlane = far;

			float right = (size * aspect)/2.0f;
			float left = -(size * aspect)/2.0f;
			float top = size/2.0f;
			float bottom = -size/2.0f;
			
			Eigen::Matrix4f projectionMatrix = Eigen::Matrix4f::Identity();

			projectionMatrix(0,0) = 2.0f/(right-left);
			projectionMatrix(1,1) = 2.0f/(top-bottom);
			projectionMatrix(2,2) = -2.0f/(far-near);
			projectionMatrix(0,3) = -(right+left)/(right-left);
			projectionMatrix(1,3) = -(top+bottom)/(top-bottom);
			projectionMatrix(2,3) = -(far+near)/(far-near);

			mProjection = projectionMatrix;
		}

		Eigen::Matrix4f view()
		{
			return transform().inverse();
		}

		Eigen::Matrix4f projection()
		{
			return mProjection;
		}

		Eigen::Vector3f cameraToWorld( Eigen::Vector2f p )
		{
			Eigen::Vector4f ps( p[0], p[1], 0.7, 1 );
			Eigen::Matrix4f invProjView = (mProjection*view()).inverse();
			Eigen::Vector4f result4 =  invProjView*ps;

			return Eigen::Vector3f(result4[0], result4[1], result4[2])/result4[3];
		}

//		int closestVertexFromPos( StandardMesh mesh, Eigen::Vector2f p )
//		{
//			return 
//		}

	private:
		Eigen::Matrix4f mView;
		Eigen::Matrix4f mProjection;
		float mNearPlane;
		float mFarPlane;
};

class ArcballCamera : public Camera
{
	public:
		ArcballCamera() : Camera(), sensitivity(4.0f){}

		void move( Eigen::Vector2f x1, Eigen::Vector2f x2 )
		{
			Eigen::Quaternionf qy, qx, q;
			qy = Eigen::AngleAxis<float>(sensitivity*(x1[0]-x2[0]), Eigen::Vector3f(0,1,0));
			qx = Eigen::AngleAxis<float>(sensitivity*(x1[1]-x2[1]), -Eigen::Vector3f(1,0,0));
			Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
			q = qx*qy;
			rotation.topLeftCorner<3,3>() = q.matrix();

			this->setPosition( q.matrix() * mPosition );
			this->lookAt( Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,1,0) );
		}

		float sensitivity;
};

}

#endif

