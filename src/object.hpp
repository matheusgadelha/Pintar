#ifndef OBJECT_H_
#define OBJECT_H_

#include <string>
#undef Success
#include <Eigen/Dense>
#include <Eigen/Geometry>


namespace Pintar
{

	class Object
	{
	public:
		Object( std::string name = "Object_"+std::to_string(nextId) )
		{
			mId = nextId;
			mName = name;
			nextId++;
			mParent = NULL;
			mScale = Eigen::Vector3f(1,1,1);
			setPosition(Eigen::Vector3f(0,0,0));
			setRotation(Eigen::Quaternionf::Identity());
		}

		void setRotation( Eigen::Quaternionf q )
		{
			mOrientation = q;
		//	mTransform = Eigen::Matrix4f::Identity();

		//	mTransform = mTransform * 
		//		Eigen::Affine3f(Eigen::Translation3f(mPosition)).matrix();

		//	Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
		//	rotationMatrix.topLeftCorner<3,3>() = mOrientation.matrix();
//			mTransform = mTransform * rotationMatrix;
		}

		void setPosition( Eigen::Vector3f p )
		{
			mPosition = p-getParentPosition();
//			mTransform = Eigen::Matrix4f::Identity();
//
//			mTransform = mTransform * 
//				Eigen::Affine3f(Eigen::Translation3f(mPosition)).matrix();
//
//			Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
//			rotationMatrix.topLeftCorner<3,3>() = mOrientation.matrix();
//			mTransform = mTransform * rotationMatrix;
		}

		void setLocalPosition( Eigen::Vector3f p )
		{
			mPosition = p;
		}

		void setScale( Eigen::Vector3f s )
		{
			mScale = s;
		}

		Eigen::Matrix4f transform()
		{
			return getParentTransform() * localTransform();
		}

		Eigen::Matrix4f localTransform()
		{
			Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
			scale(0,0) = mScale[0];
			scale(1,1) = mScale[1];
			scale(2,2) = mScale[2];

			Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
			rotation.topLeftCorner<3,3>() = mOrientation.matrix();

			Eigen::Matrix4f translation = 
				Eigen::Affine3f( Eigen::Translation3f(getLocalPosition())).matrix();

			return translation * rotation * scale;
		}

		Eigen::Matrix4f getParentTransform()
		{
			if( mParent == NULL )
				return Eigen::Matrix4f::Identity();
			else return mParent->transform();
		}

		Eigen::Vector3f getParentPosition()
		{
			if( getParent() != NULL ) return getParent()->getPosition();
			else return Eigen::Vector3f(0,0,0);
		}

		Eigen::Quaternionf getParentOrientation()
		{
			if( getParent() != NULL ) return getParent()->getOrientation();
			else return Eigen::Quaternionf::Identity();
		}

		Eigen::Quaternionf getOrientation()
		{
			return getParentOrientation() * mOrientation;
		}

		Eigen::Vector3f getPosition()
		{
			return getParentPosition() + mPosition;
		}

		Eigen::Vector3f getLocalPosition()
		{
			return mPosition;
		}

		void setParent( Object* parent )
		{
			mParent = parent;
		}

		Object* getParent()
		{
			if( mParent != NULL ) return mParent;
			else return NULL;
		}

	protected:
		Object* mParent;

		Eigen::Vector3f mPosition;
		Eigen::Vector3f mScale;
		Eigen::Quaternionf mOrientation;
		std::string mName;
		int mId;
		static int nextId;
};

int Object::nextId = 0;

}

#endif

