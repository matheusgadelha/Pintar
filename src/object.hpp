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
			setPosition(Eigen::Vector3f(0,0,0));
			setRotation(Eigen::Quaternionf::Identity());
		}

		void setRotation( Eigen::Quaternionf q )
		{
			mOrientation = q;
			mTransform = Eigen::Matrix4f::Identity();

			mTransform = mTransform * 
				Eigen::Affine3f(Eigen::Translation3f(mPosition)).matrix();

			Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
			rotationMatrix.topLeftCorner<3,3>() = mOrientation.matrix();
			mTransform = mTransform * rotationMatrix;
		}

		void setPosition( Eigen::Vector3f p )
		{
			mPosition = p-getParentPosition();
			mTransform = Eigen::Matrix4f::Identity();

			mTransform = mTransform * 
				Eigen::Affine3f(Eigen::Translation3f(mPosition)).matrix();

			Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
			rotationMatrix.topLeftCorner<3,3>() = mOrientation.matrix();
			mTransform = mTransform * rotationMatrix;
		}

		void setLocalPosition( Eigen::Vector3f p )
		{
			mPosition = p;
			mTransform = Eigen::Matrix4f::Identity();

			mTransform = mTransform * 
				Eigen::Affine3f(Eigen::Translation3f(mPosition)).matrix();

			Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
			rotationMatrix.topLeftCorner<3,3>() = mOrientation.matrix();
			mTransform = mTransform * rotationMatrix;
		}

		Eigen::Matrix4f localTransform()
		{
			return mTransform;
		}

		Eigen::Matrix4f transform()
		{
			return getParentTransform() * localTransform();
		}

		Eigen::Vector3f getParentPosition()
		{
			return getParentTransform().topRightCorner<3,1>();
		}

		Eigen::Quaternionf getParentRotation()
		{
			return Eigen::Quaternionf( getParentTransform().topLeftCorner<3,3>() );
		}

		Eigen::Vector3f getPosition()
		{
			return mPosition + getParentPosition();
		}

		Eigen::Matrix4f getParentTransform()
		{
			if( mParent == NULL )
				return Eigen::Matrix4f::Identity();
			else return mParent->transform();
		}

		void setParent( Object* parent )
		{
			mParent = parent;
		}

	protected:
		Object* mParent;

		Eigen::Matrix4f mTransform;
		Eigen::Vector3f mPosition;
		Eigen::Quaternionf mOrientation;
		std::string mName;
		int mId;
		static int nextId;
};

int Object::nextId = 0;

}

#endif

