#ifndef UTILS_H
#define UTILS_H

#include <GL/OOGL.hpp>

#include <iostream>
#include <algorithm>
#include <string>
#include <fstream>
#include <cmath>
#undef Success
#include <Eigen/Dense>
#include <Eigen/SVD>

#include "object.hpp"
#include "mesh.hpp"
#include "camera.hpp"

namespace Pintar
{
	Eigen::Matrix4f arrowTransform( Eigen::Vector3f from, Eigen::Vector3f to )
	{
		Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
		float length = (from-to).squaredNorm();
		scale(1,1) = length;

		Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
		float angle = acos(Eigen::Vector3f(0,1,0).dot(to-from)/length);
		Eigen::Vector3f axis = Eigen::Vector3f(0,1,0).cross(to-from).normalized();
		Eigen::Quaternionf q;
		q = Eigen::AngleAxis<float>( angle, axis );
		if( fabs(angle) > 0.00001f) 
			rotation.topLeftCorner<3,3>() = q.matrix();

		Eigen::Matrix4f translation =
			Eigen::Affine3f( Eigen::Translation3f( from+(to-from)*0.5f)).matrix();

		return translation * rotation * scale;
	}

	void drawMesh( 
			StandardMesh& mesh, 
			GL::VertexArray& vao,
			Camera& camera, 
			GL::Program program, 
			GL::Context& gl )
	{
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		gl.UseProgram( program );
		program.SetUniform( "mvp", camera.projection() * camera.view() * mesh.transform());
		program.SetUniform("wireColor",GL::Vec3(1,1,1));
		program.SetUniform( "normalMatrix", 
				(camera.view() * mesh.transform()).inverse().transpose());
		gl.DrawArrays( vao, GL::Primitive::Triangles, 0, mesh.indices.size() );
		
		glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
		gl.UseProgram( program );
		program.SetUniform( "mvp", camera.projection() * camera.view() * mesh.transform());
		program.SetUniform("wireColor",GL::Vec3(0,0,0));
		program.SetUniform( "normalMatrix", 
				(camera.view() * mesh.transform()).inverse().transpose());
		gl.DrawArrays( vao, GL::Primitive::Triangles, 0, mesh.indices.size() );
	}
	
	void drawArrow( 
			StandardMesh& mesh, 
			GL::Vec3 color,
			GL::VertexArray& vao,
			Camera& camera, 
			GL::Program program, 
			GL::Context& gl )
	{
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		gl.UseProgram( program );
		program.SetUniform( "mvp", camera.projection() * camera.view() * mesh.transform());
		program.SetUniform( "Color", color);
		program.SetUniform( "normalMatrix", 
				(camera.view() * mesh.transform()).inverse().transpose());
		gl.DrawArrays( vao, GL::Primitive::Triangles, 0, mesh.indices.size() );
	}

	void drawArrow( 
			StandardMesh& mesh, 
			Eigen::Vector3f from,
			Eigen::Vector3f to,
			GL::Vec3 color,
			GL::VertexArray& vao,
			Camera& camera, 
			GL::Program program, 
			GL::Context& gl )
	{
		glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
		Eigen::Matrix4f transform = arrowTransform( from, to );
		gl.UseProgram( program );
		program.SetUniform( "mvp", camera.projection() * camera.view() * transform);
		program.SetUniform( "Color", color);
		program.SetUniform( "normalMatrix", 
				(camera.view() * transform).inverse().transpose());
		gl.DrawArrays( vao, GL::Primitive::Triangles, 0, mesh.indices.size() );
	}


}

#endif
