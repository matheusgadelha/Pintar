#include <GL/OOGL.hpp>

#include <string>
#include <fstream>
//Trick to make it work on some new Nvidia drivers.
#include <pthread.h>
void junk()
{
	int i;
	i = pthread_getconcurrency();
}

#undef Success
#include <Eigen/Dense>

#include "meshio.hpp"
#include "mesh.hpp"
#include "camera.hpp"
#include "rigidbody.hpp"

std::string vertexShader = GLSL330(
	in vec3 position;
	uniform mat4 viewproj;

	void main()
	{
		gl_Position = viewproj * vec4( position, 1.0 );
//		gl_Position = vec4( position, 1.0 );
	}
);

std::string fragmentShader = GLSL330(
	out vec4 outColor;

	void main()
	{
		outColor = vec4( 1.0, 0.5, 0.0, 1.0 );
	}
);

const std::string sep = "\n-----\n";

int main( int argc, char** argv)
{
	std::string mesh_path;
	if (argc < 2)
	{
		std::cout << "ERROR: Specify the path to a mesh.\n";
		return -1;
	}

	mesh_path = argv[1];
	Pintar::StandardMesh mesh;
	Pintar::MeshIO<Pintar::MeshFileType::OBJ>::loadMesh( mesh_path, mesh );
	Pintar::ArcballCamera camera;
	Pintar::RigidBody mesh_rb( &mesh );
//	camera.setParent( &mesh );

	GL::Window window( 800, 600, "OpenGL Window", GL::WindowStyle::Close );
	GL::Context& gl = window.GetContext();
	gl.Enable( GL::Capability::DepthTest );

	GL::Shader vert( GL::ShaderType::Vertex, vertexShader );
	GL::Shader frag( GL::ShaderType::Fragment, fragmentShader );
	GL::Program program( vert, frag );

	GL::VertexBuffer model_vbo( mesh.raw_vertices, sizeof(float)*mesh.indices.size()*3, GL::BufferUsage::StaticDraw );

	GL::VertexArray model_vao;
	model_vao.BindAttribute( program.GetAttribute( "position" ), model_vbo, GL::Type::Float, 3, 0,0 );

	Eigen::Quaternion<float> q;
	q = Eigen::AngleAxis<float>(gl.Time(), Eigen::Vector3f(0,1,0));
	camera.setPosition( Eigen::Vector3f(0,0,2));
	camera.lookAt( Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,1,0));

	mesh_rb.applyForce( Pintar::Force( Eigen::Vector3f(-2,0,0) ) );

	bool mouseDown = false;
	GL::Event ev;
	Eigen::Vector2f x1, x2;
	while( window.IsOpen() )
	{
		while ( window.GetEvent( ev ) )
		{
			if( ev.Type == GL::Event::MouseDown ){ 
				mouseDown = true; 

				x1[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				x1[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;

			//	std::cout << "Camera point: \n" << x1 << std::endl;
			//	std::cout << "World point: \n" << camera.cameraToWorld(x1) << std::endl;
			//	std::cout << sep << camera.getPosition() << sep << std::endl;
			} else if( ev.Type == GL::Event::MouseUp ){
				mouseDown = false;
			} 

			if( ev.Type == GL::Event::MouseMove && mouseDown )
			{
				x2[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				x2[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;

				camera.move(x1,x2);
				
				x1 = x2;
			}
		}

		q = Eigen::AngleAxis<float>(gl.Time()/10.0f, Eigen::Vector3f(1,0,0));
//		mesh.setPosition( Eigen::Vector3f(0,0,-1)*gl.Time()/5.0f);

		Eigen::Quaternionf mq;
		mq =  Eigen::AngleAxis<float>(gl.Time(), Eigen::Vector3f(0,0,1) );

//		mesh.setRotation( mq );

		mesh_rb.simulateEuler( 0.01f );
		
		gl.Clear();
		program.SetUniform( "viewproj", camera.projection() * camera.view() * mesh.transform());
		gl.DrawArrays( model_vao, GL::Primitive::Triangles, 0, mesh.indices.size() );

		window.Present();
	}

	return 0;
}
