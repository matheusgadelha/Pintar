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
#include "shaders.hpp"
#include "utils.hpp"
#include "arap.hpp"

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
	camera.setOrthographicProjections(2,4.0f/3.0f,0.2,100);
	Pintar::RigidBody mesh_rb( &mesh );
	Pintar::ARAPDeform arapdef( &mesh );

	Pintar::StandardMesh arrow;
	Pintar::MeshIO<Pintar::MeshFileType::OBJ>::loadMesh( "models/arrow.obj", arrow );

	GL::Window window( 800, 600, "Rigid Body Simulation", GL::WindowStyle::Close );
	GL::Context& gl = window.GetContext();

	gl.Enable( GL::Capability::DepthTest );

	GL::Shader vert( GL::ShaderType::Vertex, vertexShader );
	GL::Shader frag( GL::ShaderType::Fragment, fragmentShader );
	GL::Program program( vert, frag );

	GL::Shader arrowvert( GL::ShaderType::Vertex, arrowVS );
	GL::Shader arrowfrag( GL::ShaderType::Fragment, arrowFS );
	GL::Program arrowProg( arrowvert, arrowfrag );

	GL::VertexBuffer model_vbo( mesh.raw_data, sizeof(float)*mesh.indices.size()*3*3, GL::BufferUsage::DynamicDraw );

	GL::VertexArray model_vao;
	model_vao.BindAttribute( program.GetAttribute( "position" ), 
			model_vbo, GL::Type::Float, 3, 0,0 );
	model_vao.BindAttribute( program.GetAttribute( "normal" ), 
			model_vbo, GL::Type::Float, 3, 0, mesh.indices.size()*3*sizeof(float) );
	model_vao.BindAttribute( program.GetAttribute( "color" ), 
			model_vbo, GL::Type::Float, 3, 0, mesh.indices.size()*3*2*sizeof(float) );

	GL::VertexBuffer arrow_vbo( arrow.raw_data, sizeof(float)*arrow.indices.size()*3*3, GL::BufferUsage::StaticDraw );

	GL::VertexArray arrow_vao;
	arrow_vao.BindAttribute( arrowProg.GetAttribute( "position" ), 
			arrow_vbo, GL::Type::Float, 3, 0,0 );

	GL::VertexArray arrow_vao_normals;
	arrow_vao.BindAttribute( arrowProg.GetAttribute( "normal" ), 
			arrow_vbo, GL::Type::Float, 3, 0, arrow.indices.size()*3*sizeof(float) );

	GL::VertexArray arrow_vao_colors;
//	arrow_vao.BindAttribute( arrowProg.GetAttribute( "color" ), 
//			arrow_vbo, GL::Type::Float, 3, 0, arrow.indices.size()*3*2*sizeof(float) );



	Eigen::Quaternion<float> q;
	q = Eigen::AngleAxis<float>(gl.Time(), Eigen::Vector3f(0,1,0));
	camera.setPosition( Eigen::Vector3f(0,0,2));
	camera.lookAt( Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,1,0));

//	mesh_rb.applyForce( Pintar::Force( Eigen::Vector3f(0,0,2) ) );

	bool mouseDown = false;
	bool mouseMiddleDown = false;
	GL::Event ev;
	Eigen::Vector2f x1, x2;
	Eigen::Vector3f arrOrigin;
	Eigen::Vector3f arrDest;
	bool arrowForce = false;
	while( window.IsOpen() )
	{
		gl.ClearColor( GL::Color(255,255,255,255) );
		while ( window.GetEvent( ev ) )
		{
			//Handling camera movement
			if( ev.Type == GL::Event::MouseDown && ev.Mouse.Button == GL::MouseButton::Right ){ 
				mouseMiddleDown = true; 

				x1[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				x1[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;

			} 

			if( ev.Type == GL::Event::MouseMove && mouseMiddleDown )
			{
				x2[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				x2[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;

				camera.move(x1,x2);
				
				x1 = x2;
			}

			if( ev.Type == GL::Event::MouseUp && 
					ev.Mouse.Button == GL::MouseButton::Right ){
				mouseMiddleDown = false;
			} 

			//Handling forces
			if( ev.Type == GL::Event::MouseDown && ev.Mouse.Button == GL::MouseButton::Left ){ 
				mesh.changeVertexPos(2,Eigen::Vector3f(1,1,0));
			} 

			if( ev.Type == GL::Event::MouseUp && 
					ev.Mouse.Button == GL::MouseButton::Left ){
				mesh.changeVertexPos(2,Eigen::Vector3f(-1,1,0));
			}
			
		}

		model_vbo.Data( mesh.raw_data, sizeof(float)*mesh.indices.size()*3*3,
				GL::BufferUsage::DynamicDraw );

//		mesh_rb.simulateEuler( 0.01f );
		
		gl.Clear();
		drawMesh( mesh, model_vao, camera, program, gl );

		drawArrow(arrow, Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,1,0), 
				GL::Vec3(0,1,0), arrow_vao, camera, arrowProg, gl );
		drawArrow(arrow, Eigen::Vector3f(0,0,0), Eigen::Vector3f(1,0,0), 
				GL::Vec3(1,0,0), arrow_vao, camera, arrowProg, gl );
		drawArrow(arrow, Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,0,1), 
				GL::Vec3(0,0,1), arrow_vao, camera, arrowProg, gl );

		if( arrowForce )
		{
			drawArrow(arrow, arrOrigin, arrDest, 
					GL::Vec3(1.0,1.0,0.3), arrow_vao, camera, arrowProg, gl );
		}

		window.Present();
	}

	return 0;
}
