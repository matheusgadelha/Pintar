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
	if (argc < 3)
	{
		std::cout << "ERROR: Specify the path to a mesh and a deformation method (laplacian or arap).\n";
		std::cout << "Example: ./bin/arapdemo models/cactus_small.obj arap." << std::endl;

		return -1;
	}

	bool estimateRotation = true;
	if( strcmp(argv[2],"laplacian") == 0 ) estimateRotation = false;
	if( strcmp(argv[2],"arap") == 0 ) estimateRotation = true;

	mesh_path = argv[1];
	Pintar::StandardMesh mesh;
	Pintar::MeshIO<Pintar::MeshFileType::OBJ>::loadMesh( mesh_path, mesh );

	Pintar::ArcballCamera camera;
	camera.setOrthographicProjections(2,4.0f/3.0f,0.2,100);
	Pintar::RigidBody mesh_rb( &mesh );
	Pintar::ARAPDeform arapdef( &mesh );

	Pintar::StandardMesh arrow;
	Pintar::MeshIO<Pintar::MeshFileType::OBJ>::loadMesh( "models/arrow.obj", arrow );
	Pintar::StandardMesh sphere;
	Pintar::MeshIO<Pintar::MeshFileType::OBJ>::loadMesh( "models/sphere.obj", sphere );

	GL::Window window( 800, 600, "ARAP Mesh Deformation", GL::WindowStyle::Close );
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


	GL::VertexBuffer sphere_vbo( sphere.raw_data, sizeof(float)*sphere.indices.size()*3*3, GL::BufferUsage::StaticDraw );

	GL::VertexArray sphere_vao;
	sphere_vao.BindAttribute( arrowProg.GetAttribute( "position" ), 
			sphere_vbo, GL::Type::Float, 3, 0,0 );
	GL::VertexArray sphere_vao_normals;
	sphere_vao.BindAttribute( arrowProg.GetAttribute( "normal" ), 
			sphere_vbo, GL::Type::Float, 3, 0, sphere.indices.size()*3*sizeof(float) );


	camera.setPosition( Eigen::Vector3f(0,0,2));
	camera.lookAt( Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,1,0));

	bool mouseLeftDown = false;
	bool mouseMiddleDown = false;
	bool mouseRightDown = false;
	bool shiftDown = false;
	bool ctrlDown = false;
	Eigen::Vector2f x1, x2;
	Eigen::Vector2f upclick;
	Eigen::Vector2f mc1, mc2;
	Eigen::Vector3f arrOrigin;
	Eigen::Vector3f arrDest;

	GL::Event ev;

	std::cout << std::endl << std::endl << "---" << std::endl << "Instructions:" << std::endl
		<< "-> Use the middle mouse button to rotate the camera and the scroll to zoom in/out" 
		<< std::endl
		<< "-> Use the right mouse button to insert fixed points (red vertices)" << std::endl
		<< "-> Use the right mouse button while holding ctrl to insert control points (green)"
		<< std::endl
		<< "-> Use any of the commands before while holding shift to remove fixed or control points"
		<<"(dont forget to also press ctrl to remove the later!)" << std::endl
		<< "-> Move the mouse while pressing the left mouse button to apply deformations" 
		<< std::endl
		<<"->Press ESC to clear all points" << std::endl
		<< "Have fun!!!" << std::endl;

	float screenSize = 2.0f;
	while( window.IsOpen() )
	{
		gl.ClearColor( GL::Color(255,255,255,255) );
		while ( window.GetEvent( ev ) )
		{
			//Handling camera movement
			if( ev.Type == GL::Event::MouseDown && ev.Mouse.Button == GL::MouseButton::Middle ){ 
				mouseMiddleDown = true; 

				x1[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				x1[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;
			} 

			if( ev.Type == GL::Event::MouseUp && 
					ev.Mouse.Button == GL::MouseButton::Middle ){
				mouseMiddleDown = false;
			} 

			if( ev.Type == GL::Event::MouseMove && mouseMiddleDown )
			{
				x2[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				x2[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;
				camera.move(x1,x2);
				x1 = x2;
			}

			//Zoom
			if( ev.Type == GL::Event::MouseWheel )
			{
				screenSize -= 0.05f*ev.Mouse.Delta;
				if( screenSize < 0.0f ) screenSize += 0.05f*ev.Mouse.Delta;
				camera.setOrthographicProjections( screenSize, 4.0f/3.0f, 0.2f, 1000.0f );
			}

			//Handling Mouse Left Button
			if( ev.Type == GL::Event::MouseDown && ev.Mouse.Button == GL::MouseButton::Left ){ 
				mouseLeftDown = true;
				mc1[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				mc1[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;
			} 

			if( ev.Type == GL::Event::MouseUp && 
					ev.Mouse.Button == GL::MouseButton::Left ){
				mouseLeftDown = false;
			}

			//Handling Mouse Right Button
			if( ev.Type == GL::Event::MouseDown && ev.Mouse.Button == GL::MouseButton::Right ){ 
				mouseRightDown = true;
			} 

			if( ev.Type == GL::Event::MouseUp && 
					ev.Mouse.Button == GL::MouseButton::Right ){
				mouseRightDown = false;
				arapdef.preProcess();
			}

			//Handling Shift
			if( ev.Type == GL::Event::KeyDown && ev.Key.Code == GL::Key::Shift )
				shiftDown = true;

			if( ev.Type == GL::Event::KeyUp && ev.Key.Code == GL::Key::Shift )
				shiftDown = false;

			//Handling Ctrl
			if( ev.Type == GL::Event::KeyDown && ev.Key.Code == GL::Key::Control )
				ctrlDown = true;

			if( ev.Type == GL::Event::KeyUp && ev.Key.Code == GL::Key::Control )
				ctrlDown = false;

			//Clear editing handles
			if( ev.Type == GL::Event::KeyUp && ev.Key.Code == GL::Key::Escape )
			{
				arapdef.laplacianComputed = false;
				arapdef.updatedVertices = false;
				arapdef.clearFixedVertices();
				arapdef.clearControlVertices();
			}

			//Inserting and Removing fixed points
			if( mouseRightDown && !ctrlDown )
			{
				upclick[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				upclick[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;

				int vid = camera.selectVertex( upclick, mesh );
				if( vid >= 0 )
				{
					arapdef.setFixedVertex(vid);
				}
			}

			if( mouseRightDown && shiftDown && !ctrlDown)
			{
				upclick[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				upclick[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;

				int vid = camera.selectVertex( upclick, mesh );
				if( vid >= 0 ) arapdef.unsetFixedVertex(vid);
			}

			//Inserting and Removing control points
			if( mouseRightDown && ctrlDown )
			{
				upclick[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				upclick[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;

				int vid = camera.selectVertex( upclick, mesh );
				if( vid >= 0 ) arapdef.setControlVertex(vid);
			}

			if( mouseRightDown && ctrlDown && shiftDown)
			{
				upclick[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				upclick[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;

				int vid = camera.selectVertex( upclick, mesh );
				if( vid >= 0 ) arapdef.unsetControlVertex(vid);
			}

			if( ev.Type == GL::Event::MouseMove && mouseLeftDown )
			{
				mc2[0] = 2*(ev.Mouse.X/(float)window.GetWidth())-1.0f;
				mc2[1] = 2*(( window.GetHeight()-ev.Mouse.Y )/
						(float)window.GetHeight())-1.0f;
				arapdef.translateControlVertices(
						(camera.cameraToWorld(mc2)-camera.cameraToWorld(mc1))
				);
				mc1 = mc2;
			}
		}

		model_vbo.Data( mesh.raw_data, sizeof(float)*mesh.indices.size()*3*3,
				GL::BufferUsage::DynamicDraw );

		if(arapdef.laplacianComputed && arapdef.updatedVertices)
//		if(arapdef.laplacianComputed)
		{
			arapdef.applyDeformation();
			arapdef.computeRotations();

			arapdef.iter++;
			if( arapdef.iter == arapdef.maxIter )
			{
				arapdef.iter = 0;
				arapdef.updatedVertices = false;
//				arapdef.clearData();
			}
		}
		
		gl.Clear();
		drawMesh( mesh, model_vao, camera, program, gl );

		for( size_t i=0; i<arapdef.isFixedVertex.size(); ++i )
		{
			if( arapdef.isFixedVertex[i] )
			{
				drawSphereOnVertex( sphere, mesh, i, 0.01, GL::Vec3(0.8,0,0),
						sphere_vao, camera, arrowProg, gl );
			}
		}

		for( size_t i=0; i<arapdef.isControlVertex.size(); ++i )
		{
			if( arapdef.isControlVertex[i] )
			{
				drawSphereOnVertex( sphere, mesh, i, 0.01, GL::Vec3(0,0.8,0),
						sphere_vao, camera, arrowProg, gl );
			}
		}

//		drawArrow(arrow, Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,1,0), 
//				GL::Vec3(0,1,0), arrow_vao, camera, arrowProg, gl );
//		drawArrow(arrow, Eigen::Vector3f(0,0,0), Eigen::Vector3f(1,0,0), 
//				GL::Vec3(1,0,0), arrow_vao, camera, arrowProg, gl );
//		drawArrow(arrow, Eigen::Vector3f(0,0,0), Eigen::Vector3f(0,0,1), 
//				GL::Vec3(0,0,1), arrow_vao, camera, arrowProg, gl );

		window.Present();
	}

	return 0;
}
