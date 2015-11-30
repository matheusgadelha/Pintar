#ifndef MESHLOADING_HPP_
#define MESHLOADING_HPP_

#include "mesh.hpp"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>

#undef Success
#include <Eigen/Dense>

#include <boost/progress.hpp>

namespace Pintar
{

inline std::vector<std::string> split( 
		const std::string& str, 
		const char delimiter
){
	std::vector<std::string> internal;
	std::stringstream ss(str);
	std::string tok;

	while(getline(ss, tok, delimiter)) {
		internal.push_back(tok);
	}
	return internal;
}

inline unsigned int numOfLines( std::string path )
{
	std::ifstream fs;
	fs.open (path, std::ifstream::in);
	std::string line;
	int numLines = 0;
	while( std::getline( fs, line ))
		++numLines;
	fs.close();
	return numLines;
}

enum MeshFileType
{
	OBJ, OFF
};

template<MeshFileType FileType>
struct MeshIO
{
	
	static int loadMesh (
			std::string, 
			std::vector<Eigen::Vector3f>&, 
			std::vector<int>&
	)
	{
		std::cout << "Not implemented.\n";
		return -1;
	}
	
	static int loadMesh (std::string)
	{
		std::cout << "Not implemented.\n";
		return -1;
	}
};

template<>
struct MeshIO<MeshFileType::OBJ>
{
	static int loadMesh(
			std::string path, 
			std::vector<Eigen::Vector3f>& vertices, 
			std::vector<int>& indices
	)
	{
		std::ifstream fs;
		unsigned int lineNum = numOfLines( path );
		fs.open (path, std::ifstream::in);
		if(!fs)
		{
			std::cout << "Mesh file \"" << path << "\" could not be loaded." << std::endl;
			return -1;
		}
		std::vector<Eigen::Vector3f> single_normals;
		std::vector<int> normal_indices;
		std::vector<int> uvs_indices;

		std::cout << "Loading " << path << "...";
		boost::progress_display show_progress( lineNum );

		while( !fs.eof() )
		{
			std::string type;
			fs >> type;
			if (type[0] == '#')
			{
				std::getline(fs,type);;
			}
			if (type == "v")
			{
				Eigen::Vector3f new_vertex;
				fs >> new_vertex[0];
				fs >> new_vertex[1];
				fs >> new_vertex[2];
				vertices.push_back (new_vertex);
			}
			if (type == "vn")
			{
				Eigen::Vector3f new_normal;
				fs >> new_normal[0];
				fs >> new_normal[1];
				fs >> new_normal[2];
				single_normals.push_back (new_normal);
			}
			if (type == "f")
			{
				std::string vertex_desc;
				for (int i=0; i<3; ++i)
				{
					fs >> vertex_desc;
					std::vector<std::string> vert_split;
					vert_split = split (vertex_desc,'/');
					indices.push_back(std::stoi(vert_split[0])-1);
				}
			}
			else std::getline(fs,type);;
			++show_progress;
		}
		std::cout << "Done." << std::endl;
		fs.close();

		return 0;
	}

	static int loadMesh (std::string path, StandardMesh& mesh)
	{
		std::vector<Eigen::Vector3f> raw_vertices;
		std::vector<int> indices;
		if (loadMesh (path, raw_vertices, indices) < 0)
			return -1;

		mesh.generateMesh (raw_vertices, indices);

		return 0;
	}

	static int writeMesh (std::string path, StandardMesh& mesh)
	{
		std::ofstream fs;
		fs.open (path, std::ofstream::out);

		for (size_t i=0; i < mesh.vertices.size(); i=i+3)
		{
			fs << "v " << mesh.vertices[i] << std::endl;
		}

		for (size_t i=0; i < mesh.indices.size(); i=i+3)
		{
			fs << "f ";
			fs << mesh.indices[i] << " ";
			fs << mesh.indices[i+1] << " ";
			fs << mesh.indices[i+2] << " ";
		}
		fs.close();

		return 0;
	}
};

}

#endif
