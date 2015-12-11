#ifndef SHADERS_H
#define SHADERS_H

#include <GL/OOGL.hpp>

std::string vertexShader = GLSL330(
	in vec3 position;
	in vec3 normal;
	in vec3 color;
	uniform mat4 mvp;
	uniform mat4 normalMatrix;

	out vec3 Normal;
	out vec3 Color;

	void main()
	{
		gl_Position = mvp * vec4( position, 1.0 );
		Normal = (normalMatrix * vec4(normal,1)).xyz;
		Color = color;
	}
);

std::string fragmentShader = GLSL330(
	in vec3 Normal;
	in vec3 Color;
	out vec4 outColor;

	vec3 ldir = vec3(1.0, -1.0, 0.0);

	float env = 0.2;
	float diff = 0.8;

	void main()
	{
		float diff_shading = diff*max(dot(-ldir,Normal),0);
		vec3 final_color = (diff_shading + env)*Color;
		outColor = vec4(final_color, 1.0);
	}
);

std::string arrowVS = GLSL330(
	in vec3 position;
	in vec3 normal;

	uniform mat4 mvp;
	uniform mat4 normalMatrix;

	out vec3 Normal;

	void main()
	{
		gl_Position = mvp * vec4( position, 1.0 );
		Normal = (normalMatrix * vec4(normal,1)).xyz;
	}
);

std::string arrowFS = GLSL330(
	in vec3 Normal;
	out vec4 outColor;

	vec3 ldir = vec3(1.0, -1.0, 0.0);

	uniform vec3 Color;

	float env = 0.2;
	float diff = 0.8;

	void main()
	{
		float diff_shading = diff*max(dot(-ldir,Normal),0);
		vec3 final_color = (diff_shading + env)*Color;
		outColor = vec4(Color,1);
	}
);

#endif

