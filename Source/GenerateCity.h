#ifndef GENERATE_CITY_H
#define GENERATE_CITY_H

#include <glm/glm.hpp>
#include <vector>

// Used to describe a triangular surface:
class Triangle
{
public:
	glm::vec4 v0;
	glm::vec4 v1;
	glm::vec4 v2;
	glm::vec4 normal;
	glm::vec3 color;
	glm::ivec2 t0;
	glm::ivec2 t1;
	glm::ivec2 t2;

	Triangle( glm::vec4 v0, glm::vec4 v1, glm::vec4 v2, glm::vec3 color )
		: v0(v0), v1(v1), v2(v2), color(color)
	{
		ComputeNormal();
	}

	Triangle( glm::vec4 v0, glm::vec4 v1, glm::vec4 v2, glm::vec3 color, glm::ivec2 t0, glm::ivec2 t1, glm::ivec2 t2 )
		: v0(v0), v1(v1), v2(v2), color(color), t0(t0), t1(t1), t2(t2)
	{
		ComputeNormal();
	}

	void ComputeNormal()
	{
	  glm::vec3 e1 = glm::vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
	  glm::vec3 e2 = glm::vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
	  glm::vec3 normal3 = glm::normalize( glm::cross( e2, e1 ) );
	  normal.x = normal3.x;
	  normal.y = normal3.y;
	  normal.z = normal3.z;
	  normal.w = 1.0;
	}
};

struct Car
{
  glm::vec4 position;
  glm::vec3 colour;
  glm::vec4 movement;
};

void GenerateModel( std::vector<Triangle>& triangles, int x, int y );
void GenerateCars( std::vector<Car>& cars, int cityX, int cityZ );
void GenerateLights( std::vector<glm::vec4>& lights, int cityX, int cityZ );


#endif
