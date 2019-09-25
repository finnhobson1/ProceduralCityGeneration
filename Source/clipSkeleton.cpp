#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
//#include "TestModelH.h"
#include "CityModel.h"
#include <stdint.h>
#include <bitset>

using namespace std;
using glm::vec2;
using glm::ivec2;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

SDL_Event event;

#define SCREEN_WIDTH 500
#define SCREEN_HEIGHT 500
#define FULLSCREEN_MODE true

struct Pixel
{
  int x;
  int y;
  float zinv;
  vec4 pos3d;
};

struct Vertex
{
   vec4 position;
   bitset<4> outcode;
};

/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
vector<Triangle> triangles;

float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

vec4 cameraPos( 0, 0.5, -3.001, 1 );
mat4 yRotation = mat4(1.0f);
float yAngle = 0;

vec4 lightPos( 0, -0.5, -0.7, 1.0 );
float intensity = 12.f;
float red = 1.0f;
float green = 1.0f;
float blue = 1.0f;
vec3 lightColor = intensity * vec3( red, green, blue );
vec3 indirectLight = 0.5f * vec3(red, green, blue);

vec3 currentColor;
vec4 currentNormal;

float umax = SCREEN_WIDTH/2;
float vmax = SCREEN_HEIGHT/2;


/* ---------------------------------------------------------------------------- */
/* FUNCTIONS                                                                    */
/* ---------------------------------------------------------------------------- */

bool Update();
void Draw(screen* screen);
void VertexShader( const Vertex& v, ivec2& p );
void UpdateRotation();
void ComputePolygonRows( const vector<Pixel>& vertexPixels,
    vector<Pixel>& leftPixels,
    vector<Pixel>& rightPixels );
void DrawRows( screen* screen, const vector<Pixel>& leftPixels,
    const vector<Pixel> rightPixels );
void PixelShader(screen* screen, const Pixel& p );
void DrawPolygon( screen* screen, const vector<Vertex>& vertices, vector<Pixel>& pixels );
void InterpolatePixel( Pixel a, Pixel b, vector<Pixel>& result );
vec3 DirectLight( const Vertex & v );
vector<Pixel> InterpolateLine( Pixel a, Pixel b );
void UpdateLightColour();
bool CullPolygon( vector<Vertex>& vertices );
void CalculateOutcode(vector<Vertex>& vertices);
vector<Vertex> ClipPolygonTop(vector<Vertex>& vertices);
vector<Vertex> ClipPolygonLeft(vector<Vertex>& vertices);
vector<Vertex> ClipPolygonBottom(vector<Vertex>& vertices);
vector<Vertex> ClipPolygonRight(vector<Vertex>& vertices);

int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  //LoadTestModel(triangles);
  GenerateModel(triangles);

  while ( Update())
    {
      Draw(screen);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  for( int y=0; y<SCREEN_HEIGHT; ++y )
    for( int x=0; x<SCREEN_WIDTH; ++x )
      depthBuffer[y][x] = 0;

  for( uint32_t i=0; i<triangles.size(); ++i ){
    currentColor = triangles[i].color;
    currentNormal = triangles[i].normal;

    vector<Vertex> vertices(3);
    vector<Pixel> vertexPixels(3);

    vertices[0].position = triangles[i].v0;
    vertexPixels[0].pos3d = triangles[i].v0;
    vertices[1].position = triangles[i].v1;
    vertexPixels[1].pos3d = triangles[i].v1;
    vertices[2].position = triangles[i].v2;
    vertexPixels[2].pos3d = triangles[i].v2;

    // CalculateOutcode(vertices);
    vertices = ClipPolygonTop(vertices);
    vertices = ClipPolygonLeft(vertices);
    vertices = ClipPolygonBottom(vertices);
    vertices = ClipPolygonRight(vertices);

    for (int v = 0; v < 3; v++ ) {
      //Rotate and Translate Vertex
      vertices[v].position = yRotation * vertices[v].position;
      vertices[v].position = vertices[v].position - cameraPos;
      vertices[v].position.w = vertices[v].position.z / SCREEN_HEIGHT;
    }

    bool inView = CullPolygon( vertices );

    for (int v = 0; v < 3; v++ ) vertices[v].position.w = 1;
    if (inView) DrawPolygon( screen, vertices, vertexPixels );
  }
}

/*Place updates of parameters here*/
bool Update()
{
  /*static int t = SDL_GetTicks();
  // Compute frame time
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  std::cout << "Render time: " << dt << " ms." << std::endl;*/

  SDL_Event e;
  while(SDL_PollEvent(&e))
  {
    if (e.type == SDL_QUIT) return false;
    else if (e.type == SDL_KEYDOWN)
	  {
	    int key_code = e.key.keysym.sym;
	    switch(key_code)
      {
        case SDLK_UP:
          // Move camera forward
          cameraPos.z += 0.1;
          break;
        case SDLK_DOWN:
          // Move camera backwards
          cameraPos.z -= 0.1;
          break;
        case SDLK_LEFT:
          // Move camera left
          yAngle -= float(M_PI)/30.0f;
          UpdateRotation();
          break;
        case SDLK_RIGHT:
          // Move camera right
          yAngle += float(M_PI)/30.0f;
          UpdateRotation();
          break;
          case SDLK_w:
            // Move light forward
            lightPos.z += 0.1;
            break;
          case SDLK_s:
            // Move light forward
            lightPos.z -= 0.1;
            break;
          case SDLK_a:
            // Move light left
            lightPos.x -= 0.1;
            break;
          case SDLK_d:
            // Move light right
            lightPos.x += 0.1;
            break;
          case SDLK_q:
            // Move light up
            lightPos.y -= 0.1;
            break;
          case SDLK_e:
            // Move light down
            lightPos.y += 0.1;
            break;
          case SDLK_r:
            // Increase light intensity
            intensity += 1.0;
            UpdateLightColour();
            break;
          case SDLK_f:
            //Decrease light intensity
            if (intensity > 0) intensity -= 1.0;
            UpdateLightColour();
            break;
          case SDLK_t:
            // Increase light red value
            if (red < 1) red += 0.05;
            UpdateLightColour();
            break;
          case SDLK_g:
            // Decrease light red value
            if (red > 0) red -= 0.05;
            UpdateLightColour();
            break;
          case SDLK_y:
            // Increase light green value
            if (green < 1) green += 0.05;
            UpdateLightColour();
            break;
          case SDLK_h:
            // Decreae light green value
            if (green > 0) green -= 0.05;
            UpdateLightColour();
            break;
          case SDLK_u:
            // Increase light blue value
            if (blue < 1) blue += 0.05;
            UpdateLightColour();
            break;
          case SDLK_j:
            if (blue > 0) blue -= 0.05;
            UpdateLightColour();
            break;
        case SDLK_ESCAPE:
          // Quit
          return false;
	      }
	    }
    }
  return true;
}


void VertexShader( const Vertex& v, Pixel& p ) {
  vec4 P = v.position;

  //Project points onto image plane
  p.x = (SCREEN_HEIGHT * P.x / P.z) + (SCREEN_WIDTH/2);
  p.y = (SCREEN_HEIGHT * P.y / P.z) + (SCREEN_HEIGHT/2);
  p.zinv = 1.0f / P.z;

  p.pos3d = p.pos3d * p.zinv;
}

vec3 DirectLight( const Pixel& p )
{
  vec4 position = p.pos3d / p.zinv;
  vec4 normal = currentNormal;

  // Vector from intersection point to light source
  vec4 lightDir = vec4(lightPos.x-position.x, lightPos.y-position.y, lightPos.z-position.z, position.w);
  vec4 unitLightDir = glm::normalize(lightDir);

  float projection = glm::dot(unitLightDir, normal);
  //Distance from intersection point to light source
  float radius = glm::length(vec3(lightDir.x, lightDir.y, lightDir.z));

  vec3 D = lightColor * max(projection, 0.0f) / (4.0f * float(M_PI) * radius * radius);
  return D;
}


void InterpolatePixel( Pixel a, Pixel b, vector<Pixel>& result )
{
  int N = result.size();
  vec3 vecA(a.x, a.y, a.zinv);
  vec3 vecB(b.x, b.y, b.zinv);
  vector<vec3> vecResult( N );

  vec3 step = (vecB - vecA) / float(max(N-1,1));
  vec4 posStep = (b.pos3d - a.pos3d) / float(max(N-1,1));

  vec3 current( vecA );
  vec4 currentPos( a.pos3d );
  for( int i=0; i<N; ++i )
  {
    vecResult[i] = current;
    result[i].x = round(vecResult[i].x);
    result[i].y = round(vecResult[i].y);
    result[i].zinv = vecResult[i].z;
    result[i].pos3d = currentPos;
    current += step;
    currentPos += posStep;
  }
}


vector<Pixel> InterpolateLine( Pixel a, Pixel b ) {
  int xDelta = glm::abs( a.x - b.x );
  int yDelta = glm::abs( a.y - b.y );
  int pixels = glm::max( xDelta, yDelta ) + 1;
  vector<Pixel> line(pixels);
  InterpolatePixel(a, b, line);
  return line;
}


// Updates rotation matrix R with new y rotation value
void UpdateRotation()
{
  yRotation[0][0] = cos(yAngle);
  yRotation[0][2] = sin(yAngle);
  yRotation[2][0] = -sin(yAngle);
  yRotation[2][2] = cos(yAngle);
}

void ComputePolygonRows( const vector<Pixel>& vertexPixels,
    vector<Pixel>& leftPixels,
    vector<Pixel>& rightPixels )
{
  vector<Pixel> edgePixels;
  int V = vertexPixels.size();
  for (int i = 0; i < V; i++) {
    int j = (i+1)%V;
    vector<Pixel> line = InterpolateLine( vertexPixels[i], vertexPixels[j] );
    edgePixels.insert(edgePixels.end(), line.begin(), line.end());
  }

  int yMax = -numeric_limits<int>::max();
  int yMin = numeric_limits<int>::max();

  for ( uint32_t i = 0; i < edgePixels.size(); i++) {
    if (edgePixels[i].y > yMax) yMax = edgePixels[i].y;
    if (edgePixels[i].y < yMin) yMin = edgePixels[i].y;
  }

  int ROWS = yMax - yMin + 1;

  leftPixels.resize(ROWS);
  rightPixels.resize(ROWS);

  for ( int i = 0; i < ROWS; i++ ) {
    leftPixels[i].x = numeric_limits<int>::max();
    rightPixels[i].x = -numeric_limits<int>::max();
    leftPixels[i].y = yMin + i;
    rightPixels[i].y = yMin + i;
  }

  for ( uint32_t i = 0; i < edgePixels.size(); i++ ) {
    int row = edgePixels[i].y - yMin;
    if (edgePixels[i].x < leftPixels[row].x) {
      leftPixels[row].x = edgePixels[i].x;
      leftPixels[row].zinv = edgePixels[i].zinv;
      leftPixels[row].pos3d = edgePixels[i].pos3d;
    }
    if (edgePixels[i].x > rightPixels[row].x) {
      rightPixels[row].x = edgePixels[i].x;
      rightPixels[row].zinv = edgePixels[i].zinv;
      rightPixels[row].pos3d = edgePixels[i].pos3d;
    }
  }
}


void DrawRows( screen* screen, const vector<Pixel>& leftPixels,
               const vector<Pixel> rightPixels )
{
  int ROWS = leftPixels.size();
  for ( int i = 0; i < ROWS; i++ ) {
    int pixels = rightPixels[i].x - leftPixels[i].x + 1;
    vector<Pixel> row(pixels);
    InterpolatePixel( leftPixels[i], rightPixels[i], row );
    for ( int j = 0; j < pixels; j++ ) {
      // printf("%lf\n", row[j].zinv);
      PixelShader(screen, row[j]);
      }
    }
  }

void PixelShader(screen* screen, const Pixel& p )
   {
       int x = p.x;
       int y = p.y;
       if( x > 0 && x < SCREEN_WIDTH && y > 0 && y < SCREEN_HEIGHT && p.zinv > depthBuffer[y][x] )
       {
           depthBuffer[y][x] = p.zinv;
           vec3 directLight = DirectLight(p);
           vec3 illumination = currentColor * (directLight + indirectLight);
           PutPixelSDL( screen, x, y, illumination );
       }
}

void DrawPolygon( screen* screen, const vector<Vertex>& vertices, vector<Pixel>& vertexPixels )
{
    int V = vertices.size();

    for (int i = 0; i < V; i++) {
      VertexShader( vertices[i], vertexPixels[i] );
    }

    vector<Pixel> leftPixels;
    vector<Pixel> rightPixels;
    ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
    DrawRows( screen, leftPixels, rightPixels );
}


bool CullPolygon( vector<Vertex>& vertices ) {
  bool inView = false;
  //FIND CORRECT ZMIN!!
  float zmin = -3.001;

  for (uint32_t i = 0; i < vertices.size(); i++ ){
    bool vertexInView = true;
    // float xmax = umax *  vertices[i].position.w;
    // float xmin = -xmax;
    // float ymax = vmax * vertices[i].position.w;
    // float ymin = -ymax;
    // float x = vertices[i].position.x;
    // float y = vertices[i].position.y;
    float z = vertices[i].position.z;
    if ( /*x > xmax || x < xmin || y > ymax || y < ymin ||*/ z < zmin ) vertexInView = false;
    inView = inView || vertexInView;
  }
  return inView;
}

void CalculateOutcode(vector<Vertex>& vertices) {
  for (uint32_t i = 0; i < vertices.size(); i++ ){
    float xmax = umax *  vertices[i].position.w;
    float xmin = -xmax;
    float ymax = vmax * vertices[i].position.w;
    float ymin = -ymax;
    float x = vertices[i].position.x;
    float y = vertices[i].position.y;
    if(y > ymax) vertices[i].outcode[3] = 1;
    if(y < ymin) vertices[i].outcode[2] = 1;
    if(x > xmax) vertices[i].outcode[1] = 1;
    if(x < xmin) vertices[i].outcode[0] = 1;
  }
}

vector<Vertex> ClipPolygonTop(vector<Vertex>& vertices) {
  vector<Vertex> clippedVertices;
  for(int i = 0; i < vertices.size(); i++) {
    int j = (i+1)%3;

    float ymax = vmax * vertices[i].position.w;
    float ix = vertices[i].position.x;
    float iy = vertices[i].position.y;
    float iz = vertices[i].position.z;
    float jx = vertices[j].position.x;
    float jy = vertices[j].position.y;
    float jz = vertices[j].position.z;

    // Case 1: both vertices inside. Second vertex only added to ClippedVertices
    if(iy <= ymax && jy <= ymax) {
      clippedVertices.push_back(vertices[j]);
    }

    // Case 2: first point outide the window, second inside. Point of intersection and the second point added to ClippedVertices
    if (iy >= ymax  && jy <= ymax) {

      float t = (ymax - iy)/jy;

      Vertex intersect1;
      intersect1.position.x = ix + t * jx;
      intersect1.position.y = ymax;
      intersect1.position.z = iz + t * jz;

      clippedVertices.push_back(intersect1);
      clippedVertices.push_back(vertices[j]);
    }

    // // Case 3: When only second point is outside. Only point of intersection with edge is added
    else if (iy <= ymax  && jy >= ymax) {

      float t = (ymax - iy)/jy;

      Vertex intersect2;
      intersect2.position.x = ix + t * jx;
      intersect2.position.y = ymax;
      intersect2.position.z = iz + t * jz;

      clippedVertices.push_back(intersect2);
    }

    // Case 4: Both points outside so neither added to ClippedVertices
  }
  return clippedVertices;
}

vector<Vertex> ClipPolygonLeft(vector<Vertex>& vertices) {
  vector<Vertex> clippedVertices;
  for(int i = 0; i < vertices.size(); i++) {
    int j = (i+1)%3;

    float xmax = umax *  vertices[i].position.w;
    float xmin = -xmax;
    float ymax = vmax * vertices[i].position.w;
    float ix = vertices[i].position.x;
    float iy = vertices[i].position.y;
    float iz = vertices[i].position.z;
    float jx = vertices[j].position.x;
    float jy = vertices[j].position.y;
    float jz = vertices[j].position.z;

    // Case 1: both vertices inside. Second vertex only added to ClippedVertices
    if(ix >= xmin && jy >= xmin) {
      clippedVertices.push_back(vertices[j]);
    }

    // Case 2: first point outide the window, second inside. Point of intersection and the second point added to ClippedVertices
    if (ix <= xmin  && jy >= xmin) {

      float t = (xmin - ix)/jx;

      Vertex intersect1;
      intersect1.position.x = xmin;
      intersect1.position.y = iy + t * jy;
      intersect1.position.z = iz + t * jz;

      clippedVertices.push_back(intersect1);
      clippedVertices.push_back(vertices[j]);
    }

    // // Case 3: When only second point is outside. Only point of intersection with edge is added
    else if (iy <= ymax  && jy >= ymax) {

      float t = (xmin - ix)/jx;

      Vertex intersect2;
      intersect2.position.x = xmin;
      intersect2.position.y = iy + t * jy;
      intersect2.position.z = iz + t * jz;

      clippedVertices.push_back(intersect2);
    }

    // Case 4: Both points outside so neither added to ClippedVertices
  }
  return clippedVertices;
}

vector<Vertex> ClipPolygonBottom(vector<Vertex>& vertices) {
  vector<Vertex> clippedVertices;
  for(int i = 0; i < vertices.size(); i++) {
    int j = (i+1)%3;

    float ymax = vmax * vertices[i].position.w;
    float ymin = -ymax;
    float ix = vertices[i].position.x;
    float iy = vertices[i].position.y;
    float iz = vertices[i].position.z;
    float jx = vertices[j].position.x;
    float jy = vertices[j].position.y;
    float jz = vertices[j].position.z;

    // Case 1: both vertices inside. Second vertex only added to ClippedVertices
    if(iy >= ymin && jy >= ymin) {
      clippedVertices.push_back(vertices[j]);
    }

    // Case 2: first point outide the window, second inside. Point of intersection and the second point added to ClippedVertices
    if (iy <= ymin  && jy >= ymin) {

      float t = (ymin - iy)/jy;

      Vertex intersect1;
      intersect1.position.x = ix + t * jx;;
      intersect1.position.y = ymin;
      intersect1.position.z = iz + t * jz;

      clippedVertices.push_back(intersect1);
      clippedVertices.push_back(vertices[j]);
    }

    // // Case 3: When only second point is outside. Only point of intersection with edge is added
    else if (iy >= ymin  && jy <= ymin) {

      float t = (ymin - iy)/jy;

      Vertex intersect2;
      intersect2.position.x = ix + t * jx;
      intersect2.position.y = ymin;
      intersect2.position.z = iz + t * jz;

      clippedVertices.push_back(intersect2);
    }

    // Case 4: Both points outside so neither added to ClippedVertices
  }
  return clippedVertices;
}

vector<Vertex> ClipPolygonRight(vector<Vertex>& vertices) {
  vector<Vertex> clippedVertices;
  for(int i = 0; i < vertices.size(); i++) {
    int j = (i+1)%3;

    float xmax = umax *  vertices[i].position.w;
    float ix = vertices[i].position.x;
    float iy = vertices[i].position.y;
    float iz = vertices[i].position.z;
    float jx = vertices[j].position.x;
    float jy = vertices[j].position.y;
    float jz = vertices[j].position.z;

    // Case 1: both vertices inside. Second vertex only added to ClippedVertices
    if(ix <= xmax && jx <= xmax) {
      clippedVertices.push_back(vertices[j]);
    }

    // Case 2: first point outide the window, second inside. Point of intersection and the second point added to ClippedVertices
    if (ix >= xmax  && jx <= xmax) {

      float t = (xmax - ix)/jx;

      Vertex intersect1;
      intersect1.position.x = xmax;
      intersect1.position.y = iy + t * jy;
      intersect1.position.z = iz + t * jz;

      clippedVertices.push_back(intersect1);
      clippedVertices.push_back(vertices[j]);
    }

    // // Case 3: When only second point is outside. Only point of intersection with edge is added
    else if (ix <= xmax  && jx >= xmax) {

      float t = (xmax - ix)/jx;

      Vertex intersect2;
      intersect2.position.x = xmax;
      intersect2.position.y = iy + t * jy;
      intersect2.position.z = iz + t * jz;

      clippedVertices.push_back(intersect2);
    }

    // Case 4: Both points outside so neither added to ClippedVertices
  }
  return clippedVertices;
}

void UpdateLightColour()
{
  lightColor = intensity * vec3(red, green, blue);
  indirectLight = intensity/28.0f * vec3(red, green, blue);
}
