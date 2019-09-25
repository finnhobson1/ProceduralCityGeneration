#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "GenerateCity.h"
#include <stdint.h>
#include <omp.h>

using namespace std;
using glm::vec2;
using glm::ivec2;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

SDL_Event event;

#define SCREEN_WIDTH 600
#define SCREEN_HEIGHT 600
#define FULLSCREEN_MODE true

struct Pixel
{
  int x;
  int y;
  int u;
  int v;
  float zinv;
  vec4 pos3d;
};

struct Vertex
{
   vec4 position;
   ivec2 texturePos;
};

/* ----------------------------------------------------------------------------*/
/* GLOBAL VARIABLES                                                            */
vector<Triangle> triangles;

float depthBuffer[SCREEN_HEIGHT][SCREEN_WIDTH];

vec4 cameraPos( 0, 0.6, -2.001, 1 );
mat4 yRotation = mat4(1.0f);
float yAngle = 0;

vec4 lightPos( 0, -2, -1, 1.0 );
float intensity = 15.f;
float red = 1.0f;
float green = 1.0f;
float blue = 1.0f;
vec3 lightColor = intensity * vec3( red, green, blue );
vec3 indirectLight = 0.5f * vec3(red, green, blue);

vec3 currentColor;
vec4 currentNormal;

vector<vec2> stars(200);
vector<Car> cars(2000);
vector<vec4> lights;

int currentCityX = 0;
int currentCityZ = 0;

float maxCameraZ = 0.f;

vec3 buildingTexture[512][512];
SDL_Surface *textureSurface;
bool textureOn = false;

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
bool ClipPolygon( vector<Vertex>& vertices );
void DrawTriangle( screen* screen, Triangle& triangle );
void GenerateCity();
void GenerateStars();
void GenerateTexture();
glm::vec3 GetPixelSDL(SDL_Surface *surface, int x, int y);


int main( int argc, char* argv[] )
{
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );

  GenerateCity();
  GenerateCars(cars, 0, 0);
  GenerateStars();
  GenerateTexture();

  const char file[] = "window.bmp";
  textureSurface = SDL_LoadBMP(file);

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

  vec3 blue = vec3(0, 0, 0.2);

  //Display black to blue sky gradient
  for (int i = 0; i < SCREEN_WIDTH; i++) {
    for (float j = 0; j < SCREEN_WIDTH*0.6f; j++) {
      PutPixelSDL(screen, i, j, blue*j*0.0025f);
    }
  }

  //Display stars
  for ( unsigned int i = 0; i < stars.size(); ++i )
    if (stars[i].x > 0 && stars[i].x < SCREEN_WIDTH && stars[i].y > 0 && stars[i].y < SCREEN_HEIGHT)
      PutPixelSDL(screen, stars[i].x, stars[i].y, vec3(1,1,1));

  //Display cars
  #pragma omp parallel
  {
    #pragma omp for nowait
    for ( unsigned int i = 0; i < cars.size(); ++i )
    {
      vec4 position = yRotation * cars[i].position;
      position = position - cameraPos;
      float uCar = SCREEN_HEIGHT * position.x/position.z + SCREEN_WIDTH/2;
      float vCar = SCREEN_HEIGHT * position.y/position.z + SCREEN_HEIGHT/2;
      vec3 colour = 0.5f * cars[i].colour / position.z;
      if (position.z > 0 && position.z < 4 && uCar > 2 && uCar < SCREEN_WIDTH-2 && vCar > 2 && vCar < SCREEN_HEIGHT-2) {
          PutPixelSDL(screen, uCar, vCar, colour);
          //First ring of glow
          PutPixelSDL(screen, uCar+1, vCar, colour*0.5f);
          PutPixelSDL(screen, uCar, vCar+1, colour*0.5f);
          PutPixelSDL(screen, uCar+1, vCar+1, colour*0.5f);
          PutPixelSDL(screen, uCar-1, vCar, colour*0.5f);
          PutPixelSDL(screen, uCar, vCar-1, colour*0.5f);
          PutPixelSDL(screen, uCar-1, vCar-1, colour*0.5f);
          PutPixelSDL(screen, uCar-1, vCar+1, colour*0.5f);
          PutPixelSDL(screen, uCar+1, vCar-1, colour*0.5f);
          //Second ring of glow
          PutPixelSDL(screen, uCar+2, vCar-2, colour*0.25f);
          PutPixelSDL(screen, uCar+2, vCar-1, colour*0.25f);
          PutPixelSDL(screen, uCar+2, vCar, colour*0.25f);
          PutPixelSDL(screen, uCar+2, vCar+1, colour*0.25f);
          PutPixelSDL(screen, uCar+2, vCar+2, colour*0.25f);
          PutPixelSDL(screen, uCar-2, vCar-2, colour*0.25f);
          PutPixelSDL(screen, uCar-2, vCar-1, colour*0.25f);
          PutPixelSDL(screen, uCar-2, vCar, colour*0.25f);
          PutPixelSDL(screen, uCar-2, vCar+1, colour*0.25f);
          PutPixelSDL(screen, uCar-2, vCar+2, colour*0.25f);
          PutPixelSDL(screen, uCar-1, vCar-2, colour*0.25f);
          PutPixelSDL(screen, uCar, vCar-2, colour*0.25f);
          PutPixelSDL(screen, uCar+1, vCar-2, colour*0.25f);
          PutPixelSDL(screen, uCar-1, vCar+2, colour*0.25f);
          PutPixelSDL(screen, uCar, vCar+2, colour*0.25f);
          PutPixelSDL(screen, uCar+1, vCar+2, colour*0.25f);
        }
    }
  }

  //Display street lights
  #pragma omp parallel
  {
    #pragma omp for nowait
    for ( unsigned int i = 0; i < lights.size(); ++i )
    {
      vec4 position = yRotation * lights[i];
      position = position - cameraPos;
      float uLight = SCREEN_HEIGHT * position.x/position.z + SCREEN_WIDTH/2;
      float vLight = SCREEN_HEIGHT * position.y/position.z + SCREEN_HEIGHT/2;
      vec3 colour = 1.5f * vec3(0, 1, 1) / position.z;
      if (position.z > 0.1 && position.z < 5 && uLight > 1 && uLight < SCREEN_WIDTH-1 && vLight > 1 && vLight < SCREEN_HEIGHT-1) {
        PutPixelSDL(screen, uLight, vLight, colour);
        PutPixelSDL(screen, uLight-1, vLight-1, colour*0.2f);
        PutPixelSDL(screen, uLight, vLight-1, colour*0.2f);
        PutPixelSDL(screen, uLight-1, vLight, colour*0.2f);
        PutPixelSDL(screen, uLight+1, vLight, colour*0.2f);
        PutPixelSDL(screen, uLight, vLight+1, colour*0.2f);
        PutPixelSDL(screen, uLight-1, vLight+1, colour*0.2f);
        PutPixelSDL(screen, uLight+1, vLight-1, colour*0.2f);
        PutPixelSDL(screen, uLight+1, vLight+1, colour*0.2f);
      }
    }
  }

  //Reset depth buffer
  for( int y=0; y<SCREEN_HEIGHT; ++y )
    for( int x=0; x<SCREEN_WIDTH; ++x )
      depthBuffer[y][x] = 0;

  //Draw buildings
  for( uint32_t i=0; i<triangles.size(); ++i ) DrawTriangle( screen, triangles[i] );
}


void DrawTriangle( screen* screen, Triangle& triangle ) {
  currentColor = triangle.color;
  currentNormal = triangle.normal;

  vector<Vertex> vertices(3);
  vector<Pixel> vertexPixels(3);

  vertices[0].position = triangle.v0;
  vertices[1].position = triangle.v1;
  vertices[2].position = triangle.v2;

  vertices[0].texturePos = triangle.t0;
  vertices[1].texturePos = triangle.t1;
  vertices[2].texturePos = triangle.t2;

  vertexPixels[0].pos3d = triangle.v0;
  vertexPixels[1].pos3d = triangle.v1;
  vertexPixels[2].pos3d = triangle.v2;

  for (int v = 0; v < 3; v++ ) {
    //Rotate and Translate Vertex
    vertices[v].position = yRotation * vertices[v].position;
    vertices[v].position = vertices[v].position - cameraPos;
    vertices[v].position.w = vertices[v].position.z / SCREEN_HEIGHT;
  }

  bool inView = ClipPolygon( vertices );

  for (int v = 0; v < 3; v++ ) vertices[v].position.w = 1;
  if (inView) DrawPolygon( screen, vertices, vertexPixels );
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

  //Update car positions
  for ( unsigned int i = 0; i < cars.size(); ++i )
  {
    cars[i].position = cars[i].position + cars[i].movement * 0.005f;
    if (cars[i].position.z < -5) cars[i].position.z += 20;
    //else if (cars[i].position.z > 15) cars[i].position.z -= 20;
    if (cars[i].position.x < -3) cars[i].position.x += 6;
    else if (cars[i].position.x > 3) cars[i].position.x -= 6;
  }

  //Generate new city 2 squares in front of camera
  if (cameraPos.z * 0.5f > currentCityZ) {
    currentCityZ++;
    GenerateModel(triangles, currentCityX+1, currentCityZ+2);
    GenerateModel(triangles, currentCityX, currentCityZ+2);
    GenerateModel(triangles, currentCityX-1, currentCityZ+2);
    GenerateLights(lights, currentCityX+1, currentCityZ+2);
    GenerateLights(lights, currentCityX, currentCityZ+2);
    GenerateLights(lights, currentCityX-1, currentCityZ+2);
    GenerateCars(cars, 0, currentCityZ-2);
  }

  SDL_Event e;
  while(SDL_PollEvent(&e))
  {
    if (e.type == SDL_QUIT) return false;
    else if (e.type == SDL_KEYDOWN)
	  {
	    int key_code = e.key.keysym.sym;
	    switch(key_code)
      {
        case SDLK_SPACE:
          GenerateCity();
          GenerateCars(cars, 0, currentCityZ);
          GenerateStars();
          break;
        case SDLK_RETURN:
          textureOn = !textureOn;
        case SDLK_UP:
          // Move camera forward
          cameraPos.z += 0.1f;
          lightPos.z += 0.1f;
          for ( unsigned int i = 0; i < stars.size(); ++i )
          {
            stars[i].y -= 0.1f;
            if (stars[i].y <= 0) stars[i].y += SCREEN_HEIGHT * 0.6;
          }
          break;
        case SDLK_DOWN:
          // Move camera backwards
          if (cameraPos.z > maxCameraZ - 2) {
            cameraPos.z -= 0.1f;
            lightPos.z -= 0.1f;
            if (cameraPos.z > maxCameraZ) maxCameraZ = cameraPos.z;
            for ( unsigned int i = 0; i < stars.size(); ++i )
            {
              stars[i].y += 0.05f;
              if (stars[i].y <= 0) stars[i].y += SCREEN_HEIGHT * 0.5;
            }
          }
          break;
        case SDLK_LEFT:
          // Move camera left
          if (cameraPos.x > -2) {
            cameraPos.x -= 0.1f;
            lightPos.x -= 0.1f;
            /*if (cameraPos.z < 1) yAngle -= float(M_PI) * 0.02f;
            else yAngle -= float(M_PI) * 0.01f / cameraPos.z;
            UpdateRotation();*/
            for ( unsigned int i = 0; i < stars.size(); ++i )
              stars[i].x += float(M_PI) * 0.5f;
          }
          break;
        case SDLK_RIGHT:
          // Move camera right
          if (cameraPos.x < 2) {
            cameraPos.x += 0.1f;
            lightPos.x += 0.1f;
            /*if (cameraPos.z < 1) yAngle += float(M_PI) * 0.02f;
            else yAngle += float(M_PI) * 0.01f / cameraPos.z;
            UpdateRotation();*/
            for ( unsigned int i = 0; i < stars.size(); ++i )
              stars[i].x -= float(M_PI) * 0.5f;
          }
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
  p.x = (SCREEN_HEIGHT * P.x / P.z) + (SCREEN_WIDTH * 0.5f);
  p.y = (SCREEN_HEIGHT * P.y / P.z) + (SCREEN_HEIGHT * 0.5f);
  p.zinv = 1.0f / P.z;

  p.u = v.texturePos.x;
  p.v = v.texturePos.y;

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
  vec2 textureA(a.u, a.v);
  vec2 textureB(b.u, b.v);
  vector<vec3> vecResult( N );

  vec3 step = (vecB - vecA) / float(max(N-1,1));
  vec4 posStep = (b.pos3d - a.pos3d) / float(max(N-1,1));
  vec2 textureStep = (textureB - textureA) / float(max(N-1,1));

  vec3 current( vecA );
  vec4 currentPos( a.pos3d );
  ivec2 currentTexture( textureA );
  for( int i=0; i<N; ++i )
  {
    vecResult[i] = current;
    result[i].x = round(vecResult[i].x);
    result[i].y = round(vecResult[i].y);
    result[i].zinv = vecResult[i].z;
    result[i].pos3d = currentPos;
    result[i].u = currentTexture.x;
    result[i].v = currentTexture.y;
    current += step;
    currentPos += posStep;
    currentTexture += textureStep;
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
      leftPixels[row].u = edgePixels[i].u;
      leftPixels[row].v = edgePixels[i].v;
    }
    if (edgePixels[i].x > rightPixels[row].x) {
      rightPixels[row].x = edgePixels[i].x;
      rightPixels[row].zinv = edgePixels[i].zinv;
      rightPixels[row].pos3d = edgePixels[i].pos3d;
      rightPixels[row].u = edgePixels[i].u;
      rightPixels[row].v = edgePixels[i].v;
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
    int textRow = row[0].v;
    for ( int j = 0; j < pixels; j++ ) {
      row[j].v = textRow;
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
           //currentColor = buildingTexture[p.u][p.v];
           if (textureOn) currentColor = GetPixelSDL(textureSurface, p.u, p.v) + vec3(0.1, 0.1, 0.1);
           //printf("%d, %d\n", p.u, p.v);
           vec3 illumination = currentColor * (directLight + indirectLight);
           if (p.zinv < 0.7) illumination *= (p.zinv * 1.5f);
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


bool ClipPolygon( vector<Vertex>& vertices ) {
  bool inView = false;
  float umax = SCREEN_WIDTH/2;
  float vmax = SCREEN_HEIGHT/2;
  //FIND CORRECT ZMIN!!
  float zmin = 0.1;
  float zmax = 5;

  for (uint32_t i = 0; i < vertices.size(); i++ ){
    bool vertexInView = true;
    float xmax = umax *  vertices[i].position.w;
    float xmin = -xmax;
    float ymax = vmax * vertices[i].position.w;
    float ymin = -ymax;
    float x = vertices[i].position.x;
    float y = vertices[i].position.y;
    float z = vertices[i].position.z;
    if ( x > xmax || x < xmin || y > ymax || y < ymin || z > zmax || z < zmin ) vertexInView = false;
    inView = inView || vertexInView;
  }
  return inView;
}


void UpdateLightColour()
{
  lightColor = intensity * vec3(red, green, blue);
  indirectLight = intensity/28.0f * vec3(red, green, blue);
}

void GenerateCity() {
  triangles.clear();
  lights.clear();

  for (int i = currentCityX-1; i <= currentCityX+1; i++) {
    for (int j = currentCityZ-2; j <= currentCityZ+2; j++) {
      GenerateModel(triangles, i, j);
      GenerateLights(lights, i, j);
    }
  }
}

void GenerateStars() {
  for ( unsigned int i = 0; i < stars.size(); i++ )
  {
    stars[i].x = (rand() % SCREEN_WIDTH * 6) - SCREEN_WIDTH * 3;
    stars[i].y = rand() % SCREEN_HEIGHT * 0.5;
  }
}

void GenerateTexture() {
  for (int x = 0; x < 64; x++) {
    for (int y = 0; y < 64; y++) {
      double colour = (double) rand() / (RAND_MAX);
      for (int i = 1; i < 7; i++) {
        for (int j = 1; j < 7; j++) {
          buildingTexture[x+i][y+j] = vec3(colour, colour, colour);
        }
      }
    }
  }
}
