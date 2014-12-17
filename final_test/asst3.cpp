////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////
//	These skeleton codes are later altered by Ming Jin,
//	for "CS6533: Interactive Computer Graphics", 
//	taught by Prof. Andy Nealen at NYU
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
// #if __GNUG__
// #   include <tr1/memory>
// #endif

#include <GL/glew.h>
#ifdef __MAC__
#   include <GLUT/glut.h>
#else
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"
#include "arcball.h"
#include "quat.h"
#include "rigtform.h"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff
// using namespace tr1; // for shared_ptr

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.3.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// are using. In particular, on Mac OS X currently there is no way of using
// OpenGL 3.x with GLSL 1.3 when GLUT is used.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
static const bool g_Gl2Compatible = true;


static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;
// ========================================
// TODO: you can add global variables here
// ========================================
static int g_currentView = 0;                 // 0 is sky view, 1 is cube1 view, 2 is cube2 view
static int g_currentManipulatingObject = 0;   // 0 is sky, 1 is cube 1, 2 is cube2 
static int g_currentSkyView=0;                // 0 is world-sky view, 1 is sky-sky view



struct ShaderState {
  GlProgram program;

  // Handles to uniform variables
  GLint h_uLight, h_uLight2;
  GLint h_uProjMatrix;
  GLint h_uModelViewMatrix;
  GLint h_uNormalMatrix;
  GLint h_uColor;

  // Handles to vertex attributes
  GLint h_aPosition;
  GLint h_aNormal;

  ShaderState(const char* vsfn, const char* fsfn) {
    readAndCompileShader(program, vsfn, fsfn);

    const GLuint h = program; // short hand

    // Retrieve handles to uniform variables
    h_uLight = safe_glGetUniformLocation(h, "uLight");
    h_uLight2 = safe_glGetUniformLocation(h, "uLight2");
    h_uProjMatrix = safe_glGetUniformLocation(h, "uProjMatrix");
    h_uModelViewMatrix = safe_glGetUniformLocation(h, "uModelViewMatrix");
    h_uNormalMatrix = safe_glGetUniformLocation(h, "uNormalMatrix");
    h_uColor = safe_glGetUniformLocation(h, "uColor");

    // Retrieve handles to vertex attributes
    h_aPosition = safe_glGetAttribLocation(h, "aPosition");
    h_aNormal = safe_glGetAttribLocation(h, "aNormal");

    if (!g_Gl2Compatible)
      glBindFragDataLocation(h, 0, "fragColor");
    checkGlErrors();
  }

};

static const int g_numShaders = 2;
static const char * const g_shaderFiles[g_numShaders][2] = {
  {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"}
};
static const char * const g_shaderFilesGl2[g_numShaders][2] = {
  {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"}
};
static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

// --------- Geometry

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// A vertex with floating point position and normal
struct VertexPN {
  Cvec3f p, n;

  VertexPN() {}
  VertexPN(float x, float y, float z,
           float nx, float ny, float nz)
    : p(x,y,z), n(nx, ny, nz)
  {}

  // Define copy constructor and assignment operator from GenericVertex so we can
  // use make* functions from geometrymaker.h
  VertexPN(const GenericVertex& v) {
    *this = v;
  }

  VertexPN& operator = (const GenericVertex& v) {
    p = v.pos;
    n = v.normal;
    return *this;
  }
};

struct Geometry {
  GlBufferObject vbo, ibo;
  int vboLen, iboLen;

  Geometry(VertexPN *vtx, unsigned short *idx, int vboLen, int iboLen) {
    this->vboLen = vboLen;
    this->iboLen = iboLen;

    // Now create the VBO and IBO
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
  }

  void draw(const ShaderState& curSS) {
    // Enable the attributes used by our shader

    safe_glEnableVertexAttribArray(curSS.h_aPosition);
    safe_glEnableVertexAttribArray(curSS.h_aNormal);

    // bind vbo
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p));

    safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

        // bind ibo
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

    // draw!
    glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);

    // Disable the attributes used by our shader
    safe_glDisableVertexAttribArray(curSS.h_aPosition);
    safe_glDisableVertexAttribArray(curSS.h_aNormal);

  }
};


// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_cube2, g_sphere;

// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
static RigTForm g_skyRbt = RigTForm(Cvec3(0.0, 0.25, 4.0));
// ============================================
// TODO: add a second cube's 
// 1. transformation
// 2. color
// ============================================
static RigTForm g_objectRbt[2] = {RigTForm(Cvec3(-1,0,0)), RigTForm(Cvec3(1,0,0))}; 
static Cvec3f g_objectColors[2] = {Cvec3f(1, 0, 0), Cvec3f(0, 0, 1)};

static const Cvec3f g_arcballColor = Cvec3f(0, 0.47, 1);
static double g_arcballScreenRadius = 1.0;
static double g_arcballScale = 1.0;

static RigTForm g_auxFrame = linFact(g_skyRbt);  // g_auxFrame is the auxiliary frame for manipulation

///////////////// END OF G L O B A L S //////////////////////////////////////////////////




static void initGround() {
  // A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
  VertexPN vtx[4] = {
    VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
    VertexPN(-g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
  };
  unsigned short idx[] = {0, 1, 2, 0, 2, 3};
  g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube geometry
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
  
  vector<VertexPN> vtx_2(vbLen);
  vector<unsigned short> idx_2(ibLen);

  makeCube(1, vtx_2.begin(), idx_2.begin());
  g_cube2.reset(new Geometry(&vtx_2[0], &idx_2[0], vbLen, ibLen));  

}

static void initSpheres() {  
  const int slices = 25;
  const int stacks = 25;

  int ibLen, vbLen;
  getSphereVbIbLen(slices, stacks, vbLen, ibLen);

  /* Temporary storage for sphere geometry */
  vector<VertexPN> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeSphere(1, slices, stacks, vtx.begin(), idx.begin());
  g_sphere.reset(new Geometry(&vtx[0], &idx[0], vbLen, ibLen));
}

static bool nonEgoCubeManipulation() {
  return g_currentManipulatingObject != 0 && g_currentView != g_currentManipulatingObject;
}

static bool useArcball() {
  return (g_currentManipulatingObject == 0 && g_currentSkyView == 0) || nonEgoCubeManipulation();
}


static bool worldSkyManipulation() {
  return g_currentManipulatingObject == 0 && g_currentView == 0 && g_currentSkyView == 0;
}


// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
  GLfloat glmatrix[16];
  projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
  safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
}

// takes MVM and its normal matrix to the shaders
static void sendModelViewNormalMatrix(const ShaderState& curSS, const Matrix4& MVM, const Matrix4& NMVM) {
  GLfloat glmatrix[16];
  MVM.writeToColumnMajorMatrix(glmatrix); // send MVM
  safe_glUniformMatrix4fv(curSS.h_uModelViewMatrix, glmatrix);

  NMVM.writeToColumnMajorMatrix(glmatrix); // send NMVM
  safe_glUniformMatrix4fv(curSS.h_uNormalMatrix, glmatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

//set auxFrame for transformation
static void setAFrame(){
   if (g_currentManipulatingObject == 0) { 
      if (g_currentSkyView == 0){
        g_auxFrame = linFact(g_skyRbt); 
      }
      else{
        g_auxFrame = g_skyRbt;
      }

  }
   else {
    if (g_currentView == 0) {
      g_auxFrame = transFact(g_objectRbt[g_currentManipulatingObject - 1]) * linFact(g_skyRbt);
    } else {
      g_auxFrame = transFact(g_objectRbt[g_currentManipulatingObject - 1]) * linFact(g_objectRbt[g_currentView - 1]);
    }
  }

}

static void drawStuff() {
  setAFrame();
  // short hand for current shader state
  const ShaderState& curSS = *g_shaderStates[g_activeShader];

  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(curSS, projmat);

  const RigTForm eyeRbt = (g_currentView == 0) ? g_skyRbt : g_objectRbt[g_currentView - 1];
  
  const RigTForm invEyeRbt = inv(eyeRbt);

  const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
  const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
  safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
  safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);


  // draw ground
  // ===========
  //
  const RigTForm groundRbt = RigTForm();  // identity
  Matrix4 MVM = rigTFormToMatrix(invEyeRbt * groundRbt);
  Matrix4 NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, 0.1, 0.95, 0.1); // set color
  g_ground->draw(curSS);

  // draw cubes
  // ==========
  MVM = rigTFormToMatrix(invEyeRbt * g_objectRbt[0]);                     //!!model view matrix
  NMVM = normalMatrix(MVM);                                       //!!normal ~
  sendModelViewNormalMatrix(curSS, MVM, NMVM); 
  safe_glUniform3f(curSS.h_uColor, g_objectColors[0][0], g_objectColors[0][1], g_objectColors[0][2]);
  g_cube->draw(curSS);

  MVM = rigTFormToMatrix(invEyeRbt * g_objectRbt[1]);
  NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, g_objectColors[1][0], g_objectColors[1][1], g_objectColors[1][2]);
  g_cube2->draw(curSS);


  RigTForm sphere;    

  if (g_currentManipulatingObject == 0) {
    if (g_currentSkyView == 0) {
      sphere = inv(RigTForm());
    } else {
      sphere = eyeRbt;
    }
  } else {
    sphere = g_objectRbt[g_currentManipulatingObject - 1];
  }
        
          
  if (!g_mouseMClickButton && !(g_mouseLClickButton && g_mouseRClickButton) && useArcball()) {
    g_arcballScale = getScreenToEyeScale((inv(eyeRbt) * sphere).getTranslation()[2], g_frustFovY, g_windowHeight);
  }

  /* draw wireframes */
  glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

  const Matrix4 scale = Matrix4::makeScale(g_arcballScale * g_arcballScreenRadius);
  MVM = rigTFormToMatrix(invEyeRbt * sphere) * scale;
  NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, g_arcballColor[0], g_arcballColor[1], g_arcballColor[2]);
  g_sphere->draw(curSS);
  /* draw filled */
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // draw filled again

}


static void display() {
  glUseProgram(g_shaderStates[g_activeShader]->program);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

  drawStuff();

  glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

  checkGlErrors();
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;

  g_arcballScreenRadius = 0.3 * min(g_windowWidth, g_windowHeight);

  updateFrustFovY();
  glutPostRedisplay();
}

static RigTForm getArcballRotation(const int x, const int y) {
  const RigTForm eyeRbt = (g_currentView == 0) ? g_skyRbt : g_objectRbt[g_currentView - 1];
  const RigTForm object = (g_currentManipulatingObject == 0) ? g_skyRbt : g_objectRbt[g_currentManipulatingObject - 1];

  const bool world_sky_manipulation = worldSkyManipulation();

  Cvec2 sphereOnScreenCoords;
  if (world_sky_manipulation) {
    sphereOnScreenCoords = Cvec2((g_windowWidth - 1) / 2.0, (g_windowHeight - 1) / 2.0);
  } else {
    sphereOnScreenCoords = getScreenSpaceCoord(
      (inv(eyeRbt) * object).getTranslation(),
      makeProjectionMatrix(),
      g_frustNear,
      g_frustFovY,
      g_windowWidth,
      g_windowHeight
    );
  }
  
  const Cvec3 sphere_center = Cvec3(sphereOnScreenCoords, 0);
  const Cvec3 p1 = Cvec3(g_mouseClickX, g_mouseClickY, 0) - sphere_center;
  const Cvec3 p2 = Cvec3(x, y, 0) - sphere_center;

  const Cvec3 v1 = normalize(Cvec3(p1[0], p1[1],
    sqrt(max(0.0, pow(g_arcballScreenRadius, 2) - pow(p1[0], 2) - pow(p1[1], 2)))));
  const Cvec3 v2 = normalize(Cvec3(p2[0], p2[1],
    sqrt(max(0.0, pow(g_arcballScreenRadius, 2) - pow(p2[0], 2) - pow(p2[1], 2)))));

  if (world_sky_manipulation) {
    return RigTForm(Quat(0, v1 * -1.0) * Quat(0, v2));
  } else {
    return RigTForm(Quat(0, v2) * Quat(0, v1 * -1.0));
  }
}


static void motion(const int x, const int y) {    
   if (g_currentView != 0 && g_currentManipulatingObject == 0) return;

  const double curr_x = x;
  const double curr_y = g_windowHeight - y - 1;
  const double raw_dx = curr_x - g_mouseClickX;
  const double raw_dy = curr_y - g_mouseClickY;

  double dx_t, dx_r, dy_t, dy_r;
  if (nonEgoCubeManipulation()) {
    dx_t = raw_dx; dx_r = raw_dx;
    dy_t = raw_dy; dy_r = raw_dy;
  } else if (worldSkyManipulation()) {
    dx_t = -raw_dx; dx_r = -raw_dx;
    dy_t = -raw_dy; dy_r = -raw_dy;
  } else {
    dx_t = raw_dx; dx_r = -raw_dx;
    dy_t = raw_dy; dy_r = -raw_dy;
  }


  const bool use_arcball = useArcball();  
  
  double translateFactor;
  if (use_arcball) {
    translateFactor = g_arcballScale;
  } else {
    translateFactor = 0.01;
  }

  setAFrame();

  RigTForm m;

  if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
    if (use_arcball)
      m = getArcballRotation(curr_x, curr_y);
    else
      m = RigTForm(Quat::makeXRotation(-dy_r) * Quat::makeYRotation(dx_r));
  }
  else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
    m = RigTForm(Cvec3(dx_t, dy_t, 0) * translateFactor);
  }
  else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
    m = RigTForm(Cvec3(0, 0, -dy_t) * translateFactor);
  }
  
  m = g_auxFrame * m * inv(g_auxFrame);

  if (g_mouseClickDown) {

    if (g_currentManipulatingObject == 0) {
      g_skyRbt = m * g_skyRbt;
    } else {
      g_objectRbt[g_currentManipulatingObject - 1] = m * g_objectRbt[g_currentManipulatingObject - 1];
    }
    glutPostRedisplay(); // we always redraw if we changed the scene
  }


  g_mouseClickX = curr_x;
  g_mouseClickY = curr_y;

}

static void reset()
{
	// =========================================================
	// TODO:
	// - reset g_skyRbt and g_objectRbt to their default values
	// - reset the views and manipulation mode to default
	// - reset sky camera mode to use the "world-sky" frame
	// =========================================================
  // g_skyRbt = Matrix4::makeTranslation(Cvec3(0.0, 0.25, 4.0));
  // g_objectRbt[0] = Matrix4::makeTranslation(Cvec3(-1,0,0)); 
  // g_objectRbt[1] = Matrix4::makeTranslation(Cvec3(1,0,0)); 
  // g_currentView = 0;
  // g_currentManipulatingObject = 0;
  // g_currentSkyView = 0;

	cout << "reset all to defaults" << endl;
}

static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;
}

//set the sky view whether world-sky or sky-sky
static void setCurrentSkyView() {
  if (g_currentManipulatingObject == 0 && g_currentView == 0) {
    g_currentSkyView++;
    if(g_currentSkyView==2)g_currentSkyView=0;
    if (g_currentSkyView == 0) {
      cout << "set to world-sky view" << endl;
    } else {
      cout << "set to sky-sky view" << endl;
    }
  } else {
    cout << "use sky view to enable this function" << endl;
  }
}

//set the current view
static void setCurrentView() {
  g_currentView++;
  if (g_currentView == 3) g_currentView=0;
  if (g_currentView == 0) {
    cout << "Current view is sky view" << endl;
  } else {
    cout << "Current view is cube" << g_currentView << " view" << endl;
  }
}

//set the current manipulating object
static void setCurrentManipulatingObject(){
  g_currentManipulatingObject++;
  if (g_currentManipulatingObject == 3) g_currentManipulatingObject=0;
  if (g_currentManipulatingObject == 0) {
    cout << "Current manipulating object is sky" << endl;
  } else {
    cout << "Current manipulating object is cube" << g_currentManipulatingObject << endl;
  }
}



static void keyboard(const unsigned char key, const int x, const int y) {
  switch (key) {
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "f\t\tToggle flat shading on/off.\n"
    << "o\t\tCycle object to edit\n"
    << "v\t\tCycle view\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n" << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'f':
    g_activeShader ^= 1;
    break;
  case 'v':
    setCurrentView();
    break;
  case 'o':
    setCurrentManipulatingObject();
    break;
  case 'm':
    setCurrentSkyView();
  break;
  case 'r':
    reset();
  break;


  // ============================================================
  // TODO: add the following functionality for 
  //       keybaord inputs
  // - 'v': cycle through the 3 views
  // - 'o': cycle through the 3 objects being manipulated
  // - 'm': switch between "world-sky" frame and "sky-sky" frame
  // - 'r': reset the scene
  // ============================================================
  }
  glutPostRedisplay();
}


static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 3");                       // title the window

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initShaders() {
  g_shaderStates.resize(g_numShaders);
  for (int i = 0; i < g_numShaders; ++i) {
    if (g_Gl2Compatible)
      g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
    else
      g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
  }
}

static void initGeometry() {
  initGround();
  initCubes();
  initSpheres();
}

int main(int argc, char * argv[]) {

  try {

    initGlutState(argc,argv);

    glewInit(); // load the OpenGL extensions

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");

    initGLState();
    initShaders();
    initGeometry();
    glutMainLoop();

    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
