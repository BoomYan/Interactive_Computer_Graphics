////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

// #include <cstddef>
// #include <vector>
// #include <list>
// #include <string>
// #include <memory>
// #include <map>
// #include <fstream>
// #include <stdexcept>


// #ifdef __MAC__
// #   include <OpenGL/gl3.h>
// #   include <GLUT/glut.h>
// #else
// #   include <GL/glew.h>
// #   include <GL/glut.h>
// #endif

// #include "ppm.h"
// #include "cvec.h"
// #include "matrix4.h"
// #include "rigtform.h"
// // #include "glsupport.h"
// #include "geometrymaker.h"
// #include "arcball.h"
// #include "scenegraph.h"
// #include "sgutils.h"

// #include "asstcommon.h"
// #include "drawer.h"
// #include "picker.h"
// #include "geometry.h"
// #include "mesh.h"


#include <cstddef>
#include <vector>
#include <list>
#include <string>
#include <memory>
#include <map>
#include <fstream>
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
//#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
//#include "glsupport.h"
#include "arcball.h"
#include "quat.h"
#include "rigtform.h"

#include "ppm.h"
#include "asstcommon.h"
#include "scenegraph.h"
#include "drawer.h"
#include "picker.h"

#include <iostream>
#include <fstream>
#include <sstream>

#include "sgutils.h"
#include "list"

#include "geometry.h"

#include "mesh.h"
using namespace std;

// G L O B A L S ///////////////////////////////////////////////////

// --------- IMPORTANT --------------------------------------------------------
// Before you start working on this assignment, set the following variable
// properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// OpenGL 3.x+ with GLSL 1.5.
//
// Set g_Gl2Compatible = true to use GLSL 1.0 and g_Gl2Compatible = false to
// use GLSL 1.5. Use GLSL 1.5 unless your system does not support it.
//
// If g_Gl2Compatible=true, shaders with -gl2 suffix will be loaded.
// If g_Gl2Compatible=false, shaders with -gl3 suffix will be loaded.
// To complete the assignment you only need to edit the shader files that get
// loaded
// ----------------------------------------------------------------------------
const bool g_Gl2Compatible = false;


static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

enum SkyMode {WORLD_SKY=0, SKY_SKY=1};

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static bool g_spaceDown = false;         // space state, for middle mouse emulation
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;

static SkyMode g_activeCameraFrame = WORLD_SKY;

static bool g_displayArcball = true;
static double g_arcballScreenRadius = 100; // number of pixels
static double g_arcballScale = 1;

static bool g_pickingMode = false;

static bool g_playingAnimation = false;
static float g_scaleFactor = 0.5;
static float g_cubeAnimationSpeed = 1.0;
static int g_subdivisions = 0;
static bool g_smoothShaded = true;

// Global variables for used physical simulation
static const Cvec3 g_gravity(0, -0.5, 0);  // gavity vector
static double g_timeStep = 0.02;
static double g_numStepsPerFrame = 10;
static double g_damping = 0.96;
static double g_stiffness = 4;
static int g_simulationsPerSecond = 60;
static RigTForm g_bunnyRbt;
static bool g_shellNeedsUpdate = false;

static std::vector<Cvec3> g_tipPos,        // should be hair tip pos in world-space coordinates
                          g_tipVelocity;   // should be hair tip velocity in world-space coordinates


// -------- Shaders

static shared_ptr<Material> g_redDiffuseMat,
                            g_blueDiffuseMat,
                            g_bumpFloorMat,
                            g_arcballMat,
                            g_pickingMat,
                            g_lightMat,
                            g_specularMat;

shared_ptr<Material> g_overridingMaterial;

static shared_ptr<Material> g_bunnyMat; // for the bunny

static vector<shared_ptr<Material> > g_bunnyShellMats; // for bunny shells

// --------- Geometry

typedef SgGeometryShapeNode MyShapeNode;

// New Geometry
static const int g_numShells = 24; // constants defining how many layers of shells
static double g_furHeight = 0.21;
static double g_hairyness = 0.7;

static shared_ptr<SimpleGeometryPN> g_bunnyGeometry;
static vector<shared_ptr<SimpleGeometryPNX> > g_bunnyShellGeometries;
static Mesh g_bunnyMesh;

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube, g_sphere;

// --------- Scene

//static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space

static shared_ptr<SgRootNode> g_world;
static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node;
static shared_ptr<SgRbtNode> g_light1Node, g_light2Node;
static shared_ptr<SgRbtNode> g_meshNode;

static shared_ptr<SgRbtNode> g_currentCameraNode;
static shared_ptr<SgRbtNode> g_currentPickedRbtNode;

static Mesh g_mesh, g_refMesh;
static shared_ptr<SimpleGeometryPN> g_meshGeometry;

static shared_ptr<SgRbtNode> g_bunnyNode;

class Animator {
public:
  typedef vector<shared_ptr<SgRbtNode> > SgRbtNodes;
  typedef vector<RigTForm> KeyFrame;
  typedef list<KeyFrame> KeyFrames;
  typedef KeyFrames::iterator KeyFrameIter;

private:
  SgRbtNodes nodes_;
  KeyFrames keyFrames_;

public:
  void attachSceneGraph(shared_ptr<SgNode> root) {
    nodes_.clear();
    keyFrames_.clear();
    dumpSgRbtNodes(root, nodes_);
  }

  void loadAnimation(const char *filename) {
    ifstream f(filename, ios::binary);
    if (!f)
      throw runtime_error(string("Cannot load ") + filename);
    int numFrames, numRbtsPerFrame;
    f >> numFrames >> numRbtsPerFrame;
    if (numRbtsPerFrame != nodes_.size()) {
      cerr << "Number of Rbt per frame in " << filename
           <<" does not match number of SgRbtNodes in the current scene graph.";
      return;
    }

    Cvec3 t;
    Quat r;
    keyFrames_.clear();
    for (int i = 0; i < numFrames; ++i) {
      keyFrames_.push_back(KeyFrame());
      keyFrames_.back().reserve(numRbtsPerFrame);
      for (int j = 0; j < numRbtsPerFrame; ++j) {
        f >> t[0] >> t[1] >> t[2] >> r[0] >> r[1] >> r[2] >> r[3];
        keyFrames_.back().push_back(RigTForm(t, r));
      }
    }
  }

  void saveAnimation(const char *filename) {
    ofstream f(filename, ios::binary);
    int numRbtsPerFrame = nodes_.size();
    f << getNumKeyFrames() << ' ' << numRbtsPerFrame << '\n';
    for (KeyFrames::const_iterator frameIter = keyFrames_.begin(), e = keyFrames_.end(); frameIter != e; ++frameIter) {
      for (int j = 0; j < numRbtsPerFrame; ++j) {
        const RigTForm& rbt = (*frameIter)[j];
        const Cvec3& t = rbt.getTranslation();
        const Quat& r = rbt.getRotation();
        f << t[0] << ' ' << t[1] << ' ' << t[2] << ' '
        << r[0] << ' ' << r[1] << ' ' << r[2] << ' ' << r[3] << '\n';
      }
    }
  }

  int getNumKeyFrames() const {
    return keyFrames_.size();
  }

  int getNumRbtNodes() const {
    return nodes_.size();
  }

  // t can be in the range [0, keyFrames_.size()-3]. Fractional amount like 1.5 is allowed.
  void animate(double t) {
    if (t < 0 || t > keyFrames_.size() - 3)
      throw runtime_error("Invalid animation time parameter. Must be in the range [0, numKeyFrames - 3]");

    t += 1; // interpret the key frames to be at t= -1, 0, 1, 2, ...
    const int integralT = int(floor(t));
    const double fraction = t - integralT;

    KeyFrameIter f0 = getNthKeyFrame(integralT), f1 = f0, f2 = f0, f3 = f0;
    --f0;
    ++f2;
    ++f3; ++f3;

    // Don't try to go to the last frame.
    if (f3 == keyFramesEnd()) {
      f3 = f2;
    }
    
    // for (int i = 0, n = nodes_.size(); i < n; ++i) {
    //   nodes_[i]->setRbt(CRinterpolate((*f0)[i], (*f1)[i], (*f2)[i], (*f3)[i], fraction));
    // }
  }

  KeyFrameIter keyFramesBegin() {
    return keyFrames_.begin();
  }

  KeyFrameIter keyFramesEnd() {
    return keyFrames_.end();
  }

  KeyFrameIter getNthKeyFrame(int n) {
    KeyFrameIter frameIter = keyFrames_.begin();
    advance(frameIter, n);
    return frameIter;
  }

  void deleteKeyFrame(KeyFrameIter keyFrameIter) {
    keyFrames_.erase(keyFrameIter);
  }

  void pullKeyFrameFromSg(KeyFrameIter keyFrameIter) {
    for (int i = 0, n = nodes_.size(); i < n; ++i) {
      (*keyFrameIter)[i] = nodes_[i]->getRbt();
    }
  }

  void pushKeyFrameToSg(KeyFrameIter keyFrameIter) {
    for (int i = 0, n = nodes_.size(); i < n; ++i) {
      nodes_[i]->setRbt((*keyFrameIter)[i]);
    }
  }

  KeyFrameIter insertEmptyKeyFrameAfter(KeyFrameIter beforeFrame) {
    if (beforeFrame != keyFrames_.end())
      ++beforeFrame;

    KeyFrameIter frameIter = keyFrames_.insert(beforeFrame, KeyFrame());
    frameIter->resize(nodes_.size());
    return frameIter;
  }

};

static int g_msBetweenKeyFrames = 2000; // 2 seconds between keyframes
static int g_animateFramesPerSecond = 60; // frames to render per second during animation playback


static Animator g_animator;
static Animator::KeyFrameIter g_curKeyFrame;
static int g_curKeyFrameNum;

///////////////// END OF G L O B A L S //////////////////////////////////////////////////

static void initGround() {
  int ibLen, vbLen;
  getPlaneVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makePlane(g_groundSize*2, vtx.begin(), idx.begin());
  g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initCubes() {
  int ibLen, vbLen;
  getCubeVbIbLen(vbLen, ibLen);

  // Temporary storage for cube Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);

  makeCube(1, vtx.begin(), idx.begin());
  g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
}

static void initSphere() {
  int ibLen, vbLen;
  getSphereVbIbLen(20, 10, vbLen, ibLen);

  // Temporary storage for sphere Geometry
  vector<VertexPNTBX> vtx(vbLen);
  vector<unsigned short> idx(ibLen);
  makeSphere(1, 20, 10, vtx.begin(), idx.begin());
  g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
}

static void initBunnyMeshes() {
  g_bunnyMesh.load("bunny.mesh");

  vector<VertexPN> vertices;

  // Smooth shading code.
  for (int vertexidx = 0; vertexidx < g_bunnyMesh.getNumVertices(); vertexidx++) {
    g_bunnyMesh.getVertex(vertexidx).setNormal(Cvec3(0));
  }

  // Average normals (smooth shading)
  for (int faceidx = 0; faceidx < g_bunnyMesh.getNumFaces(); faceidx++) {
    Mesh::Face cur_face = g_bunnyMesh.getFace(faceidx);
    Cvec3 cur_face_normal = cur_face.getNormal();

    for (int vertexidx = 0; vertexidx < cur_face.getNumVertices(); vertexidx++) {
      cur_face.getVertex(vertexidx).setNormal(cur_face.getVertex(vertexidx).getNormal() + cur_face_normal);
    }
  }

  // Normalize normals (smooth shading)
  for (int vertexidx = 0; vertexidx < g_bunnyMesh.getNumVertices(); vertexidx++) {
    Cvec3 normal = g_bunnyMesh.getVertex(vertexidx).getNormal();
    if (norm2(normal) > CS175_EPS2) {
      g_bunnyMesh.getVertex(vertexidx).setNormal(normalize(normal));
    }
  }

  for (int faceidx = 0; faceidx < g_bunnyMesh.getNumFaces(); faceidx++) {
    Mesh::Face cur_face = g_bunnyMesh.getFace(faceidx);

    for (int triVertidx = 0; triVertidx < 3; triVertidx++) {
      vertices.push_back(VertexPN(cur_face.getVertex(triVertidx).getPosition(), cur_face.getVertex(triVertidx).getNormal()));
    }
  }

  g_bunnyGeometry.reset(new SimpleGeometryPN());
  g_bunnyGeometry->upload(&vertices[0], vertices.size());

  // Now allocate array of SimpleGeometryPNX to for shells, one per layer
  g_bunnyShellGeometries.resize(g_numShells);
  for (int i = 0; i < g_numShells; ++i) {
    g_bunnyShellGeometries[i].reset(new SimpleGeometryPNX());
  }
}

static void initMesh() {
  g_mesh.load("cube.mesh");

  vector<VertexPN> vertices;

  // Smooth shading code.
  for (int vertexidx = 0; vertexidx < g_mesh.getNumVertices(); vertexidx++) {
    g_mesh.getVertex(vertexidx).setNormal(Cvec3(0));
  }

  // Average normals (smooth shading)
  for (int faceidx = 0; faceidx < g_mesh.getNumFaces(); faceidx++) {
    Mesh::Face cur_face = g_mesh.getFace(faceidx);
    Cvec3 cur_face_normal = cur_face.getNormal();

    for (int vertexidx = 0; vertexidx < cur_face.getNumVertices(); vertexidx++) {
      cur_face.getVertex(vertexidx).setNormal(cur_face.getVertex(vertexidx).getNormal() + cur_face_normal);
    }
  }

  // Normalize normals (smooth shading)
  for (int vertexidx = 0; vertexidx < g_mesh.getNumVertices(); vertexidx++) {
    Cvec3 normal = g_mesh.getVertex(vertexidx).getNormal();
    if (norm2(normal) > CS175_EPS2) {
      g_mesh.getVertex(vertexidx).setNormal(normalize(normal));
    }
  }

  for (int faceidx = 0; faceidx < g_mesh.getNumFaces(); faceidx++) {
    Mesh::Face cur_face = g_mesh.getFace(faceidx);

    for (int triVertidx = 0; triVertidx < 3; triVertidx++) {
      vertices.push_back(VertexPN(cur_face.getVertex(triVertidx).getPosition(), cur_face.getVertex(triVertidx).getNormal()));
    }

    for (int triVertidx = 0; triVertidx < 3; triVertidx++) {
      vertices.push_back(VertexPN(cur_face.getVertex((2 + triVertidx) % 4).getPosition(), cur_face.getVertex((2 + triVertidx) % 4).getNormal()));
    }
  }

  g_meshGeometry.reset(new SimpleGeometryPN());
  g_meshGeometry->upload(&vertices[0], vertices.size());
  g_refMesh = g_mesh;
}

static void initRobots() {
  // Init whatever geometry needed for the robots
}

// takes a projection matrix and send to the the shaders

inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
  uniforms.put("uProjMatrix", projMatrix);
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

enum ManipMode {
  ARCBALL_ON_PICKED,
  ARCBALL_ON_SKY,
  EGO_MOTION
};

static ManipMode getManipMode() {
  // if nothing is picked or the picked transform is the transfrom we are viewing from
  if (g_currentPickedRbtNode == nullptr || g_currentPickedRbtNode == g_currentCameraNode) {
    if (g_currentCameraNode == g_skyNode && g_activeCameraFrame == WORLD_SKY)
      return ARCBALL_ON_SKY;
    else
      return EGO_MOTION;
  }
  else
    return ARCBALL_ON_PICKED;
}

static bool shouldUseArcball() {
  return getManipMode() != EGO_MOTION;
}

// The translation part of the aux frame either comes from the current
// active object, or is the identity matrix when
static RigTForm getArcballRbt() {
  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    return getPathAccumRbt(g_world, g_currentPickedRbtNode);
  case ARCBALL_ON_SKY:
    return RigTForm();
  case EGO_MOTION:
    return getPathAccumRbt(g_world, g_currentCameraNode);
  default:
    throw runtime_error("Invalid ManipMode");
  }
}

static void updateArcballScale() {
  RigTForm arcballEye = inv(getPathAccumRbt(g_world, g_currentCameraNode)) * getArcballRbt();
  double depth = arcballEye.getTranslation()[2];
  if (depth > -CS175_EPS)
    g_arcballScale = 0.02;
  else
    g_arcballScale = getScreenToEyeScale(depth, g_frustFovY, g_windowHeight);
}

static void drawArcBall(Uniforms uniforms) {

  RigTForm arcballEye = inv(getPathAccumRbt(g_world, g_currentCameraNode)) * getArcballRbt();
  Matrix4 MVM = rigTFormToMatrix(arcballEye) * Matrix4::makeScale(Cvec3(1, 1, 1) * g_arcballScale * g_arcballScreenRadius);
  sendModelViewNormalMatrix(uniforms, MVM, normalMatrix(MVM));

  g_arcballMat->draw(*g_sphere, uniforms);
}

Cvec3 rbtToCvec3(RigTForm rbt, Cvec3 vec) {
  return Cvec3(rbt * Cvec4(vec,1));
}

static void updateShellGeometry() {
  // TASK 1 and 3 TODO: finish this function as part of Task 1 and Task 3
  // TASK 1 COMPLETED
  g_bunnyRbt = getPathAccumRbt(g_world, g_bunnyNode);

  for (int i = 0; i < g_numShells; i++) {
    vector<VertexPNX> verticies;

    for (int faceidx = 0; faceidx < g_bunnyMesh.getNumFaces(); faceidx++) {
    Mesh::Face cur_face = g_bunnyMesh.getFace(faceidx);
    Cvec3 position, normal;

      for (int vertexidx = 0; vertexidx < cur_face.getNumVertices(); vertexidx++) {
        Cvec3 t = g_tipPos[g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getIndex()];
        Cvec3 p = g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getPosition();
        normal = g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getNormal();
        Cvec3 s = p + (normal * g_furHeight);
        Cvec3 n = normal * (g_furHeight / g_numShells);

        t = rbtToCvec3(inv(g_bunnyRbt), t);
        Cvec3 d = (t - p - normal * g_furHeight) / ((g_numShells - 1) * g_numShells / 2);

        if (i == 0) {
          position = p;
        } else {
          position = p + (n * i) + (d * ((i*i - i)/2));
        }

        n = n + ((d * ((i*i - i)/2)) * i);
      
        if (vertexidx == 0) {
          verticies.push_back(VertexPNX(position, n, Cvec2(0,0)));
        } else if (vertexidx == 1) {
          verticies.push_back(VertexPNX(position, n, (g_hairyness, 0)));
        } else {
          verticies.push_back(VertexPNX(position, n, (0, g_hairyness)));
        }
      }
    }
    g_bunnyShellGeometries[i]->upload(&verticies[0], verticies.size());
  }
  g_shellNeedsUpdate = false;
}

static void hairsSimulationCallback(int dontCare) {
  // TASK 2 TODO: wrte dynamics simulation code here as part of TASK2
  g_bunnyRbt = getPathAccumRbt(g_world, g_bunnyNode, 0);

  for (int faceidx = 0; faceidx < g_bunnyMesh.getNumFaces(); faceidx++) {
    Mesh::Face cur_face = g_bunnyMesh.getFace(faceidx);  

    for (int vertexidx = 0; vertexidx < cur_face.getNumVertices(); vertexidx++) {
      Cvec3 p = g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getPosition();
      Cvec3 n = g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getNormal();
      Cvec3 t = g_tipPos[g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getIndex()];
      Cvec3 v = g_tipVelocity[g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getIndex()];
      Cvec3 s = p + (n * g_furHeight);

      s = rbtToCvec3(g_bunnyRbt, s);
      p = rbtToCvec3(g_bunnyRbt, p);

      Cvec3 f = g_gravity + ((s-t) * g_stiffness);

      t = t + (v * g_timeStep);
      t = p + (t - p) / norm(t - p) * g_furHeight;

      v = (v + (f * g_timeStep)) * g_damping;

      g_tipPos[g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getIndex()] = t;
      g_tipVelocity[g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getIndex()] = v;
    }
  }

  g_shellNeedsUpdate = true;

  // schedule this to get called again
  glutTimerFunc(1000/g_simulationsPerSecond, hairsSimulationCallback, 0);
  glutPostRedisplay(); // signal redisplaying
}

static void initSimulation() {
  // get accumRbt
  g_bunnyRbt = getPathAccumRbt(g_world, g_bunnyNode, 0);

  g_tipPos.resize(g_bunnyMesh.getNumVertices(), Cvec3(0));
  g_tipVelocity.resize(g_bunnyMesh.getNumVertices(), Cvec3(0));

  g_tipPos.clear();
  g_tipVelocity.clear();

  // TASK 1 TODO: initialize g_tipPos to "at-rest" hair tips in world coordinates
  // COMPLETED

  for (int faceidx = 0; faceidx < g_bunnyMesh.getNumFaces(); faceidx++) {
    Mesh::Face cur_face = g_bunnyMesh.getFace(faceidx);
    
    for (int vertexidx = 0; vertexidx < cur_face.getNumVertices(); vertexidx++) {
      Cvec3 p = g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getPosition();
      Cvec3 n = g_bunnyMesh.getFace(faceidx).getVertex(vertexidx).getNormal();
      //p = rbtToCvec3(inv(g_bunnyRbt), p);
      //n = rbtToCvec3(inv(g_bunnyRbt), n);
      Cvec3 t = p + (n * g_furHeight);
      g_tipPos.push_back(t);
      g_tipVelocity.push_back(Cvec3(0,0,0));
    }
  }

  // Starts hair tip simulation
  hairsSimulationCallback(0);
}

static void drawStuff(bool picking) {
  if (g_shellNeedsUpdate) {
    updateShellGeometry();
  }

  // if we are not translating, update arcball scale
  if (!(g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton) || (g_mouseLClickButton && !g_mouseRClickButton && g_spaceDown)))
    updateArcballScale();

  // empty uniforms
  Uniforms uniforms;

  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(uniforms, projmat);

  const RigTForm eyeRbt = getPathAccumRbt(g_world, g_currentCameraNode);
  const RigTForm invEyeRbt = inv(eyeRbt);

  const Cvec3 light1 = getPathAccumRbt(g_world, g_light1Node).getTranslation();
  const Cvec3 light2 = getPathAccumRbt(g_world, g_light2Node).getTranslation();

  uniforms.put("uLight", Cvec3(invEyeRbt * Cvec4(light1,1)));
  uniforms.put("uLight2", Cvec3(invEyeRbt * Cvec4(light2,1)));

  if (!picking) {
    Drawer drawer(invEyeRbt, uniforms);
    g_world->accept(drawer);

    if (g_displayArcball && shouldUseArcball())
      drawArcBall(uniforms);
  }
  else {
    Picker picker(invEyeRbt, uniforms);
    g_overridingMaterial = g_pickingMat;
    g_world->accept(picker);
    g_overridingMaterial.reset();

    glFlush();
    g_currentPickedRbtNode = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
    if (g_currentPickedRbtNode == g_groundNode)
      g_currentPickedRbtNode = shared_ptr<SgRbtNode>(); // set to NULL

    cout << (g_currentPickedRbtNode ? "Part picked" : "No part picked") << endl;
  }
}

static void display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  drawStuff(false);

  glutSwapBuffers();

  checkGlErrors();
}

static void pick() {
  // We need to set the clear color to black, for pick rendering.
  // so let's save the clear color
  GLdouble clearColor[4];
  glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

  glClearColor(0, 0, 0, 0);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // No more glUseProgram
  drawStuff(true); // no more curSS

  // Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
  // to see result of the pick rendering pass
  // glutSwapBuffers();

  //Now set back the clear color
  glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

  checkGlErrors();
}

bool interpolateAndDisplay(float t) {
  if (t > g_animator.getNumKeyFrames() - 3)
    return true;
  g_animator.animate(t);
  return false;
}

static void animateTimerCallback(int ms) {
  double t = (double)ms / g_msBetweenKeyFrames;
  bool endReached = interpolateAndDisplay(t);
  if (g_playingAnimation && !endReached) {
    glutTimerFunc(1000/g_animateFramesPerSecond, animateTimerCallback, ms + 1000/g_animateFramesPerSecond);
  }
  else {
    cerr << "Finished playing animation" << endl;
    g_curKeyFrame = g_animator.keyFramesEnd();
    advance(g_curKeyFrame, -2);
    g_animator.pushKeyFrameToSg(g_curKeyFrame);
    g_playingAnimation = false;

    g_curKeyFrameNum = g_animator.getNumKeyFrames() - 2;
    cerr << "Now at frame [" << g_curKeyFrameNum << "]" << endl;
  }
  display();
}

static Mesh subdivide(Mesh mesh) {
  const int nFaces = mesh.getNumFaces();
  const int nEdges = mesh.getNumEdges();
  const int nVertices = mesh.getNumVertices();

  // compute face vertices
  for (int i = 0; i < nFaces; i++) {
    Mesh::Face f = mesh.getFace(i);
    int nfVertices = f.getNumVertices();
    
    Cvec3 p = Cvec3();
    for (int vi = 0; vi < nfVertices; vi++) {
      p += f.getVertex(vi).getPosition();
    }
    p = p /((float) nfVertices);

    mesh.setNewFaceVertex(f,p);
  }

  // compute edge vertices
  for (int i = 0; i < nEdges; i++) {
    Mesh::Edge e = mesh.getEdge(i);

    Cvec3 p = 
      (e.getVertex(0).getPosition() +
       e.getVertex(1).getPosition() +
       mesh.getNewFaceVertex(e.getFace(0)) +
       mesh.getNewFaceVertex(e.getFace(1))) * 0.25; 

    mesh.setNewEdgeVertex(e,p);
  }

  // compute vertex vertices
  for (int i = 0; i < nVertices; i++) {
    Mesh::Vertex v = mesh.getVertex(i);
    Mesh::VertexIterator iter = v.getIterator();
    Mesh::VertexIterator initIter = iter;

    int nv = 0;
    Cvec3 vvSum = Cvec3();
    Cvec3 fvSum = Cvec3();

    do {
      nv++;
      vvSum += iter.getVertex().getPosition();
      fvSum += mesh.getNewFaceVertex(iter.getFace());
      ++iter;
    } while (initIter != iter);

    float fnv = float(nv);
    float nvinvsq = 1.0 / pow(double(fnv),2.0);
    Cvec3 p = v.getPosition()*((fnv-2)/fnv) + vvSum*nvinvsq + fvSum*nvinvsq;
    mesh.setNewVertexVertex(v,p);
  }

  mesh.subdivide();
  return mesh; 
}

static void animateMesh(int t) {

  const float sinVal = sin((double(t) / 100.0) * 2 * M_PI);
  const float cosVal = cos((double(t) / 100.0) * 2 * M_PI);
  const float mods[] = {sinVal, cosVal};

  g_mesh = g_refMesh;
  for (int vertexidx = 0; vertexidx < g_mesh.getNumVertices(); vertexidx++) {
    Cvec3 oldPosition = g_mesh.getVertex(vertexidx).getPosition();

    oldPosition += oldPosition * (g_scaleFactor * mods[vertexidx%2]);

    g_mesh.getVertex(vertexidx).setPosition(oldPosition);
  }

  for (int i = 0; i < g_subdivisions; i++) {
    g_mesh = subdivide(g_mesh);
  }

  // Smooth shading code.
  for (int vertexidx = 0; vertexidx < g_mesh.getNumVertices(); vertexidx++) {
    g_mesh.getVertex(vertexidx).setNormal(Cvec3(0));
  }

  // Average normals (smooth shading)
  for (int faceidx = 0; faceidx < g_mesh.getNumFaces(); faceidx++) {
    Mesh::Face cur_face = g_mesh.getFace(faceidx);
    Cvec3 cur_face_normal = cur_face.getNormal();

    for (int vertexidx = 0; vertexidx < cur_face.getNumVertices(); vertexidx++) {
      cur_face.getVertex(vertexidx).setNormal(cur_face.getVertex(vertexidx).getNormal() + cur_face_normal);
    }
  }

  // Normalize normals (smooth shading)
  for (int vertexidx = 0; vertexidx < g_mesh.getNumVertices(); vertexidx++) {
    Cvec3 normal = g_mesh.getVertex(vertexidx).getNormal();
    if (norm2(normal) > CS175_EPS2) {
      g_mesh.getVertex(vertexidx).setNormal(normalize(normal));
    }
  }

  vector<VertexPN> vertices;

  for (int faceidx = 0; faceidx < g_mesh.getNumFaces(); faceidx++) {
    Mesh::Face cur_face = g_mesh.getFace(faceidx);

    for (int triVertidx = 0; triVertidx < 3; triVertidx++) {
      Cvec3 normal;
      if (g_smoothShaded) {
        normal = cur_face.getVertex(triVertidx).getNormal();
      } else {
        normal = cur_face.getNormal();
      }
      vertices.push_back(VertexPN(cur_face.getVertex(triVertidx).getPosition(), normal));
    }

    for (int triVertidx = 0; triVertidx < 3; triVertidx++) {
      Cvec3 normal;
      if (g_smoothShaded) {
        normal = cur_face.getVertex((2 + triVertidx) % 4).getNormal();
      } else {
        normal = cur_face.getNormal();
      }
      vertices.push_back(VertexPN(cur_face.getVertex((2 + triVertidx) % 4).getPosition(), normal));
    }
  }

  g_meshGeometry->upload(&vertices[0], vertices.size());

  glutPostRedisplay();

  if (t++ > 100) {
    t = 0;
  }

  glutTimerFunc(1000 / (g_cubeAnimationSpeed * g_animateFramesPerSecond), animateMesh, t);

}


static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  g_arcballScreenRadius = max(1.0, min(h, w) * 0.25);
  updateFrustFovY();
  glutPostRedisplay();
}

static Cvec3 getArcballDirection(const Cvec2& p, const double r) {
  double n2 = norm2(p);
  if (n2 >= r*r)
    return normalize(Cvec3(p, 0));
  else
    return normalize(Cvec3(p, sqrt(r*r - n2)));
}

static RigTForm moveArcball(const Cvec2& p0, const Cvec2& p1) {
  const Matrix4 projMatrix = makeProjectionMatrix();
  const RigTForm eyeInverse = inv(getPathAccumRbt(g_world, g_currentCameraNode));
  const Cvec3 arcballCenter = getArcballRbt().getTranslation();
  const Cvec3 arcballCenter_ec = Cvec3(eyeInverse * Cvec4(arcballCenter, 1));

  if (arcballCenter_ec[2] > -CS175_EPS)
    return RigTForm();

  Cvec2 ballScreenCenter = getScreenSpaceCoord(arcballCenter_ec,
                                               projMatrix, g_frustNear, g_frustFovY, g_windowWidth, g_windowHeight);
  const Cvec3 v0 = getArcballDirection(p0 - ballScreenCenter, g_arcballScreenRadius);
  const Cvec3 v1 = getArcballDirection(p1 - ballScreenCenter, g_arcballScreenRadius);

  return RigTForm(Quat(0.0, v1[0], v1[1], v1[2]) * Quat(0.0, -v0[0], -v0[1], -v0[2]));
}

static RigTForm doMtoOwrtA(const RigTForm& M, const RigTForm& O, const RigTForm& A) {
  return A * M * inv(A) * O;
}

static RigTForm getMRbt(const double dx, const double dy) {
  RigTForm M;

  if (g_mouseLClickButton && !g_mouseRClickButton && !g_spaceDown) {
    if (shouldUseArcball())
      M = moveArcball(Cvec2(g_mouseClickX, g_mouseClickY), Cvec2(g_mouseClickX + dx, g_mouseClickY + dy));
    else
      M = RigTForm(Quat::makeXRotation(-dy) * Quat::makeYRotation(dx));
  }
  else {
    double movementScale = getManipMode() == EGO_MOTION ? 0.02 : g_arcballScale;
    if (g_mouseRClickButton && !g_mouseLClickButton) {
      M = RigTForm(Cvec3(dx, dy, 0) * movementScale);
    }
    else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton) || (g_mouseLClickButton && g_spaceDown)) {
      M = RigTForm(Cvec3(0, 0, -dy) * movementScale);
    }
  }

  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    break;
  case ARCBALL_ON_SKY:
    M = inv(M);
    break;
  case EGO_MOTION:
    if (g_mouseLClickButton && !g_mouseRClickButton && !g_spaceDown) // only invert rotation
      M = inv(M);
    break;
  }
  return M;
}

static RigTForm makeMixedFrame(const RigTForm& objRbt, const RigTForm& eyeRbt) {
  return transFact(objRbt) * linFact(eyeRbt);
}

// l = w X Y Z
// o = l O
// a = w A = l (Z Y X)^1 A = l A'
// o = a (A')^-1 O
//   => a M (A')^-1 O = l A' M (A')^-1 O

static void motion(const int x, const int y) {
  if (!g_mouseClickDown)
    return;

  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;

  const RigTForm M = getMRbt(dx, dy);   // the "action" matrix

  // the matrix for the auxiliary frame (the w.r.t.)
  RigTForm A = makeMixedFrame(getArcballRbt(), getPathAccumRbt(g_world, g_currentCameraNode));

  shared_ptr<SgRbtNode> target;
  switch (getManipMode()) {
  case ARCBALL_ON_PICKED:
    target = g_currentPickedRbtNode;
    break;
  case ARCBALL_ON_SKY:
    target = g_skyNode;
    break;
  case EGO_MOTION:
    target = g_currentCameraNode;
    break;
  }

  A = inv(getPathAccumRbt(g_world, target, 1)) * A;

  target->setRbt(doMtoOwrtA(M, target->getRbt(), A));

  g_mouseClickX += dx;
  g_mouseClickY += dy;
  glutPostRedisplay();  // we always redraw if we changed the scene
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

  if (g_pickingMode && button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
    pick();
    g_pickingMode = false;
    cerr << "Picking mode is off" << endl;
    glutPostRedisplay(); // request redisplay since the arcball will have moved
  }
  glutPostRedisplay();
}

static void keyboardUp(const unsigned char key, const int x, const int y) {
  switch (key) {
  case ' ':
    g_spaceDown = false;
    break;
  }
  glutPostRedisplay();
}

static void keyboard(const unsigned char key, const int x, const int y) {
  switch (key) {
  case ' ':
    g_spaceDown = true;
    break;
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "f\t\tToggle flat shading on/off.\n"
    << "p\t\tUse mouse to pick a part to edit\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n"
    << "a\t\tToggle display arcball\n"
    << "w\t\tWrite animation to animation.txt\n"
    << "i\t\tRead animation from animation.txt\n"
    << "c\t\tCopy frame to scene\n"
    << "u\t\tCopy sceneto frame\n"
    << "n\t\tCreate new frame after current frame and copy scene to it\n"
    << "d\t\tDelete frame\n"
    << ">\t\tGo to next frame\n"
    << "<\t\tGo to prev. frame\n"
    << "y\t\tPlay/Stop animation\n"
    << endl;
    break;
  case 'f':
    g_smoothShaded = !g_smoothShaded;
    if (g_smoothShaded == true) {
      cout << "Smooth shading enabled" << endl;
    } else {
      cout << "Flat shading enabled" << endl;
    }
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'v':
  {
    shared_ptr<SgRbtNode> viewers[] = {g_skyNode, g_robot1Node, g_robot2Node};
    for (int i = 0; i < 3; ++i) {
      if (g_currentCameraNode == viewers[i]) {
        g_currentCameraNode = viewers[(i+1)%3];
        break;
      }
    }
  }
  break;
  case 'p':
    g_pickingMode = !g_pickingMode;
    cerr << "Picking mode is " << (g_pickingMode ? "on" : "off") << endl;
    break;
  case 'm':
    g_activeCameraFrame = SkyMode((g_activeCameraFrame+1) % 2);
    cerr << "Editing sky eye w.r.t. " << (g_activeCameraFrame == WORLD_SKY ? "world-sky frame\n" : "sky-sky frame\n") << endl;
    break;
  case 'a':
    g_displayArcball = !g_displayArcball;
    break;
  case 'u':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }

    if (g_curKeyFrame == g_animator.keyFramesEnd()) { // only possible when frame list is empty
      cerr << "Create new frame [0]."  << endl;
      g_curKeyFrame = g_animator.insertEmptyKeyFrameAfter(g_animator.keyFramesBegin());
      g_curKeyFrameNum = 0;
    }
    cerr << "Copying scene graph to current frame [" << g_curKeyFrameNum << "]" << endl;
    g_animator.pullKeyFrameFromSg(g_curKeyFrame);
    break;
  case 'n':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    if (g_animator.getNumKeyFrames() != 0)
      ++g_curKeyFrameNum;
    g_curKeyFrame = g_animator.insertEmptyKeyFrameAfter(g_curKeyFrame);
    g_animator.pullKeyFrameFromSg(g_curKeyFrame);
    cerr << "Create new frame [" << g_curKeyFrameNum << "]" << endl;
    break;
  case 'c':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    if (g_curKeyFrame != g_animator.keyFramesEnd()) {
      cerr << "Loading current key frame [" << g_curKeyFrameNum << "] to scene graph" << endl;
      g_animator.pushKeyFrameToSg(g_curKeyFrame);
    }
    else {
      cerr << "No key frame defined" << endl;
    }
    break;
  case 'd':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    if (g_curKeyFrame != g_animator.keyFramesEnd()) {
      Animator::KeyFrameIter newCurKeyFrame = g_curKeyFrame;
      cerr << "Deleting current frame [" << g_curKeyFrameNum << "]" << endl;;
      if (g_curKeyFrame == g_animator.keyFramesBegin()) {
        ++newCurKeyFrame;
      }
      else {
        --newCurKeyFrame;
        --g_curKeyFrameNum;
      }
      g_animator.deleteKeyFrame(g_curKeyFrame);
      g_curKeyFrame = newCurKeyFrame;
      if (g_curKeyFrame != g_animator.keyFramesEnd()) {
        g_animator.pushKeyFrameToSg(g_curKeyFrame);
        cerr << "Now at frame [" << g_curKeyFrameNum << "]" << endl;
      }
      else
        cerr << "No frames defined" << endl;
    }
    else {
      cerr << "Frame list is now EMPTY" << endl;
    }
    break;
  case '>':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    if (g_curKeyFrame != g_animator.keyFramesEnd()) {
      if (++g_curKeyFrame == g_animator.keyFramesEnd())
        --g_curKeyFrame;
      else {
        ++g_curKeyFrameNum;
        g_animator.pushKeyFrameToSg(g_curKeyFrame);
        cerr << "Stepped forward to frame [" << g_curKeyFrameNum <<"]" << endl;
      }
    }
    break;
  case '<':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    if (g_curKeyFrame != g_animator.keyFramesBegin()) {
      --g_curKeyFrame;
      --g_curKeyFrameNum;
      g_animator.pushKeyFrameToSg(g_curKeyFrame);
      cerr << "Stepped backward to frame [" << g_curKeyFrameNum << "]" << endl;
    }
    break;
  case 'w':
    cerr << "Writing animation to animation.txt\n";
    g_animator.saveAnimation("animation.txt");
    break;
  case 'i':
    if (g_playingAnimation) {
      cerr << "Cannot operate when playing animation" << endl;
      break;
    }
    cerr << "Reading animation from animation.txt\n";
    g_animator.loadAnimation("animation.txt");
    g_curKeyFrame = g_animator.keyFramesBegin();
    cerr << g_animator.getNumKeyFrames() << " frames read.\n";
    if (g_curKeyFrame != g_animator.keyFramesEnd()) {
      g_animator.pushKeyFrameToSg(g_curKeyFrame);
      cerr << "Now at frame [0]" << endl;
    }
    g_curKeyFrameNum = 0;
    break;
  case '-':
    g_msBetweenKeyFrames = min(g_msBetweenKeyFrames + 100, 10000);
    cerr << g_msBetweenKeyFrames << " ms between keyframes.\n";
    break;
  case '+':
    g_msBetweenKeyFrames = max(g_msBetweenKeyFrames - 100, 100);
    cerr << g_msBetweenKeyFrames << " ms between keyframes.\n";
    break;
  case 'y':
    if (!g_playingAnimation) {
      if (g_animator.getNumKeyFrames() < 4) {
        cerr << " Cannot play animation with less than 4 keyframes." << endl;
      }
      else {
        g_playingAnimation = true;
        cerr << "Playing animation... "<< endl;
        animateTimerCallback(0);
      }
    }
    else {
      cerr << "Stopping animation... " << endl;
      g_playingAnimation = false;
    }
    break;
  case '0':
    if (g_subdivisions < 6) {
      g_subdivisions++;
      cerr<<"Subdividing..."<<endl;
    } else {
      cerr<<"Cannot subdivide further."<<endl;
    }
    break;
  case '9':
    if (g_subdivisions > 0) {
      g_subdivisions--;
      cerr<<"Unsubdividing..."<<endl;
    } else {
      cerr<<"Cannot unsubdivide further."<<endl;
    }
    break;
  case '7':
    cerr<<"halving cube animation speed"<<endl;
    g_cubeAnimationSpeed /= 2.0;
    break;
  case '8':
    cerr<<"doubling cube animation speed"<<endl;
    g_cubeAnimationSpeed *= 2.0;
    break;
  }

  // Sanity check that our g_curKeyFrameNum is in sync with the g_curKeyFrame
  if (g_animator.getNumKeyFrames() > 0)
    assert(g_animator.getNthKeyFrame(g_curKeyFrameNum) == g_curKeyFrame);

  glutPostRedisplay();
}

static void specialKeyboard(const int key, const int x, const int y) {
  switch (key) {
  case GLUT_KEY_RIGHT:
    g_furHeight *= 1.05;
    cerr << "fur height = " << g_furHeight << std::endl;
    break;
  case GLUT_KEY_LEFT:
    g_furHeight /= 1.05;
    std::cerr << "fur height = " << g_furHeight << std::endl;
    break;
  case GLUT_KEY_UP:
    g_hairyness *= 1.05;
    cerr << "hairyness = " << g_hairyness << std::endl;
    break;
  case GLUT_KEY_DOWN:
    g_hairyness /= 1.05;
    cerr << "hairyness = " << g_hairyness << std::endl;
    break;
  }
  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
#ifdef __MAC__
  glutInitDisplayMode(GLUT_3_2_CORE_PROFILE|GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH); // core profile flag is required for GL 3.2 on Mac
#else
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
#endif
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Assignment 5");                       // title the window

  glutIgnoreKeyRepeat(true);                              // avoids repeated keyboard calls when holding space to emulate middle mouse

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
  glutKeyboardUpFunc(keyboardUp);
  glutSpecialFunc(specialKeyboard);
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

static void initMaterials() {
  // Create some prototype materials
  Material diffuse("./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader");
  Material solid("./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader");
  Material specular("./shaders/basic-gl3.vshader", "./shaders/specular-gl3.fshader");

  // copy diffuse prototype and set red color
  g_redDiffuseMat.reset(new Material(diffuse));
  g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0.5, 0));

  // copy diffuse prototype and set blue color
  g_blueDiffuseMat.reset(new Material(diffuse));
  g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0.5, 1));

  // normal mapping material
  g_bumpFloorMat.reset(new Material("./shaders/normal-gl3.vshader", "./shaders/normal-gl3.fshader"));
  g_bumpFloorMat->getUniforms().put("uTexColor",  dynamic_pointer_cast<Texture>(shared_ptr<ImageTexture>(new ImageTexture("Fieldstone.ppm", true))));
  g_bumpFloorMat->getUniforms().put("uTexNormal",  dynamic_pointer_cast<Texture>(shared_ptr<ImageTexture>(new ImageTexture("FieldstoneNormal.ppm", false))));

  // copy solid prototype, and set to wireframed rendering
  g_arcballMat.reset(new Material(solid));
  g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
  g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

  // copy solid prototype, and set to color white
  g_lightMat.reset(new Material(solid));
  g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

  // Specular shader
  g_specularMat.reset(new Material(specular));
  g_specularMat->getUniforms().put("uColor", Cvec3f(1.0f, 1.0f, 1.0f));

  // pick shader
  g_pickingMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/pick-gl3.fshader"));

  // bunny material
  g_bunnyMat.reset(new Material("./shaders/basic-gl3.vshader", "./shaders/bunny-gl3.fshader"));
  g_bunnyMat->getUniforms()
  .put("uColorAmbient", Cvec3f(0.45f, 0.3f, 0.3f))
  .put("uColorDiffuse", Cvec3f(0.2f, 0.2f, 0.2f));

  // bunny shell materials;
  shared_ptr<ImageTexture> shellTexture(new ImageTexture("shell.ppm", false)); // common shell texture

  // needs to enable repeating of texture coordinates
  shellTexture->bind();
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

  // eachy layer of the shell uses a different material, though the materials will share the
  // same shader files and some common uniforms. hence we create a prototype here, and will
  // copy from the prototype later
  Material bunnyShellMatPrototype("./shaders/bunny-shell-gl3.vshader", "./shaders/bunny-shell-gl3.fshader");
  bunnyShellMatPrototype.getUniforms().put("uTexShell",  dynamic_pointer_cast<Texture>(shellTexture));
  bunnyShellMatPrototype.getRenderStates()
  .blendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) // set blending mode
  .enable(GL_BLEND) // enable blending
  .disable(GL_CULL_FACE); // disable culling

  // allocate array of materials
  g_bunnyShellMats.resize(g_numShells);
  for (int i = 0; i < g_numShells; ++i) {
    g_bunnyShellMats[i].reset(new Material(bunnyShellMatPrototype)); // copy from the prototype
    // but set a different exponent for blending transparency
    g_bunnyShellMats[i]->getUniforms().put("uAlphaExponent", 2.f + 5.f * float(i + 1)/g_numShells);
  }
};

static void initGeometry() {
  initGround();
  initCubes();
  initSphere();
  initRobots();
  initMesh();
  initBunnyMeshes();
}

static void constructRobot(shared_ptr<SgTransformNode> base, shared_ptr<Material> material) {
  const double ARM_LEN = 0.7,
               ARM_THICK = 0.25,
               LEG_LEN = 1,
               LEG_THICK = 0.25,
               TORSO_LEN = 1.5,
               TORSO_THICK = 0.25,
               TORSO_WIDTH = 1,
               HEAD_SIZE = 0.7;
  const int NUM_JOINTS = 10,
            NUM_SHAPES = 10;

  struct JointDesc {
    int parent;
    float x, y, z;
  };

  JointDesc jointDesc[NUM_JOINTS] = {
    {-1}, // torso
    {0,  TORSO_WIDTH/2, TORSO_LEN/2, 0}, // upper right arm
    {0, -TORSO_WIDTH/2, TORSO_LEN/2, 0}, // upper left arm
    {1,  ARM_LEN, 0, 0}, // lower right arm
    {2, -ARM_LEN, 0, 0}, // lower left arm
    {0, TORSO_WIDTH/2-LEG_THICK/2, -TORSO_LEN/2, 0}, // upper right leg
    {0, -TORSO_WIDTH/2+LEG_THICK/2, -TORSO_LEN/2, 0}, // upper left leg
    {5, 0, -LEG_LEN, 0}, // lower right leg
    {6, 0, -LEG_LEN, 0}, // lower left
    {0, 0, TORSO_LEN/2, 0} // head
  };

  struct ShapeDesc {
    int parentJointId;
    float x, y, z, sx, sy, sz;
    shared_ptr<Geometry> geometry;
  };

  ShapeDesc shapeDesc[NUM_SHAPES] = {
    {0, 0,         0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube}, // torso
    {1, ARM_LEN/2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphere}, // upper right arm
    {2, -ARM_LEN/2, 0, 0, ARM_LEN/2, ARM_THICK/2, ARM_THICK/2, g_sphere}, // upper left arm
    {3, ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower right arm
    {4, -ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // lower left arm
    {5, 0, -LEG_LEN/2, 0, LEG_THICK/2, LEG_LEN/2, LEG_THICK/2, g_sphere}, // upper right leg
    {6, 0, -LEG_LEN/2, 0, LEG_THICK/2, LEG_LEN/2, LEG_THICK/2, g_sphere}, // upper left leg
    {7, 0, -LEG_LEN/2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower right leg
    {8, 0, -LEG_LEN/2, 0, LEG_THICK, LEG_LEN, LEG_THICK, g_cube}, // lower left leg
    {9, 0, HEAD_SIZE/2 * 1.5, 0, HEAD_SIZE/2, HEAD_SIZE/2, HEAD_SIZE/2, g_sphere}, // head
  };

  shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

  for (int i = 0; i < NUM_JOINTS; ++i) {
    if (jointDesc[i].parent == -1)
      jointNodes[i] = base;
    else {
      jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
      jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
    }
  }
  for (int i = 0; i < NUM_SHAPES; ++i) {
    shared_ptr<MyShapeNode> shape(
      new MyShapeNode(shapeDesc[i].geometry,
                      material,
                      Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
                      Cvec3(0, 0, 0),
                      Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
    jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
  }
}

static void initScene() {
  g_world.reset(new SgRootNode());

  g_light1Node.reset(new SgRbtNode(RigTForm(Cvec3(2.0,3.0,14.0))));
  g_light1Node->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_sphere, g_lightMat, Cvec3(0,0,0))));

  g_light2Node.reset(new SgRbtNode(RigTForm(Cvec3(-2.0,-3.0,-5.0))));
  g_light2Node->addChild(shared_ptr<MyShapeNode>(
        new MyShapeNode(g_sphere, g_lightMat, Cvec3(0,0,0))));

  g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));


  g_groundNode.reset(new SgRbtNode());
  g_groundNode->addChild(shared_ptr<MyShapeNode>(
                           new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));

  g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
  g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

  constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
  constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot

  g_meshNode.reset(new SgRbtNode(RigTForm(Cvec3(5,5,0))));
  g_meshNode->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_meshGeometry, g_specularMat)));

  g_bunnyNode.reset(new SgRbtNode());
  g_bunnyNode->addChild(shared_ptr<MyShapeNode>(
                          new MyShapeNode(g_bunnyGeometry, g_bunnyMat)));

  for (int i = 0; i < g_numShells; ++i) {
    g_bunnyNode->addChild(shared_ptr<MyShapeNode>(
                            new MyShapeNode(g_bunnyShellGeometries[i], g_bunnyShellMats[i])));
  }

  g_world->addChild(g_light1Node);
  g_world->addChild(g_light2Node);
  g_world->addChild(g_skyNode);
  g_world->addChild(g_groundNode);
  g_world->addChild(g_robot1Node);
  g_world->addChild(g_robot2Node);
  g_world->addChild(g_meshNode);
  g_world->addChild(g_bunnyNode);

  g_currentCameraNode = g_skyNode;
}

static void initAnimation() {
  g_animator.attachSceneGraph(g_world);
  g_curKeyFrame = g_animator.keyFramesBegin();
  animateMesh(0);
}

int main(int argc, char * argv[]) {
  try {
    initGlutState(argc,argv);

    // on Mac, we shouldn't use GLEW.

#ifndef __MAC__
    glewInit(); // load the OpenGL extensions
#endif

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.5") << endl;

#ifndef __MAC__
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");
#endif

    initGLState();
    initMaterials();
    initGeometry();
    initScene();
    initAnimation();
    initSimulation();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}

// ////////////////////////////////////////////////////////////////////////
// //
// //   Harvard University
// //   CS175 : Computer Graphics
// //   Professor Steven Gortler
// //
// ////////////////////////////////////////////////////////////////////////
// //  These skeleton codes are later altered by Ming Jin,
// //  for "CS6533: Interactive Computer Graphics", 
// //  taught by Prof. Andy Nealen at NYU
// ////////////////////////////////////////////////////////////////////////

// #include <vector>
// #include <string>
// #include <memory>
// #include <stdexcept>
// // #if __GNUG__
// // #   include <tr1/memory>
// // #endif

// #include <GL/glew.h>
// #ifdef __MAC__
// #   include <GLUT/glut.h>
// #else
// #   include <GL/glut.h>
// #endif

// #include "cvec.h"
// //#include "matrix4.h"
// #include "geometrymaker.h"
// #include "ppm.h"
// //#include "glsupport.h"
// #include "arcball.h"
// #include "quat.h"
// #include "rigtform.h"

// #include "ppm.h"
// #include "asstcommon.h"
// #include "scenegraph.h"
// #include "drawer.h"
// #include "picker.h"

// #include <iostream>
// #include <fstream>
// #include <sstream>

// #include "sgutils.h"
// #include "list"

// #include "geometry.h"

// #include "mesh.h"

// using namespace std;      // for string, vector, iostream, and other standard C++ stuff
// // using namespace tr1; // for shared_ptr

// // G L O B A L S ///////////////////////////////////////////////////

// // --------- IMPORTANT --------------------------------------------------------
// // Before you start working on this assignment, set the following variable
// // properly to indicate whether you want to use OpenGL 2.x with GLSL 1.0 or
// // OpenGL 3.x+ with GLSL 1.3.
// //
// // Set g_Gl2Compatible                               = true to use GLSL 1.0 and g_Gl2Compatible = false to
// // use GLSL 1.3. Make sure that your machine supports the version of GLSL you
// // are using. In particular, on Mac OS X currently there is no way of using
// // OpenGL 3.x with GLSL 1.3 when GLUT is used.
// //
// // If g_Gl2Compatible                                = true, shaders with -gl2 suffix will be loaded.
// // If g_Gl2Compatible                                = false, shaders with -gl3 suffix will be loaded.
// // To complete the assignment you only need to edit the shader files that get
// // loaded
// // ----------------------------------------------------------------------------
// const bool g_Gl2Compatible                           = true;


// static const float g_frustMinFov                     = 60.0;  // A minimal of 60 degree field of view
// static float g_frustFovY                             = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

// static const float g_frustNear                       = -0.1;    // near plane
// static const float g_frustFar                        = -50.0;    // far plane
// static const float g_groundY                         = -2.0;      // y coordinate of the ground
// static const float g_groundSize                      = 10.0;   // half the ground length

// static int g_windowWidth                             = 512;
// static int g_windowHeight                            = 512;
// static bool g_mouseClickDown                         = false;    // is the mouse button pressed
// static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
// static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
// static int g_activeShader                            = 0;
// // ========================================
// // TODO: you can add global variables here
// // ========================================
// static int g_currentViewIndex                        = 0;                 // 0 is sky view, 1 is cube1 view, 2 is cube2 view
// static int g_currentManipulatingObject               = 0;   // 0 is sky, 1 is cube 1, 2 is cube2 
// static int g_currentSkyView                          = 0;                // 0 is world-sky view, 1 is sky-sky view

// static bool g_picking                                = false;
// static const int PICKING_SHADER                      = 2;

// static Mesh g_mesh;
// static bool g_smoothShading                          = true;
// static int g_bubblingMs                              = 0;
// static bool g_bubbling                               = false;
// static Mesh g_meshOriginal;
// static int g_subDivisions                            = 0;
// static int g_msBetweenBubblingFrames                 = 1000;

// // Global variables for used physical simulation
// static const Cvec3 g_gravity(0, -0.5, 0);  // gravity vector
// static double g_timeStep                             = 0.02;
// static double g_numStepsPerFrame                     = 10;
// static double g_damping                              = 0.96;
// static double g_stiffness                            = 4;
// static int g_simulationsPerSecond                    = 60;
// static bool g_shellNeedsUpdate                       = false;

// static std::vector<Cvec3> g_tipPos,        // should be hair tip pos in world-space coordinates
// g_tipVelocity;   // should be hair tip velocity in world-space coordinates





// static shared_ptr<Material> g_redDiffuseMat,
// g_blueDiffuseMat,
// g_bumpFloorMat,
// g_arcballMat,
// g_pickingMat,
// g_lightMat,
// g_meshMat
// 	;


// static shared_ptr<Material> g_bunnyMat; // for the bunny
// static vector<shared_ptr<Material> > g_bunnyShellMats; // for bunny shells

// // New Geometry
// static const int g_numShells                         = 24; // constants defining how many layers of shells
// static double g_furHeight                            = 0.5;
// static double g_hairyness                            = 1.2;

// static shared_ptr<SimpleGeometryPN> g_bunnyGeometry;
// static vector<shared_ptr<SimpleGeometryPNX> > g_bunnyShellGeometries;
// static Mesh g_bunnyMesh;

// // New Scene node
// static shared_ptr<SgRbtNode> g_bunnyNode;



// shared_ptr<Material> g_overridingMaterial;
// typedef SgGeometryShapeNode MyShapeNode;

// // static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

// // --------- Geometry

// // Macro used to obtain relative offset of a field within a struct
// #define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// // // A vertex with floating point position and normal
// // struct VertexPN {
// //   Cvec3f p, n;

// //   VertexPN() {}
// //   VertexPN(float x, float y, float z,
// //            float nx, float ny, float nz)
// //     : p(x,y,z), n(nx, ny, nz)
// //   {}

// //   // Define copy constructor and assignment operator from GenericVertex so we can
// //   // use make* functions from geometrymaker.h
// //   VertexPN(const GenericVertex& v) {
// //     *this                                         = v;
// //   }

// //   VertexPN& operator                              = (const GenericVertex& v) {
// //     p                                             = v.pos;
// //     n                                             = v.normal;
// //     return *this;
// //   }
// // };

// // struct Geometry {
// //   GlBufferObject vbo, ibo;
// //   int vboLen, iboLen;

// //   Geometry(VertexPN *vtx, unsigned short *idx, int vboLen, int iboLen) {
// //     this->vboLen                                  = vboLen;
// //     this->iboLen                                  = iboLen;

// //     // Now create the VBO and IBO
// //     glBindBuffer(GL_ARRAY_BUFFER, vbo);
// //     glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);

// //     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
// //     glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
// //   }

// //   void draw(Uniforms& uniforms) {
// //     // Enable the attributes used by our shader

// //     safe_glEnableVertexAttribArray(uniforms.h_aPosition);
// //     safe_glEnableVertexAttribArray(uniforms.h_aNormal);

// //     // bind vbo
// //     glBindBuffer(GL_ARRAY_BUFFER, vbo);
// //     safe_glVertexAttribPointer(uniforms.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p));

// //     safe_glVertexAttribPointer(uniforms.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));

// //     // bind ibo
// //     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);

// //     // draw!
// //     glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);

// //     // Disable the attributes used by our shader
// //     safe_glDisableVertexAttribArray(uniforms.h_aPosition);
// //     safe_glDisableVertexAttribArray(uniforms.h_aNormal);

// //   }
// // };

// //typedef SgGeometryShapeNode<Geometry> MyShapeNode;


// // Vertex buffer and index buffer associated with the ground and cube geometry
// static shared_ptr<Geometry> g_ground, g_cube, g_sphere;
// static shared_ptr<SgRootNode> g_world;
// static shared_ptr<SgRbtNode> g_skyNode, g_groundNode, g_robot1Node, g_robot2Node, g_light1Node, g_light2Node, g_meshNode;
// static shared_ptr<SgRbtNode> g_currentPickedRbtNode;
// static shared_ptr<SgRbtNode> g_currentView; 

// static shared_ptr<SimpleGeometryPN> g_meshGeometry;
// // --------- Scene

// // static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space

// static Cvec3f g_objectColors[2]                      = {Cvec3f(1, 0, 0), Cvec3f(0, 0, 1)};

// static const Cvec3f g_arcballColor                   = Cvec3f(0, 1, 0);
// static double g_arcballScreenRadius                  = 1.0;
// static double g_arcballScale                         = 1.0;

// static RigTForm g_auxFrame;  // g_auxFrame is the auxiliary frame for manipulation


// static int g_msBetweenKeyFrames                      = 2000; // 2 seconds between keyframes
// static int g_animateFramesPerSecond                  = 60; // frames to render per second during animation playback

// static const string g_fileName                       = "frames.txt";

// vector<shared_ptr<SgRbtNode> > g_rbts;


// list< vector<RigTForm> > g_keyframes;

// list< vector<RigTForm> >::iterator g_currentKeyFrame = g_keyframes.begin();

// //
// ///////////////// END OF G L O B A L S //////////////////////////////////////////////////



// static void setPicking(bool a);

// static void initGround() {
// 	int ibLen, vbLen;
// 	getPlaneVbIbLen(vbLen, ibLen);

// 	// Temporary storage for cube Geometry
// 	vector<VertexPNTBX> vtx(vbLen);
// 	vector<unsigned short> idx(ibLen);

// 	makePlane(g_groundSize*2, vtx.begin(), idx.begin());
// 	g_ground.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
// }

// static void initCubes() {
// 	int ibLen, vbLen;
// 	getCubeVbIbLen(vbLen, ibLen);

// 	// Temporary storage for cube Geometry
// 	vector<VertexPNTBX> vtx(vbLen);
// 	vector<unsigned short> idx(ibLen);

// 	makeCube(1, vtx.begin(), idx.begin());
// 	g_cube.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vbLen, ibLen));
// }

// static void initSphere() {
// 	int ibLen, vbLen;
// 	getSphereVbIbLen(20, 10, vbLen, ibLen);

// 	// Temporary storage for sphere Geometry
// 	vector<VertexPNTBX> vtx(vbLen);
// 	vector<unsigned short> idx(ibLen);
// 	makeSphere(1, 20, 10, vtx.begin(), idx.begin());
// 	g_sphere.reset(new SimpleIndexedGeometryPNTBX(&vtx[0], &idx[0], vtx.size(), idx.size()));
// }


// static void initMesh() {
// 	g_mesh.load("cube.mesh");
// 	g_meshOriginal.load("cube.mesh");
// 	g_meshGeometry.reset(new SimpleGeometryPN);
// 	g_meshGeometry->upload(g_mesh, g_smoothShading);
// }


// static void subdivideMeshCatmullClark(Mesh& mesh) {
// 	for (int i                                          = 0; i < mesh.getNumFaces(); i++) {
// 		Mesh::Face face                                    = mesh.getFace(i);
// 		mesh.setNewFaceVertex(face, Cvec3());
// 		for (int j                                         = 0; j < face.getNumVertices(); j++) {
// 			mesh.setNewFaceVertex(face, mesh.getNewFaceVertex(face) + face.getVertex(j).getPosition());
// 		}
// 		mesh.setNewFaceVertex(face, mesh.getNewFaceVertex(face)/face.getNumVertices());
// 	}
// 	for (int i                                          = 0; i < mesh.getNumEdges(); i++) {
// 		Mesh::Edge edge                                    = mesh.getEdge(i);
// 		mesh.setNewEdgeVertex(edge, Cvec3());
// 		for (int j                                         = 0; j < 2; j++) {
// 			mesh.setNewEdgeVertex(edge, mesh.getNewEdgeVertex(edge) + edge.getVertex(j).getPosition() + mesh.getNewFaceVertex(edge.getFace(j)));
// 		}
// 		mesh.setNewEdgeVertex(edge, mesh.getNewEdgeVertex(edge)/4);
// 	}
// 	for (int i                                          = 0; i < mesh.getNumVertices(); i++) {
// 		Mesh::Vertex vertex                                = mesh.getVertex(i);
// 		Mesh::VertexIterator iterator(vertex.getIterator());
// 		Mesh::VertexIterator it(iterator);
// 		float vtNum                                        = 0;
// 		Cvec3 vtSum                                        = Cvec3();
// 		Cvec3 fcSum                                        = Cvec3();
// 		do {

// 			vtSum += iterator.getVertex().getPosition();
// 			fcSum += mesh.getNewFaceVertex(iterator.getFace());
// 			vtNum++;
// 		}
// 		while (++iterator != it);
// 		mesh.setNewVertexVertex(vertex, vertex.getPosition() * (vtNum-2)/vtNum + vtSum/(vtNum*vtNum)+fcSum/(vtNum*vtNum));
// 	}
// 	mesh.subdivide();
// }


// // takes a projection matrix and send to the the shaders
// // static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
// //   GLfloat glmatrix[16];
// //   projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
// //   safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
// // }
// inline void sendProjectionMatrix(Uniforms& uniforms, const Matrix4& projMatrix) {
// 	uniforms.put("uProjMatrix", projMatrix);
// }



// static bool nonEgoCubeManipulation() {
// 	return g_currentPickedRbtNode != g_skyNode && g_currentView != g_currentPickedRbtNode;
// }

// static bool useArcball() {
// 	return (g_currentManipulatingObject == 0 && g_currentSkyView == 0) || nonEgoCubeManipulation();
// }

// static bool worldSkyManipulation() {
// 	return g_currentPickedRbtNode == g_skyNode && g_currentView == g_skyNode && g_currentSkyView == 0;
// }




// // // takes MVM and its normal matrix to the shaders
// // static void sendModelViewNormalMatrix(const ShaderState& curSS, const Matrix4& MVM, const Matrix4& NMVM) {
// //   GLfloat glmatrix[16];
// //   MVM.writeToColumnMajorMatrix(glmatrix); // send MVM
// //   safe_glUniformMatrix4fv(curSS.h_uModelViewMatrix, glmatrix);

// //   NMVM.writeToColumnMajorMatrix(glmatrix); // send NMVM
// //   safe_glUniformMatrix4fv(curSS.h_uNormalMatrix, glmatrix);
// // }

// // update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
// static void updateFrustFovY() {
// 	if (g_windowWidth >= g_windowHeight)
// 		g_frustFovY                                        = g_frustMinFov;
// 	else {
// 		const double RAD_PER_DEG                           = 0.5 * CS175_PI/180;
// 		g_frustFovY                                        = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
// 	}
// }

// static Matrix4 makeProjectionMatrix() {
// 	return Matrix4::makeProjection(
// 		g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
// 	g_frustNear, g_frustFar);
// }

// //set auxFrame for transformation
// static void setAFrame(){

// 	if (g_currentPickedRbtNode == g_skyNode) { 
// 		if (g_currentView == g_skyNode) { 
// 			if (g_currentSkyView == 0) {

// 				g_auxFrame                                       = linFact(g_skyNode->getRbt()); 
// 			} else {
// 				g_auxFrame                                       = g_skyNode->getRbt();
// 			}
// 		}
// 	} else {
// 		if (g_currentView == g_skyNode) { 
// 			g_auxFrame                                        = inv(getPathAccumRbt(g_world, g_currentPickedRbtNode, 1)) *
// 				transFact(getPathAccumRbt(g_world, g_currentPickedRbtNode)) * linFact(getPathAccumRbt(g_world, g_skyNode));
// 		} else { 
// 			g_auxFrame                                        = inv(getPathAccumRbt(g_world, g_currentPickedRbtNode, 1)) * getPathAccumRbt(g_world, g_currentPickedRbtNode);
// 		}
// 	}

// }


// // Specifying shell geometries based on g_tipPos, g_furHeight, and g_numShells.
// // You need to call this function whenver the shell needs to be updated
// ///////////////////..........
// static void updateShellGeometry() {
// 	// TASK 1 and 3 TODO: finish this function as part of Task 1 and Task 3

// 	RigTForm surface                                    = getPathAccumRbt(g_world, g_bunnyNode);
// 	Cvec3 p                                             = g_bunnyMesh.getFace(0).getVertex(0).getPosition();
// 	Cvec3 s                                             = g_tipPos[g_bunnyMesh.getFace(0).getVertex(0).getIndex()];
// 	for(int i                                           = 0; i < g_numShells; i++) {
// 		vector<VertexPNX> vtx;

// 		for(int f                                          = 0; f < g_bunnyMesh.getNumFaces(); f++) {
// 			Cvec3 pos, norm;
// 			Cvec2 texture;
// 			for(int v                                         = 0; v < g_bunnyMesh.getFace(i).getNumVertices(); v++) {

// 				if (v==0) {
// 					texture                                         = Cvec2(0,0);
// 				} else if (v==1) {
// 					texture                                         = Cvec2(g_hairyness, 0);
// 				} else {
// 					texture                                         = Cvec2(0, g_hairyness);
// 				}

// 				Cvec3 p                                          = g_bunnyMesh.getFace(f).getVertex(v).getPosition();
// 				Cvec3 s                                          = g_tipPos[g_bunnyMesh.getFace(f).getVertex(v).getIndex()];
// 				s                                                = Cvec3((surface) * Cvec4(s, 0));        
        
// 				Cvec3 n                                          = g_bunnyMesh.getFace(f).getVertex(v).getNormal() * (g_furHeight / g_numShells);
// 				Cvec3 d                                          = (s * 2 - p * 2 - n * 2 * g_numShells) / (g_numShells * (g_numShells-1));    
// 				pos                                              = p + (n*i) + (d * (i * (i-1))/2);

// 				if(i==0) { pos =p;       }

// 				vtx.push_back(VertexPNX(pos, n+d*i, texture));
// 			}
// 		}
// 		g_bunnyShellGeometries[i]->upload(&vtx[0],vtx.size());
// 	}
// 	g_shellNeedsUpdate                                  = false;

// }


// static void drawStuff(bool picking) {

// 	if (g_shellNeedsUpdate)
// 		updateShellGeometry();


// 	Uniforms uniforms; 
// 	setAFrame();

// 	// build & send proj. matrix to vshader
// 	const Matrix4 projmat                               = makeProjectionMatrix();


// 	sendProjectionMatrix(uniforms, projmat);

// 	const RigTForm eyeRbt                               = getPathAccumRbt(g_world, g_currentView);

// 	const RigTForm invEyeRbt                            = inv(eyeRbt);



// 	// const Cvec3 eyeLight1                            = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
// 	// const Cvec3 eyeLight2                            = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
// 	// safe_glUniform3f(uniforms.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
// 	// safe_glUniform3f(uniforms.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);
// 	Cvec3 light1                                        = getPathAccumRbt(g_world, g_light1Node).getTranslation();
// 	Cvec3 light2                                        = getPathAccumRbt(g_world, g_light2Node).getTranslation();
// 	uniforms.put("uLight", Cvec3(invEyeRbt * Cvec4(light1, 1)));
// 	uniforms.put("uLight2", Cvec3(invEyeRbt * Cvec4(light2, 1)));

// 	if (!picking) {
// 		Drawer drawer(invEyeRbt, uniforms);
// 		g_world->accept(drawer);

// 		RigTForm sphereTarget;
// 		if (g_currentPickedRbtNode == g_skyNode) {
// 			if (g_currentSkyView == 0) {
// 				sphereTarget                                     = inv(RigTForm());
// 			} else {
// 				sphereTarget                                     = eyeRbt;
// 			}
// 		} else {
// 			sphereTarget                                      = getPathAccumRbt(g_world, g_currentPickedRbtNode);
// 		}

// 		if (!g_mouseMClickButton && !(g_mouseLClickButton && g_mouseRClickButton) && useArcball()) {
// 			g_arcballScale                                    = getScreenToEyeScale(
// 				(inv(eyeRbt) * sphereTarget).getTranslation()[2],
// 			g_frustFovY,
// 			g_windowHeight
// 				);
// 		}


// 		//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

// 		const Matrix4 scale                                = Matrix4::makeScale(g_arcballScale * g_arcballScreenRadius);
// 		Matrix4 MVM                                        = rigTFormToMatrix(invEyeRbt * sphereTarget) * scale;
// 		Matrix4 NMVM                                       = normalMatrix(MVM);
// 		sendModelViewNormalMatrix(uniforms, MVM, NMVM);
// 		//safe_glUniform3f(uniforms.h_uColor, g_arcballColor[0], g_arcballColor[1], g_arcballColor[2]);
// 		uniforms.put("uColor", Cvec3(g_arcballColor[0], g_arcballColor[1], g_arcballColor[2]));
// 		g_arcballMat->draw(*g_sphere, uniforms);
// 		//g_sphere->draw(uniforms);

// 		/* draw filled */
// 		//glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // draw filled again
// 	}
// 	else {
// 		// intialize the picker with our uniforms, as opposed to curSS
// 		Picker picker(invEyeRbt, uniforms);

// 		// set overiding material to our picking material
// 		g_overridingMaterial                               = g_pickingMat;

// 		g_world->accept(picker);

// 		// unset the overriding material
// 		g_overridingMaterial.reset();
// 		glFlush();
// 		g_currentPickedRbtNode                             = picker.getRbtNodeAtXY(g_mouseClickX, g_mouseClickY);
// 		if ((g_currentPickedRbtNode == g_groundNode)||(g_currentPickedRbtNode == nullptr))
// 			g_currentPickedRbtNode                            = g_skyNode; 
// 	}

// }


// static void display() {

// 	// glUseProgram(g_shaderStates[g_activeShader]->program);
// 	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

// 	drawStuff(false);

// 	glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

// 	// if (!g_picking) {
// 	//   glutSwapBuffers();
// 	// }

// 	checkGlErrors();
// }

// static void reshape(const int w, const int h) {
// 	g_windowWidth                                       = w;
// 	g_windowHeight                                      = h;
// 	glViewport(0, 0, w, h);
// 	cerr << "Size of window is now " << w << "x" << h << endl;

// 	g_arcballScreenRadius                               = 0.3 * min(g_windowWidth, g_windowHeight);

// 	updateFrustFovY();
// 	glutPostRedisplay();

// }

// static RigTForm getArcballRotation(const int x, const int y) {
// 	const RigTForm eyeRbt                               = getPathAccumRbt(g_world, g_currentView);
// 	const RigTForm object                               = getPathAccumRbt(g_world, g_currentPickedRbtNode);

// 	const bool world_sky_manipulation                   = worldSkyManipulation();

// 	Cvec2 sphereOnScreenCoords;
// 	if (world_sky_manipulation) {
// 		sphereOnScreenCoords                               = Cvec2((g_windowWidth - 1) / 2.0, (g_windowHeight - 1) / 2.0);
// 	} else {
// 		sphereOnScreenCoords                               = getScreenSpaceCoord(
// 			(inv(eyeRbt) * object).getTranslation(),
// 		makeProjectionMatrix(),
// 		g_frustNear,
// 		g_frustFovY,
// 		g_windowWidth,
// 		g_windowHeight
// 			);
// 	}

// 	const Cvec3 sphere_center                           = Cvec3(sphereOnScreenCoords, 0);
// 	const Cvec3 p1                                      = Cvec3(g_mouseClickX, g_mouseClickY, 0) - sphere_center;
// 	const Cvec3 p2                                      = Cvec3(x, y, 0) - sphere_center;

// 	const Cvec3 v1                                      = normalize(Cvec3(p1[0], p1[1],
// 	sqrt(max(0.0, pow(g_arcballScreenRadius, 2) - pow(p1[0], 2) - pow(p1[1], 2)))));
// 	const Cvec3 v2                                      = normalize(Cvec3(p2[0], p2[1],
// 	sqrt(max(0.0, pow(g_arcballScreenRadius, 2) - pow(p2[0], 2) - pow(p2[1], 2)))));

// 	if (world_sky_manipulation) {
// 		return RigTForm(Quat(0, v1 * -1.0) * Quat(0, v2));
// 	} else {
// 		return RigTForm(Quat(0, v2) * Quat(0, v1 * -1.0));
// 	}
// }


// static void motion(const int x, const int y) {    
// 	if (g_currentView != g_skyNode && g_currentPickedRbtNode == g_skyNode) return;

// 	const double curr_x                                 = x;
// 	const double curr_y                                 = g_windowHeight - y - 1;
// 	const double raw_dx                                 = curr_x - g_mouseClickX;
// 	const double raw_dy                                 = curr_y - g_mouseClickY;

// 	double dx_t, dx_r, dy_t, dy_r;
// 	if (nonEgoCubeManipulation()) {
// 		dx_t                                               = raw_dx; dx_r = raw_dx;
// 		dy_t                                               = raw_dy; dy_r = raw_dy;
// 	} else if (worldSkyManipulation()) {
// 		dx_t                                               = -raw_dx; dx_r = -raw_dx;
// 		dy_t                                               = -raw_dy; dy_r = -raw_dy;
// 	} else {
// 		dx_t                                               = raw_dx; dx_r = -raw_dx;
// 		dy_t                                               = raw_dy; dy_r = -raw_dy;
// 	}


// 	const bool use_arcball                              = useArcball();  

// 	double translateFactor;
// 	if (use_arcball) {
// 		translateFactor                                    = g_arcballScale;
// 	} else {
// 		translateFactor                                    = 0.01;
// 	}

// 	setAFrame();

// 	RigTForm m;

// 	if (g_mouseLClickButton && !g_mouseRClickButton) { // left button down?
// 		if (use_arcball)
// 			m                                                 = getArcballRotation(curr_x, curr_y);
// 		else
// 			m                                                 = RigTForm(Quat::makeXRotation(-dy_r) * Quat::makeYRotation(dx_r));
// 	}
// 	else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
// 		m                                                  = RigTForm(Cvec3(dx_t, dy_t, 0) * translateFactor);
// 	}
// 	else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton)) {  // middle or (left and right) button down?
// 		m                                                  = RigTForm(Cvec3(0, 0, -dy_t) * translateFactor);
// 	}

// 	if (g_mouseClickDown) {
// 		m                                                  = g_auxFrame * m * inv(g_auxFrame);

// 		g_currentPickedRbtNode->setRbt(m * g_currentPickedRbtNode->getRbt());
// 	}


// 	g_mouseClickX                                       = curr_x;
// 	g_mouseClickY                                       = curr_y;


// 	glutPostRedisplay();
// }

// static void reset()
// {
// 	// =========================================================
// 	// TODO:
// 	// - reset g_skyRbt and g_objectRbt to their default values
// 	// - reset the views and manipulation mode to default
// 	// - reset sky camera mode to use the "world-sky" frame
// 	// =========================================================
// 	// g_skyRbt                                         = Matrix4::makeTranslation(Cvec3(0.0, 0.25, 4.0));
// 	// g_objectRbt[0]                                   = Matrix4::makeTranslation(Cvec3(-1,0,0)); 
// 	// g_objectRbt[1]                                   = Matrix4::makeTranslation(Cvec3(1,0,0)); 
// 	// g_currentViewIndex                               = 0;
// 	// g_currentManipulatingObject                      = 0;
// 	// g_currentSkyView                                 = 0;

// 	cout << "reset all to defaults not implemented" << endl;
// }

// static void pick() {
// 	// We need to set the clear color to black, for pick rendering.
// 	// so let's save the clear color
// 	GLdouble clearColor[4];
// 	glGetDoublev(GL_COLOR_CLEAR_VALUE, clearColor);

// 	glClearColor(0, 0, 0, 0);

// 	// // using PICKING_SHADER as the shader
// 	// glUseProgram(g_shaderStates[PICKING_SHADER]->program);

// 	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
// 	//drawStuff(*g_shaderStates[PICKING_SHADER], true);
// 	drawStuff(true); // no more curSS
// 	// Uncomment below and comment out the glutPostRedisplay in mouse(...) call back
// 	// to see result of the pick rendering pass
// 	// glutSwapBuffers();

// 	//Now set back the clear color
// 	glClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);

// 	checkGlErrors();
// }

// static void mouse(const int button, const int state, const int x, const int y) {
// 	g_mouseClickX                                       = x;
// 	g_mouseClickY                                       = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

// 	g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
// 	g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
// 	g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

// 	g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
// 	g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
// 	g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

// 	g_mouseClickDown                                    = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;


// 	if (g_picking && g_mouseLClickButton && !g_mouseRClickButton) {
// 		pick();
// 		setPicking(false);
// 	}
// 	glutPostRedisplay();
// }

// //set the sky view whether world-sky or sky-sky
// static void setCurrentSkyView() {
// 	if (g_currentManipulatingObject == 0 && g_currentViewIndex == 0) {
// 		g_currentSkyView++;
// 		if(g_currentSkyView==2)g_currentSkyView=0;
// 		if (g_currentSkyView == 0) {
// 			cout << "set to world-sky view" << endl;
// 		} else {
// 			cout << "set to sky-sky view" << endl;
// 		}
// 	} else {
// 		cout << "use sky view to enable this function" << endl;
// 	}
// }

// //set the current view
// static void setCurrentView() {
// 	g_currentViewIndex++;
// 	if (g_currentViewIndex == 3) g_currentViewIndex=0;
// 	if (g_currentViewIndex == 0) {
// 		cout << "Current view is sky view" << endl;
// 	} else {
// 		cout << "Current view is robot" << g_currentViewIndex << " view" << endl;

// 		switch (g_currentViewIndex) {
// 			case 0:
// 			g_currentView                                     = g_skyNode;
// 			break;
// 			case 1:
// 			g_currentView                                     = g_robot1Node;
// 			break;
// 			case 2:
// 			g_currentView                                     = g_robot2Node;
// 			break;
// 		}
// 	}
// }

// // //set the current manipulating object
// // static void setCurrentManipulatingObject(){
// //   g_currentManipulatingObject++;
// //   if (g_currentManipulatingObject == 3) g_currentManipulatingObject=0;
// //   if (g_currentManipulatingObject == 0) {
// //     cout << "Current manipulating object is sky" << endl;
// //   } else {
// //     cout << "Current manipulating object is cube" << g_currentManipulatingObject << endl;
// //   }
// // }

// //set the g_picking
// static void setPicking(bool a) {
// 	g_picking                                           = a;
// 	if (g_picking)
// 		cout << "Ready to pick" << endl;
// 	else
// 		cout << "Stop picking" << endl;
// }



// bool interpolateAndDisplay(float t) {
// 	if(t >= g_keyframes.size()-3)  return true;


// 	const double alpha                                  = t - int(t);

// 	list< vector<RigTForm> >::iterator i0               = g_keyframes.begin();
// 	std::advance(i0, int(t));
// 	list< vector<RigTForm> >::iterator i1               = i0;
// 	std::advance(i1, 1);
// 	list< vector<RigTForm> >::iterator i2               = i0;
// 	std::advance(i2, 2);
// 	list< vector<RigTForm> >::iterator i3               = i0;
// 	std::advance(i3, 3);
// 	if (i3 == g_keyframes.end()) i3 = i2;

// 	for(int i                                           = 0; i<g_rbts.size(); ++i) {

// 		g_rbts[i]->setRbt(interpolateCatmullRom((*i0)[i],(*i1)[i],(*i2)[i],(*i3)[i],alpha));
// 	} 

// 	glutPostRedisplay();

// 	return false;
// }

// static void interpolateMesh(float t) {
// 	g_mesh                                              = g_meshOriginal;
// 	for (int i                                          = 0; i < g_mesh.getNumVertices(); i++) {
// 		g_mesh.getVertex(i).setPosition(g_mesh.getVertex(i).getPosition() * (sin(t + i)*0.5+1));
// 	}

// 	for (int i                                          = 0; i < g_subDivisions; i++) {
// 		subdivideMeshCatmullClark(g_mesh);
// 	}
// 	g_meshGeometry->upload(g_mesh, g_smoothShading);
// 	glutPostRedisplay();
// }

// static void animateTimerCallback(int ms) {
// 	float t                                             = (float)ms/(float)g_msBetweenKeyFrames;
// 	bool endReached                                     = interpolateAndDisplay(t);
// 	if (!endReached)
// 		glutTimerFunc(1000/g_animateFramesPerSecond,
// 	animateTimerCallback,
// 	ms + 1000/g_animateFramesPerSecond);
// 	else
// 		cout<<"Reached the end of the animation."<<endl;

// }

// static void bubblingCallback(int ms) {
// 	float t                                             = (float)ms/(float)g_msBetweenBubblingFrames;
// 	if (g_bubbling) {
// 		g_bubblingMs                                       = ms;
// 		interpolateMesh(t);
// 		glutTimerFunc(1000/g_animateFramesPerSecond, bubblingCallback, ms + 1000/g_animateFramesPerSecond);
// 	}

// }

// static void insertKeyFrame(){
// 	vector<RigTForm> keyframe;

// 	for(vector< shared_ptr<SgRbtNode> >::iterator i     = g_rbts.begin(); i != g_rbts.end(); ++i) {
// 		keyframe.push_back((*i)->getRbt());
// 	}

// 	g_currentKeyFrame                                   = g_keyframes.insert(g_currentKeyFrame, keyframe);
// 	g_currentKeyFrame--;

// 	cout << "Inserted new keyframe." << endl;
// }

// static void updateCurrentKeyFrame(){
// 	if (g_keyframes.size() == 0) {
// 		insertKeyFrame();
// 	} else {
// 		for (int i                                         = 0; i<g_rbts.size(); ++i) {
// 			(*g_currentKeyFrame)[i]                           = g_rbts[i]->getRbt();
// 		}
// 		cerr << "Updated current keyframe." << endl;
// 	}

// }

// static void copyCurrentKeyFrameToSceneGraph(){
// 	for(int i                                           = 0; i<g_rbts.size(); ++i) {
// 		g_rbts[i]->setRbt((*g_currentKeyFrame)[i]);
// 	}

// 	cout << "Copied current keyframe to scenegraph." << endl;
// }

// static void goToPreviousKeyFrame(){
// 	if(--g_currentKeyFrame == g_keyframes.begin())
// 	{
// 		cout<<"It's the first frame"<<endl;
// 		g_currentKeyFrame++;
// 	}
// 	else{
// 		for(int i                                          = 0; i<g_rbts.size(); ++i) {
// 			g_rbts[i]->setRbt((*g_currentKeyFrame)[i]);
// 		}

// 		cout << "Went to the previous keyframe." << endl;

// 	} 

// }
// //
// static void goToNextKeyFrame(){
// 	if(++g_currentKeyFrame == g_keyframes.end())
// 	{
// 		cout<<"It's the last frame"<<endl;
// 		g_currentKeyFrame--;
// 	}
// 	else{
// 		for(int i                                          = 0; i<g_rbts.size(); ++i) {
// 			g_rbts[i]->setRbt((*g_currentKeyFrame)[i]);
// 		}

// 		cout << "Went to the next keyframe." << endl;

// 	} 

// }

// static void deleteCurrentKeyFrame(){

// 	if (g_keyframes.size() == 0) {
// 		return;
// 	}

// 	if (g_currentKeyFrame-- != g_keyframes.begin()) {

// 		g_currentKeyFrame++;
// 		g_currentKeyFrame                                  = g_keyframes.erase(g_currentKeyFrame);
// 		g_currentKeyFrame--;

// 	} else {

// 		g_currentKeyFrame++;
// 		g_currentKeyFrame                                  = g_keyframes.erase(g_currentKeyFrame);

// 	}

// 	if (g_keyframes.size() > 0) {
// 		for(int i                                          = 0; i<g_rbts.size(); ++i) {
// 			g_rbts[i]->setRbt((*g_currentKeyFrame)[i]);
// 		}    
// 	}

// 	cout << "Deleted current keyframe." << endl;
// }

// static void writeFramesToFile(){
// 	ofstream file;
// 	file.open (g_fileName);
// 	for(list< vector<RigTForm> >::iterator it           = g_keyframes.begin(); it != g_keyframes.end(); ++it) {
// 		for(int i                                          = 0; i<g_rbts.size(); ++i) {
// 			Cvec3 t_                                          = (*it)[i].getTranslation();
// 			Quat r_                                           = (*it)[i].getRotation();
// 			file << t_[0] << " " << t_[1] << " " << t_[2] << " " << r_[0] << " " << r_[1] << " " << r_[2] << " " << r_[3] << '\n';
// 		}

// 	}
// 	file.close();
// 	cout << "Written frames to file "<<g_fileName << endl;
// }

// static void inputFramesFromFile(){
// 	vector<RigTForm> keyframe;
// 	ifstream file;
// 	file.open(g_fileName);
// 	if (file == NULL) {
// 		cout << "No file found, please check the file again" <<endl;
// 	}

// 	cout << "Input not implemented" << endl;
// }

// static void setBubbling(){
// 	g_bubbling                                          = !g_bubbling;
// 	if (g_bubbling) {
// 		cout << "Bubbling animation started" << endl;
// 		bubblingCallback(g_bubblingMs);
// 	}
// 	else {
// 		cout << "Bubbling animation paused" << endl;
// 	}
// }

// static void setSmoothShading(){
// 	g_smoothShading                                     = !g_smoothShading;
// 	g_meshGeometry->upload(g_mesh, g_smoothShading);
// 	if (g_smoothShading) {
// 		cout << "Smooth shading is enalbled" << endl;
// 	}
// 	else {
// 		cout << "Smooth shading is disabled" << endl;
// 	}
// }

// static void setSubDivisions(bool i){
// 	if(i)
// 		g_subDivisions--;
// 	else
// 		g_subDivisions++;
// 	if (g_subDivisions<0) g_subDivisions                = 0;
// 	if (g_subDivisions>7) g_subDivisions                = 7;
// 	cout << "Subdivisions is set to " << g_subDivisions <<"."<<endl;
// 	g_mesh                                              = g_meshOriginal;
// 	for (int i                                          = 0; i < g_subDivisions; i++) {
// 		subdivideMeshCatmullClark(g_mesh);
// 	}
// 	g_meshGeometry->upload(g_mesh, g_smoothShading);

// }

// static void specialKeyboard(const int key, const int x, const int y) {
// 	switch (key) {
// 		case GLUT_KEY_RIGHT:
// 		g_furHeight *= 1.05;
// 		cerr << "fur height                                = " << g_furHeight << std::endl;
// 		break;
// 		case GLUT_KEY_LEFT:
// 		g_furHeight /= 1.05;
// 		std::cerr << "fur height                           = " << g_furHeight << std::endl;
// 		break;
// 		case GLUT_KEY_UP:
// 		g_hairyness *= 1.05;
// 		cerr << "hairyness                                 = " << g_hairyness << std::endl;
// 		break;
// 		case GLUT_KEY_DOWN:
// 		g_hairyness /= 1.05;
// 		cerr << "hairyness                                 = " << g_hairyness << std::endl;
// 		break;
// 	}
// 	glutPostRedisplay();
// }

// // New glut timer call back that perform dynamics simulation
// // every g_simulationsPerSecond times per second
// static void hairsSimulationCallback(int dontCare) {

// 	// TASK 2 TODO: wrte dynamics simulation code here as part of TASK2
// 	RigTForm acc                                        = inv(getPathAccumRbt(g_world, g_bunnyNode));

// 	for (int v                                          = 0; v<g_bunnyMesh.getNumVertices(); v++) {
    
// 		Cvec3 p                                            = Cvec3(acc * Cvec4(g_bunnyMesh.getVertex(v).getPosition(), 0));
// 		Cvec3 n                                            = g_bunnyMesh.getVertex(v).getNormal();

// 		Cvec3 s                                            = p + (n * g_furHeight);
        
// 		Cvec3 spring                                       = (s - g_tipPos[v]) * g_stiffness;
// 		Cvec3 force                                        = spring + g_gravity;

// 		g_tipPos[v]                                        = g_tipPos[v] + g_tipVelocity[v] * g_timeStep;

// 		g_tipPos[v]                                        = p + (g_tipPos[v]-p) * g_furHeight;
        
// 		g_tipVelocity[v]                                   = (g_tipVelocity[v] + force * g_timeStep) * g_damping;
// 	}
    

// 	//updateShellGeometry();
// 	g_shellNeedsUpdate                                  = true;

// 	// schedule this to get called again
// 	glutTimerFunc(1000/g_simulationsPerSecond, hairsSimulationCallback, 0);
// 	glutPostRedisplay(); // signal redisplaying
// }



// static void keyboard(const unsigned char key, const int x, const int y) {

// 	switch (key) {
// 		case 27:
// 		exit(0);                                  // ESC
// 		case 'h':
// 		cout << " ============== H E L P ==============\n\n"
// 			<< "h\t\thelp menu\n"
// 				<< "s\t\tsave screenshot\n"
// 					<< "f\t\tToggle flat shading on/off.\n"
// 						<< "o\t\tCycle object to edit\n"
// 							<< "v\t\tCycle view\n"
// 								<< "v\t\tCycle view\n"
// 									<< "drag left mouse to rotate\n" << endl;
// 		break;
// 		case 's':
// 		glFlush();
// 		writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
// 		break;
// 		// case 'f':
// 		//   g_activeShader ^= 1;
// 		//   break;
// 		case 'v':
// 		setCurrentView();
// 		break;
// 		// case 'o':
// 		//   setCurrentManipulatingObject();
// 		//   break;
// 		case 'm':
// 		setCurrentSkyView();
// 		break;
// 		case 'p':
// 		setPicking(true);
// 		break;
// 		case 'r':
// 		reset();
// 		break;
// 		case 'n':
// 		insertKeyFrame();
// 		break;
// 		case '+':
// 		g_msBetweenKeyFrames-=200;
// 		cout << "msBetweenKeyFrames is decreased to" << g_msBetweenKeyFrames <<"."<<endl;
// 		break;
// 		case '-':
// 		g_msBetweenKeyFrames+=200;
// 		cout << "msBetweenKeyFrames is increased to" << g_msBetweenKeyFrames <<"."<<endl;
// 		break;
// 		case 'y':
// 		animateTimerCallback(0);
// 		break;
// 		case 'u':
// 		updateCurrentKeyFrame();
// 		break;
// 		case 32:
// 		copyCurrentKeyFrameToSceneGraph();
// 		break;
// 		case 60:
// 		goToPreviousKeyFrame();
// 		break;
// 		case 62:
// 		goToNextKeyFrame();
// 		break;
// 		case 'd':
// 		deleteCurrentKeyFrame();
// 		break;
// 		case 'w':
// 		writeFramesToFile();
// 		break;
// 		case 'i':
// 		inputFramesFromFile();
// 		break;
// 		case 'b':
// 		setBubbling();
// 		break;
// 		case 'f':
// 		setSmoothShading();
// 		break;
// 		case '0':
// 		setSubDivisions(false);
// 		break;
// 		case '9':
// 		setSubDivisions(true);
// 		break;
// 		case '8':
// 		g_msBetweenBubblingFrames                          = g_msBetweenBubblingFrames/2;
// 		cout << "msBetweenBubblingFrames is halved to" << g_msBetweenBubblingFrames <<"."<<endl;
// 		break;
// 		case '7':
// 		g_msBetweenBubblingFrames                          = g_msBetweenBubblingFrames*2;
// 		cout << "msBetweenBubblingFrames is doubled to" << g_msBetweenBubblingFrames <<"."<<endl;
// 		break;
	
// 	}
// 	glutPostRedisplay();
// }


// static void initGlutState(int argc, char * argv[]) {
// 	glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
// 	glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
// 	glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
// 	glutCreateWindow("Assignment 5");                       // title the window

// 	glutDisplayFunc(display);                               // display rendering callback
// 	glutReshapeFunc(reshape);                               // window reshape callback
// 	glutMotionFunc(motion);                                 // mouse movement callback
// 	glutMouseFunc(mouse);                                   // mouse click callback
// 	glutKeyboardFunc(keyboard);
// 	glutSpecialFunc(specialKeyboard);                       
// }

// static void initGLState() {
// 	glClearColor(128./255., 200./255., 255./255., 0.);
// 	glClearDepth(0.);
// 	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
// 	glPixelStorei(GL_PACK_ALIGNMENT, 1);
// 	glCullFace(GL_BACK);
// 	glEnable(GL_CULL_FACE);
// 	glEnable(GL_DEPTH_TEST);
// 	glDepthFunc(GL_GREATER);
// 	glReadBuffer(GL_BACK);
// 	if (!g_Gl2Compatible)
// 		glEnable(GL_FRAMEBUFFER_SRGB);
// }

// // static void initShaders() {
// //   g_shaderStates.resize(g_numShaders);
// //   for (int i                                      = 0; i < g_numShaders; ++i) {
// //     if (g_Gl2Compatible)
// //       g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
// //     else
// //       g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
// //   }
// // }

// static void initBunnyMeshes() {
// 	g_bunnyMesh.load("bunny.mesh");

// 	// TODO: Init the per vertex normal of g_bunnyMesh, using codes from asst7  
// 	for (int i                                          = 0; i < g_bunnyMesh.getNumVertices(); ++i) {
// 		g_bunnyMesh.getVertex(i).setNormal(Cvec3(0));
// 	}

// 	for (int i                                          = 0; i < g_bunnyMesh.getNumFaces(); ++i) {
// 		Mesh::Face f                                       = g_bunnyMesh.getFace(i);
// 		for (int j                                         = 0; j < f.getNumVertices(); ++j) {
// 			f.getVertex(j).setNormal( f.getVertex(j).getNormal() + f.getNormal() );
// 		}
// 	}

// 	for (int i                                          = 0; i < g_bunnyMesh.getNumVertices(); ++i) {
// 		Cvec3 n                                            = g_bunnyMesh.getVertex(i).getNormal();
// 		if (norm2(n) > CS175_EPS2){
// 			g_bunnyMesh.getVertex(i).setNormal(normalize(n));
// 		}

  
// 	}
  
// 	// TODO: Initialize g_bunnyGeometry from g_bunnyMesh, similar to what you did for asst7 ...

// 	vector<VertexPN> vs;
// 	for (int i                                          = 0; i < g_bunnyMesh.getNumFaces(); ++i) {
// 		Mesh::Face f                                       = g_bunnyMesh.getFace(i);
// 		for (int j                                         = 0; j<f.getNumVertices(); j++) {
// 			vs.push_back(VertexPN(f.getVertex(j).getPosition(), f.getVertex(j).getNormal()));
// 		}
// 	}
// 	g_bunnyGeometry.reset(new SimpleGeometryPN());
// 	g_bunnyGeometry->upload(&vs[0], vs.size());

// 	// Now allocate array of SimpleGeometryPNX to for shells, one per layer
// 	g_bunnyShellGeometries.resize(g_numShells);
// 	for (int i                                          = 0; i < g_numShells; ++i) {
// 		g_bunnyShellGeometries[i].reset(new SimpleGeometryPNX());
// 	}
// }

// static void initMaterials() {
// 	// Create some prototype materials
// 	Material diffuse("./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader");
// 	Material solid("./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader");

// 	// copy diffuse prototype and set red color
// 	g_redDiffuseMat.reset(new Material(diffuse));
// 	g_redDiffuseMat->getUniforms().put("uColor", Cvec3f(1, 0, 0));

// 	// copy diffuse prototype and set blue color
// 	g_blueDiffuseMat.reset(new Material(diffuse));
// 	g_blueDiffuseMat->getUniforms().put("uColor", Cvec3f(0, 0, 1));

// 	// normal mapping material
// 	g_bumpFloorMat.reset(new Material("./shaders/normal-gl2.vshader", "./shaders/normal-gl2.fshader"));
// 	g_bumpFloorMat->getUniforms().put("uTexColor", shared_ptr<Texture>(new ImageTexture("Fieldstone.ppm", true)));
// 	g_bumpFloorMat->getUniforms().put("uTexNormal", shared_ptr<Texture>(new ImageTexture("FieldstoneNormal.ppm", false)));

// 	// copy solid prototype, and set to wireframed rendering
// 	g_arcballMat.reset(new Material(solid));
// 	g_arcballMat->getUniforms().put("uColor", Cvec3f(0.27f, 0.82f, 0.35f));
// 	g_arcballMat->getRenderStates().polygonMode(GL_FRONT_AND_BACK, GL_LINE);

// 	// copy solid prototype, and set to color white
// 	g_lightMat.reset(new Material(solid));
// 	g_lightMat->getUniforms().put("uColor", Cvec3f(1, 1, 1));

// 	// pick shader
// 	g_pickingMat.reset(new Material("./shaders/basic-gl2.vshader", "./shaders/pick-gl2.fshader"));

// 	g_meshMat.reset(new Material("./shaders/basic-gl2.vshader", "./shaders/specular-gl2.fshader"));
// 	g_meshMat->getUniforms().put("uColor", Cvec3f(0, 1, 0));

// 	// bunny material
// 	g_bunnyMat.reset(new Material("./shaders/basic-gl2.vshader", "./shaders/bunny-gl2.fshader"));
// 	g_bunnyMat->getUniforms()
// 		.put("uColorAmbient", Cvec3f(0.45f, 0.3f, 0.3f))
// 			.put("uColorDiffuse", Cvec3f(0.2f, 0.2f, 0.2f));

// 	// bunny shell materials;
// 	shared_ptr<ImageTexture> shellTexture(new ImageTexture("shell.ppm", false)); // common shell texture

// 	// needs to enable repeating of texture coordinates
// 	shellTexture->bind();
// 	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
// 	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
// 	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

// 	// eachy layer of the shell uses a different material, though the materials will share the
// 	// same shader files and some common uniforms. hence we create a prototype here, and will
// 	// copy from the prototype later
// 	Material bunnyShellMatPrototype("./shaders/bunny-shell-gl2.vshader", "./shaders/bunny-shell-gl2.fshader");
// 	bunnyShellMatPrototype.getUniforms().put("uTexShell", dynamic_pointer_cast<Texture>(shellTexture));
// 	bunnyShellMatPrototype.getRenderStates()
// 		.blendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA) // set blending mode
// 			.enable(GL_BLEND) // enable blending
// 				.disable(GL_CULL_FACE); // disable culling

// 	// allocate array of materials
// 	g_bunnyShellMats.resize(g_numShells);
// 	for (int i                                          = 0; i < g_numShells; ++i) {
// 		g_bunnyShellMats[i].reset(new Material(bunnyShellMatPrototype)); // copy from the prototype
// 		// but set a different exponent for blending transparency
// 		g_bunnyShellMats[i]->getUniforms().put("uAlphaExponent", 2.f + 5.f * float(i + 1)/g_numShells);
// 	}
// };



// static void initGeometry() {
// 	initGround();
// 	initCubes();
// 	initSphere();
// 	initBunnyMeshes();

// }

// static void constructRobot(shared_ptr<SgTransformNode> base, shared_ptr<Material> material){

// 	const double ARM_LEN                                = 0.7,
// 	ARM_THICK                                           = 0.25,
// 	TORSO_LEN                                           = 1.5,
// 	TORSO_THICK                                         = 0.25,
// 	TORSO_WIDTH                                         = 1,
// 	HEAD_RADIUS                                         = 0.35;
// 	const int NUM_JOINTS                                = 10,
// 	NUM_SHAPES                                          = 10;

// 	struct JointDesc {
// 		int parent;
// 		float x, y, z;
// 	};

// 	JointDesc jointDesc[NUM_JOINTS]                     = {
// 		{-1}, // torso
// 		{0,  TORSO_WIDTH/2, TORSO_LEN/2, 0}, // upper right arm
// 		{1,  ARM_LEN, 0, 0}, // lower right arm
// 		{0,  -TORSO_WIDTH/2, TORSO_LEN/2, 0}, // upper left arm
// 		{3, -ARM_LEN, 0, 0}, // lower left arm
// 		{0, 0, TORSO_LEN/2, 0}, // noggin
// 		{0, TORSO_WIDTH/2-ARM_THICK/2, -TORSO_LEN/2, 0}, // upper right leg
// 		{6, 0, -ARM_LEN, 0}, // lower right leg
// 		{0, -(TORSO_WIDTH/2-ARM_THICK/2), -TORSO_LEN/2, 0}, // upper left leg
// 		{8, 0, -ARM_LEN, 0}, // lower left leg
// 	};

// 	struct ShapeDesc {
// 		int parentJointId;
// 		float x, y, z, sx, sy, sz;
// 		shared_ptr<Geometry> geometry;
// 	};

// 	ShapeDesc shapeDesc[NUM_SHAPES]                     = {
// 		{0, 0,         0, 0, TORSO_WIDTH, TORSO_LEN, TORSO_THICK, g_cube}, // torso

// 		{1,  ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // upper right arm
// 		{2,  ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK * 0.7, ARM_THICK, g_cube}, // lower right arm
// 		{3, -ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK, ARM_THICK, g_cube}, // upper left arm
// 		{4, -ARM_LEN/2, 0, 0, ARM_LEN, ARM_THICK * 0.7, ARM_THICK, g_cube}, // lower left arm

// 		{5, 0,  HEAD_RADIUS, 0, HEAD_RADIUS, HEAD_RADIUS, HEAD_RADIUS, g_sphere}, // noggin

// 		{6, 0, -ARM_LEN/2, 0, ARM_THICK, ARM_LEN, ARM_THICK, g_cube}, // upper right leg
// 		{7, 0, -ARM_LEN/2, 0, ARM_THICK * 0.7, ARM_LEN, ARM_THICK, g_cube}, // lower right leg
// 		{8, 0, -ARM_LEN/2, 0, ARM_THICK, ARM_LEN, ARM_THICK, g_cube}, // upper left leg
// 		{9, 0, -ARM_LEN/2, 0, ARM_THICK * 0.7, ARM_LEN, ARM_THICK, g_cube}, // lower left leg
// 	};

// 	shared_ptr<SgTransformNode> jointNodes[NUM_JOINTS];

// 	for (int i                                          = 0; i < NUM_JOINTS; ++i) {
// 		if (jointDesc[i].parent == -1)
// 			jointNodes[i]                                     = base;
// 		else {
// 			jointNodes[i].reset(new SgRbtNode(RigTForm(Cvec3(jointDesc[i].x, jointDesc[i].y, jointDesc[i].z))));
// 			jointNodes[jointDesc[i].parent]->addChild(jointNodes[i]);
// 		}
// 	}
// 	// The new MyShapeNode takes in a material as opposed to color
// 	for (int i                                          = 0; i < NUM_SHAPES; ++i) {
// 		shared_ptr<SgGeometryShapeNode> shape(
// 			new MyShapeNode(shapeDesc[i].geometry,
// 		material, // USE MATERIAL as opposed to color
// 		Cvec3(shapeDesc[i].x, shapeDesc[i].y, shapeDesc[i].z),
// 		Cvec3(0, 0, 0),
// 		Cvec3(shapeDesc[i].sx, shapeDesc[i].sy, shapeDesc[i].sz)));
// 		jointNodes[shapeDesc[i].parentJointId]->addChild(shape);
// 	}
// }

// static void initScene() {
// 	g_world.reset(new SgRootNode());




// 	g_skyNode.reset(new SgRbtNode(RigTForm(Cvec3(0.0, 0.25, 4.0))));
// 	g_auxFrame                                          = linFact(g_skyNode->getRbt());
// 	g_currentPickedRbtNode                              = g_skyNode;
// 	g_currentView                                       = g_skyNode;

// 	g_groundNode.reset(new SgRbtNode());
// 	g_groundNode->addChild(shared_ptr<MyShapeNode>(
// 		new MyShapeNode(g_ground, g_bumpFloorMat, Cvec3(0, g_groundY, 0))));

// 	g_robot1Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 1, 0))));
// 	g_robot2Node.reset(new SgRbtNode(RigTForm(Cvec3(2, 1, 0))));

// 	g_light1Node.reset(new SgRbtNode(RigTForm(Cvec3(2.0, 3.0, 14.0))));
// 	g_light2Node.reset(new SgRbtNode(RigTForm(Cvec3(-2, 3.0, -5.0))));

// 	g_light1Node->addChild(shared_ptr<MyShapeNode>(
// 		new MyShapeNode(g_sphere, g_lightMat)));
// 	g_light2Node->addChild(shared_ptr<MyShapeNode>(
// 		new MyShapeNode(g_sphere, g_lightMat)));

// 	g_meshNode.reset(new SgRbtNode(RigTForm(Cvec3(0,0,-5))));
// 	g_meshNode->addChild(shared_ptr<MyShapeNode>(new MyShapeNode(g_meshGeometry, g_meshMat, Cvec3())));


// 	constructRobot(g_robot1Node, g_redDiffuseMat); // a Red robot
// 	constructRobot(g_robot2Node, g_blueDiffuseMat); // a Blue robot

// 	// create a single transform node for both the bunny and the bunny shells
// 	g_bunnyNode.reset(new SgRbtNode());

// 	// add bunny as a shape nodes
// 	g_bunnyNode->addChild(shared_ptr<MyShapeNode>(
// 		new MyShapeNode(g_bunnyGeometry, g_bunnyMat)));

// 	// add each shell as shape node
// 	for (int i                                          = 0; i < g_numShells; ++i) {
// 		g_bunnyNode->addChild(shared_ptr<MyShapeNode>(
// 			new MyShapeNode(g_bunnyShellGeometries[i], g_bunnyShellMats[i])));
// 	}
// 	// from this point, calling g_bunnyShellGeometries[i]->upload(...) will change the
// 	// geometry of the ith layer of shell that gets drawn

// 	g_world->addChild(g_skyNode);
// 	g_world->addChild(g_groundNode);
// 	g_world->addChild(g_robot1Node);
// 	g_world->addChild(g_robot2Node);
// 	g_world->addChild(g_light1Node);
// 	g_world->addChild(g_light2Node);
// 	g_world->addChild(g_meshNode);

// 	g_world->addChild(g_bunnyNode);

// 	dumpSgRbtNodes(g_world, g_rbts);
// }

// // New function that initialize the dynamics simulation
// static void initSimulation() {
// 	g_tipPos.resize(g_bunnyMesh.getNumVertices(), Cvec3(0));
// 	g_tipVelocity                                       = g_tipPos;

// 	// TASK 1 TODO: initialize g_tipPos to "at-rest" hair tips in world coordinates
// 	g_tipPos.clear();
// 	for(int f                                           = 0; f < g_bunnyMesh.getNumFaces(); f++) {
// 		for(int v                                          = 0; v < 3; v++) {
// 			Cvec3 p                                           = g_bunnyMesh.getFace(f).getVertex(v).getPosition();
// 			Cvec3 n                                           = g_bunnyMesh.getFace(f).getVertex(v).getNormal();
// 			g_tipPos.push_back(p + n * g_furHeight);
// 			g_tipVelocity.push_back(Cvec3(0,0,0));
// 		}
// 	}


// 	// Starts hair tip simulation
// 	hairsSimulationCallback(0);
// }


// int main(int argc, char * argv[]) {

// 	try {

// 		initGlutState(argc,argv);

// 		glewInit(); // load the OpenGL extensions

// 		cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.3") << endl;
// 		if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
// 			throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
// 		else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
// 			throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");
// 		initMesh();
// 		initGLState();
// 		//initShaders();
// 		initMaterials();
// 		initGeometry();
// 		initScene();
// 		initSimulation();

// 		glutMainLoop();

// 		return 0;
// 	}
// 	catch (const runtime_error& e) {
// 		cout << "Exception caught: " << e.what() << endl;
// 		return -1;
// 	}
// }
