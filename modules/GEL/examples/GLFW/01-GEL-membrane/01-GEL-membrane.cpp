//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2016, CHAI3D.
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.2.0 $Rev: 1907 $
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
#include "GEL3D.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// DECLARED VARIABLES
//---------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the objects in the world
cDirectionalLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a haptic device
cGenericHapticDevicePtr hapticDevice;

// force scale factor
double deviceForceScale;

// scale factor between the device workspace and cursor workspace
double workspaceScaleFactor;

// desired workspace radius of the virtual cursor
double cursorWorkspaceRadius;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;

// root resource path
string resourceRoot;

//---------------------------------------------------------------------------
// GEL
//---------------------------------------------------------------------------

// deformable world
cGELWorld* defWorld;

// object mesh
cGELMesh* defObject_l;
cGELMesh* defObject_c;
cGELMesh* defObject_r;

// Define some constants
std::int32_t kNumNodeX{ 10 };
std::int32_t kNumNodeY{ 10 };
std::int32_t kNumNodeZ{ 1 };

double kDefObjOffset{ 1.2 };

// dynamic nodes
cGELSkeletonNode* nodes_l[10][10][1];
cGELSkeletonNode* nodes_c[10][10][1];
cGELSkeletonNode* nodes_r[10][10][1];

// haptic device model
cShapeSphere* device;
double deviceRadius;

// radius of the dynamic model sphere (GEM)
double radius;

// stiffness properties between the haptic device tool and the model (GEM)
double stiffness_l;
double stiffness_c;
double stiffness_r;


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function closes the application
void close(void);

// compute forces between tool and environment
cVector3d computeForce(const cVector3d& a_cursor,
                       double a_cursorRadius,
                       const cVector3d& a_spherePos,
                       double a_radius,
                       double a_stiffness);


//---------------------------------------------------------------------------
// DECLARED MACROS
//---------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())


//===========================================================================
/*
    DEMO:    GEM_membrane.cpp

    This application illustrates the use of the GEM libraries to simulate
    deformable object. In this example we load a simple mesh object and
    build a dynamic skeleton composed of volumetric spheres and 3 dimensional
    springs which model torsion, flexion and elongation properties.
*/
//===========================================================================

int main(int argc, char* argv[])
{
    //-----------------------------------------------------------------------
    // INITIALIZATION
    //-----------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 50-GEL-membrane" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[s] - Show/Hide GEL Skeleton" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

    // parse first arg to try and locate resources
    resourceRoot = string(argv[0]).substr(0,string(argv[0]).find_last_of("/\\")+1);


    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 0.8 * mode->height;
    int h = 0.5 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set key callback
    glfwSetKeyCallback(window, keyCallback);

    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);

    // set current display context
    glfwMakeContextCurrent(window);

    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

    // initialize GLEW library
#ifdef GLEW_VERSION
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //-----------------------------------------------------------------------
    // 3D - SCENEGRAPH
    //-----------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(2.0, 0.0, 2.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.02);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // enable multi-pass rendering to handle transparent objects
    camera->setUseMultipassTransparency(true);

    // create a directional light source
    light = new cDirectionalLight(world);

    // insert light source inside world
    world->addChild(light);

    // enable light source
    light->setEnabled(true);

    // define direction of light beam
    light->setDir(0.0, 0.0,-1.0); 


    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // open connection to haptic device
    hapticDevice->open();

    // desired workspace radius of the cursor
    cursorWorkspaceRadius = 0.7;

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    workspaceScaleFactor = cursorWorkspaceRadius / hapticDeviceInfo.m_workspaceRadius;

    // define a scale factor between the force perceived at the cursor and the
    // forces actually sent to the haptic device
    deviceForceScale = 5.0;

    // create a large sphere that represents the haptic device
    deviceRadius = 0.1;
    device = new cShapeSphere(deviceRadius);
    world->addChild(device);
    device->m_material->setWhite();
    device->m_material->setShininess(100);

    // interaction stiffness between tool and deformable model 
    stiffness_l = 25;
    stiffness_c = 75;
    stiffness_r = 125;


    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create a world which supports deformable object
    defWorld = new cGELWorld();
    world->addChild(defWorld);

    // create a deformable mesh
    defObject_l = new cGELMesh();
    defWorld->m_gelMeshes.push_front(defObject_l);
    defObject_c = new cGELMesh();
    defWorld->m_gelMeshes.push_front(defObject_c);
    defObject_r = new cGELMesh();
    defWorld->m_gelMeshes.push_front(defObject_r);

    // set object position
    defObject_l->setLocalPos(0.0, -kDefObjOffset, 0.0);
    defObject_c->setLocalPos(0.0, 0.0, 0.0);
    defObject_r->setLocalPos(0.0, kDefObjOffset, 0.0);


    // load model
    bool fileload;
    fileload = defObject_l->loadFromFile(RESOURCE_PATH("../resources/models/box/box.obj"));
    if (!fileload) {
        cout << "Error - 3D Model failed to load correctly." << endl;
        close();
        return (-1);
    }
    fileload = defObject_c->loadFromFile(RESOURCE_PATH("../resources/models/box/box.obj"));
    if (!fileload) {
        cout << "Error - 3D Model failed to load correctly." << endl;
        close();
        return (-1);
    }
    fileload = defObject_r->loadFromFile(RESOURCE_PATH("../resources/models/box/box.obj"));
    if (!fileload) {
        cout << "Error - 3D Model failed to load correctly." << endl;
        close();
        return (-1);
    }

    // set some material color on the object
    cMaterial mat;
    mat.setWhite();
    mat.setShininess(100);
    defObject_l->setMaterial(mat, true);
    defObject_c->setMaterial(mat, true);
    defObject_r->setMaterial(mat, true);

    // let's create a some environment mapping
    shared_ptr<cTexture2d> texture_l(new cTexture2d());
    fileload = texture_l->loadFromFile(RESOURCE_PATH("../resources/images/bio.jpg"));
    if (!fileload) {
         cout << "Error - Texture failed to load correctly." << endl;
        close();
        return (-1);
    }
    shared_ptr<cTexture2d> texture_c(new cTexture2d());
    fileload = texture_c->loadFromFile(RESOURCE_PATH("../resources/images/shadow.jpg"));
    if (!fileload) {
        cout << "Error - Texture failed to load correctly." << endl;
        close();
        return (-1);
    }
    shared_ptr<cTexture2d> texture_r(new cTexture2d());
    fileload = texture_r->loadFromFile(RESOURCE_PATH("../resources/images/stone.jpg"));
    if (!fileload) {
        cout << "Error - Texture failed to load correctly." << endl;
        close();
        return (-1);
    }

    // enable environmental texturing
    texture_l->setEnvironmentMode(GL_DECAL);
    texture_l->setSphericalMappingEnabled(true);
    // enable environmental texturing
    texture_c->setEnvironmentMode(GL_DECAL);
    texture_c->setSphericalMappingEnabled(true);
    // enable environmental texturing
    texture_r->setEnvironmentMode(GL_DECAL);
    texture_r->setSphericalMappingEnabled(true);

    // assign and enable texturing
    defObject_l->setTexture(texture_l, true);
    defObject_l->setUseTexture(true, true);
    //
    defObject_c->setTexture(texture_c, true);
    defObject_c->setUseTexture(true, true);
    //
    defObject_r->setTexture(texture_r, true);
    defObject_r->setUseTexture(true, true);

    // set object to be transparent
    defObject_l->setTransparencyLevel(0.65, true, true);
    defObject_c->setTransparencyLevel(0.65, true, true);
    defObject_r->setTransparencyLevel(0.65, true, true);
    
    // build dynamic vertices
    defObject_l->buildVertices();
    defObject_c->buildVertices();
    defObject_r->buildVertices();

    // set default properties for skeleton nodes
    cGELSkeletonNode::s_default_radius        = 0.05;  // [m]
    cGELSkeletonNode::s_default_kDampingPos   = 2.5;
    cGELSkeletonNode::s_default_kDampingRot   = 0.6;
    cGELSkeletonNode::s_default_mass          = 0.002; // [kg]
    cGELSkeletonNode::s_default_showFrame     = true;
    cGELSkeletonNode::s_default_color.setBlueCornflower();
    cGELSkeletonNode::s_default_useGravity    = true;
    //cGELSkeletonNode::s_default_gravity.set(0.00, 0.00,-9.81);
    cGELSkeletonNode::s_default_gravity.set(0.00, 0.00, 0.00);
    radius = cGELSkeletonNode::s_default_radius;

    // use internal skeleton as deformable model
    defObject_l->m_useSkeletonModel = true;
    defObject_c->m_useSkeletonModel = true;
    defObject_r->m_useSkeletonModel = true;

    // create an array of nodes
    for (int z = 0; z < kNumNodeZ; z++) {
        for (int y = 0; y < kNumNodeY; y++) {
            for (int x = 0; x < kNumNodeX; x++) {
                cGELSkeletonNode* newNode_l = new cGELSkeletonNode();
                nodes_l[x][y][z] = newNode_l;
                defObject_l->m_nodes.push_front(newNode_l);
                newNode_l->m_pos.set((-0.45 + 2.0 * radius * (double)x), (-0.43 + 2.0 * radius * (double)y), (2.0 * radius * (double)z));
                newNode_l->m_kDampingPos = 0.5 * cGELSkeletonNode::s_default_kDampingPos;;
                //
                cGELSkeletonNode* newNode_c = new cGELSkeletonNode();
                nodes_c[x][y][z] = newNode_c;
                defObject_c->m_nodes.push_front(newNode_c);
                newNode_c->m_pos.set((-0.45 + 2.0 * radius * (double)x), (-0.43 + 2.0 * radius * (double)y), (2.0 * radius * (double)z));
                //
                cGELSkeletonNode* newNode_r = new cGELSkeletonNode();
                nodes_r[x][y][z] = newNode_r;
                defObject_r->m_nodes.push_front(newNode_r);
                newNode_r->m_pos.set((-0.45 + 2.0 * radius * (double)x), (-0.43 + 2.0 * radius * (double)y), (2.0 * radius * (double)z));
                newNode_r->m_kDampingPos = 2.0 * cGELSkeletonNode::s_default_kDampingPos;

                // set corner nodes as fixed
                if ((x == 0 && y == 9) || (x == 9 && y == 0) || (x == 9 && y == 9) || (x == 0 && y == 0)) {
                    newNode_l->m_fixed = true;
                    newNode_c->m_fixed = true;
                    newNode_r->m_fixed = true;
                }
            }
        }
    }

    // set default physical properties for links
    cGELSkeletonLink::s_default_kSpringElongation = 25.0;  // [N/m]
    cGELSkeletonLink::s_default_kSpringFlexion    = 0.5;   // [Nm/RAD]
    cGELSkeletonLink::s_default_kSpringTorsion    = 0.1;   // [Nm/RAD]
    cGELSkeletonLink::s_default_color.setBlueCornflower();

    // create links between nodes
    for (int z = 0; z < kNumNodeZ; z++) {
        for (int y = 0; y < kNumNodeY - 1; y++) {
            for (int x = 0; x < kNumNodeX - 1; x++) {
                cGELSkeletonLink::s_default_kSpringElongation = 0.5 * 25.0;
                cGELSkeletonLink* newLinkX0_l = new cGELSkeletonLink(nodes_l[x + 0][y + 0][z], nodes_l[x + 1][y + 0][z]);
                cGELSkeletonLink* newLinkX1_l = new cGELSkeletonLink(nodes_l[x + 0][y + 1][z], nodes_l[x + 1][y + 1][z]);
                cGELSkeletonLink* newLinkY0_l = new cGELSkeletonLink(nodes_l[x + 0][y + 0][z], nodes_l[x + 0][y + 1][z]);
                cGELSkeletonLink* newLinkY1_l = new cGELSkeletonLink(nodes_l[x + 1][y + 0][z], nodes_l[x + 1][y + 1][z]);
                defObject_l->m_links.push_front(newLinkX0_l);
                defObject_l->m_links.push_front(newLinkX1_l);
                defObject_l->m_links.push_front(newLinkY0_l);
                defObject_l->m_links.push_front(newLinkY1_l);
                //
                cGELSkeletonLink::s_default_kSpringElongation = 25.0;
                cGELSkeletonLink* newLinkX0_c = new cGELSkeletonLink(nodes_c[x + 0][y + 0][z], nodes_c[x + 1][y + 0][z]);
                cGELSkeletonLink* newLinkX1_c = new cGELSkeletonLink(nodes_c[x + 0][y + 1][z], nodes_c[x + 1][y + 1][z]);
                cGELSkeletonLink* newLinkY0_c = new cGELSkeletonLink(nodes_c[x + 0][y + 0][z], nodes_c[x + 0][y + 1][z]);
                cGELSkeletonLink* newLinkY1_c = new cGELSkeletonLink(nodes_c[x + 1][y + 0][z], nodes_c[x + 1][y + 1][z]);
                defObject_c->m_links.push_front(newLinkX0_c);
                defObject_c->m_links.push_front(newLinkX1_c);
                defObject_c->m_links.push_front(newLinkY0_c);
                defObject_c->m_links.push_front(newLinkY1_c);
                //
                cGELSkeletonLink::s_default_kSpringElongation = 4.0 * 25.0;
                cGELSkeletonLink* newLinkX0_r = new cGELSkeletonLink(nodes_r[x + 0][y + 0][z], nodes_r[x + 1][y + 0][z]);
                cGELSkeletonLink* newLinkX1_r = new cGELSkeletonLink(nodes_r[x + 0][y + 1][z], nodes_r[x + 1][y + 1][z]);
                cGELSkeletonLink* newLinkY0_r = new cGELSkeletonLink(nodes_r[x + 0][y + 0][z], nodes_r[x + 0][y + 1][z]);
                cGELSkeletonLink* newLinkY1_r = new cGELSkeletonLink(nodes_r[x + 1][y + 0][z], nodes_r[x + 1][y + 1][z]);
                defObject_r->m_links.push_front(newLinkX0_r);
                defObject_r->m_links.push_front(newLinkX1_r);
                defObject_r->m_links.push_front(newLinkY0_r);
                defObject_r->m_links.push_front(newLinkY1_r);
            }

        }
    }

    // connect skin (mesh) to skeleton (GEM)
    defObject_l->connectVerticesToSkeleton(false);
    defObject_c->connectVerticesToSkeleton(false);
    defObject_r->connectVerticesToSkeleton(false);

    // show/hide underlying dynamic skeleton model
    defObject_l->m_showSkeletonModel = false;
    defObject_c->m_showSkeletonModel = false;
    defObject_r->m_showSkeletonModel = false;


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);
    labelRates->m_fontColor.setBlack();

    // create a background
    cBackground* background = new cBackground();
    camera->m_backLayer->addChild(background);

    // set background properties
    background->setCornerColors(cColorf(1.00f, 1.00f, 1.00f),
                                cColorf(0.95f, 0.95f, 0.95f),
                                cColorf(0.85f, 0.85f, 0.85f),
                                cColorf(0.80f, 0.80f, 0.80f));


    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - show/hide skeleton
    else if (a_key == GLFW_KEY_S)
    {
        defObject_l->m_showSkeletonModel = !defObject_l->m_showSkeletonModel;
        defObject_c->m_showSkeletonModel = !defObject_c->m_showSkeletonModel;
        defObject_r->m_showSkeletonModel = !defObject_r->m_showSkeletonModel;
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    hapticDevice->close();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);


    /////////////////////////////////////////////////////////////////////
    // UPDATE DEFORMABLE MODELS
    /////////////////////////////////////////////////////////////////////

    // update skins deformable objects
    defWorld->updateSkins(true);


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err;
    err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // initialize precision clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        // stop clock
        double time = cMin(0.001, clock.stop());

        // restart clock
        clock.start(true);

        // read position from haptic device
        cVector3d pos;
        hapticDevice->getPosition(pos);
        pos.mul(workspaceScaleFactor);
        device->setLocalPos(pos);

        // clear all external forces
        defWorld->clearExternalForces();

        // compute reaction forces
        cVector3d force(0.0, 0.0, 0.0);
        for (int z = 0; z < kNumNodeZ; z++) {
            for (int y = 0; y < kNumNodeY; y++) {
                for (int x = 0; x < kNumNodeX; x++) {
                    //cVector3d nodePos = nodes_l[x][y][z]->m_pos;
                    cVector3d nodePos = nodes_l[x][y][z]->m_pos + cVector3d(0.0, -kDefObjOffset, 0.0);
                    cVector3d f = computeForce(pos, deviceRadius, nodePos, radius, stiffness_l);
                    cVector3d tmpfrc = -1.0 * f;
                    nodes_l[x][y][z]->setExternalForce(tmpfrc);
                    force.add(f);
                    //
                    nodePos = nodes_c[x][y][z]->m_pos;
                    f = computeForce(pos, deviceRadius, nodePos, radius, stiffness_c);
                    tmpfrc = -1.0 * f;
                    nodes_c[x][y][z]->setExternalForce(tmpfrc);
                    force.add(f);
                    //
                    //nodePos = nodes_r[x][y][z]->m_pos;
                    nodePos = nodes_r[x][y][z]->m_pos + cVector3d(0.0, kDefObjOffset, 0.0);
                    f = computeForce(pos, deviceRadius, nodePos, radius, stiffness_r);
                    tmpfrc = -1.0 * f;
                    nodes_r[x][y][z]->setExternalForce(tmpfrc);
                    force.add(f);
                }
            }
        }

        // integrate dynamics
        defWorld->updateDynamics(time);

        // scale force
        force.mul(deviceForceScale / workspaceScaleFactor);

        force += cVector3d(0.0, 0.0, 0.981);

        // DEBUG INFO
        std::cout << force << "\n";

        // send forces to haptic device
        hapticDevice->setForce(force);

        // signal frequency counter
        freqCounterHaptics.signal(1);
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

cVector3d computeForce(const cVector3d& a_cursor,
                       double a_cursorRadius,
                       const cVector3d& a_spherePos,
                       double a_radius,
                       double a_stiffness)
{
    // compute the reaction forces between the tool and the ith sphere.
    cVector3d force;
    force.zero();
    cVector3d vSphereCursor = a_cursor - a_spherePos;

    // check if both objects are intersecting
    if (vSphereCursor.length() < 0.0000001)
    {
        return (force);
    }

    if (vSphereCursor.length() > (a_cursorRadius + a_radius))
    {
        return (force);
    }

    // compute penetration distance between tool and surface of sphere
    double penetrationDistance = (a_cursorRadius + a_radius) - vSphereCursor.length();
    cVector3d forceDirection = cNormalize(vSphereCursor);
    force = cMul( penetrationDistance * a_stiffness, forceDirection);

    // return result
    return (force);
}

//---------------------------------------------------------------------------
