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

#include "json.hpp"

#include <GLFW/glfw3.h>

#include <fstream>
#include <string>
#include <sstream>
#include <ctime>

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

cLabel* labelTime;

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

bool force_over_limit{ false };
bool max_trial_reached{ false };
bool max_time_reached{ false };
bool trial_started{ false };
std::int32_t lost_trial{0};

cPrecisionClock exec_time;
cPrecisionClock resp_time;

//---------------------------------------------------------------------------
// GEL
//---------------------------------------------------------------------------

// Define some constants
const std::int32_t kNumSurf{ 2 };
const std::int32_t kNumNodeX{ 12 };
const std::int32_t kNumNodeY{ 12 };
const std::int32_t kNumNodeZ{ 1 };

std::int32_t active_point_x{ 0 };
std::int32_t active_point_y{ 0 };
std::int32_t active_surface{ -1 };// -1 -> INVALID
std::int32_t multimodal_feedback{ 0 };

cShapeSphere* active_point_sphere;
const double kActivePointRadius{ 0.05 };

std::vector<std::vector<std::int32_t>> stimuli(0);

nlohmann::json config_file_json;
std::ifstream config_file("./config.json");
std::ofstream log_file;
std::ofstream response_file;
std::vector<double> max_force;
double max_time;
std::int32_t trial_limit;
// stiffness properties between the haptic device tool and the model (GEM)
std::vector<double> stiffness;

bool next_trial{ true };
std::int32_t trial_idx{ -1 };

double tot_reward{ 0.0 };
double reward{ 0.0 };
double lost_reward{ 0.0 };
double tot_lost_reward{ 0.0 };

// deformable world
cGELWorld* defWorld;

// object mesh
cGELMesh* defObject[kNumSurf];

// dynamic nodes
cGELSkeletonNode* nodes[kNumSurf][kNumNodeX][kNumNodeY][kNumNodeZ];

// haptic device model
cShapeSphere* device;
double deviceRadius;

const double kStartPosRadius{ 0.05 };

// haptic device model
cShapeSphere* device_start_pos;

// radius of the dynamic model sphere (GEM)
double radius;

const double kGEMSphereRadius{ 0.05 };
const double kGEMSide{ kGEMSphereRadius * kNumNodeX };

cMultiMesh* scalpel;

double r_camera{ 2.0 };
double camera_angle{ 0.785/2.0 };

const std::int32_t kNumCoins{ 10 };

cBitmap* coins_green[kNumCoins];
cBitmap* coins_red[kNumCoins];

double w_coin;
double h_coin;

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

std::vector< std::vector<std::int32_t>> LoadStimuli(const std::string filename = "./stimuli.csv");

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
                       double a_stiffness,
                       const cVector3d& a_cursorVel);


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

    stimuli = LoadStimuli("./stimuli.csv");

    config_file >> config_file_json;
    config_file.close();
    
    /////////
    std::string log_file_name;
    std::string resp_file_name;
    std::time_t now = time(0);
    std::tm* gmtm = std::gmtime(&now);
    std::string participant_name = config_file_json["name"].get<std::string>();
    std::string root_file_name = std::to_string(gmtm->tm_year) + std::to_string(gmtm->tm_mon) + std::to_string(gmtm->tm_mday);
    log_file_name  = root_file_name + participant_name + "_log.csv";
    resp_file_name = root_file_name + participant_name + "_response.csv";
    if (participant_name == "test") {
        log_file_name  = participant_name + "_log.csv";
        resp_file_name = participant_name + "_response.csv";
    }
    log_file.open(log_file_name);
    response_file.open(resp_file_name);

    max_force = config_file_json["force_limit"].get<std::vector<double>>();
    max_time = config_file_json["max_time"].get<double>();
    trial_limit = config_file_json["trial_limit"].get<std::int32_t>();
    stiffness = config_file_json["stiffness"].get<std::vector<double>>();

    log_file << "trial_idx,surface,x,y,multimodal,elap_time,force_x,force_y,force_z\n";
    response_file << "trial_idx,surface,x,y,multimodal,"
        << "elap_time,force_x,force_y,force_z,"
        << "reward,lost_reward,total_reward,total_lost_reward,"
        << "max_force,max_time,trial_limit,stiffness,"
        << "force_over_limit,max_time_reached,max_trial_reached\n";

    //-----------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //-----------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit()) {
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
    if (stereoMode == C_STEREO_ACTIVE) {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    } else {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window) {
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
    if (glewInit() != GLEW_OK) {
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
    camera->set(cVector3d(r_camera*std::cos(camera_angle), 0.0, r_camera * std::sin(camera_angle)),    // camera position (eye)
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
    light->setDir(-10.0, 0.0, 0.0); 

    scalpel = new cMultiMesh();
    world->addChild(scalpel);
    scalpel->loadFromFile(RESOURCE_PATH("../resources/surgeons_task/scalpel.stl"));
    scalpel->m_material->setGraySilver();
    scalpel->m_material->setShininess(100);
    scalpel->setLocalPos(0.0, 0.0, 0.0);
    scalpel->setShowFrame(false);
    scalpel->setUseCulling(true);

    active_point_sphere = new cShapeSphere(kActivePointRadius);
    world->addChild(active_point_sphere);
    active_point_sphere->m_material->setRedCrimson();
    active_point_sphere->m_material->setShininess(100);
    active_point_sphere->setLocalPos(0.0, 0.0, 0.0);
    active_point_sphere->setEnabled(false);
    //-----------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //-----------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    std::cout << hapticDeviceInfo.m_maxAngularDamping << ","
        << hapticDeviceInfo.m_maxAngularStiffness << ","
        << hapticDeviceInfo.m_maxAngularTorque << ","
        << hapticDeviceInfo.m_maxLinearDamping << ","
        << hapticDeviceInfo.m_maxLinearStiffness << ","
        << hapticDeviceInfo.m_maxLinearForce << ","
        << hapticDeviceInfo.m_workspaceRadius << "\n";

    // open connection to haptic device
    hapticDevice->open();

    // desired workspace radius of the cursor
    cursorWorkspaceRadius = 1.0;//0.7

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    workspaceScaleFactor = cursorWorkspaceRadius / hapticDeviceInfo.m_workspaceRadius;

    // define a scale factor between the force perceived at the cursor and the
    // forces actually sent to the haptic device
    deviceForceScale = 5.0;

    deviceRadius = 0.1;

    // create a large sphere that represents the haptic device
    device_start_pos = new cShapeSphere(kStartPosRadius);
    world->addChild(device_start_pos);
    device_start_pos->m_material->setGreenDark();
    device_start_pos->m_material->setShininess(100);
    device_start_pos->setLocalPos(0.0, 0.0, 0.5);

    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create a world which supports deformable object
    defWorld = new cGELWorld();
    world->addChild(defWorld);

    // set default properties for skeleton nodes
    cGELSkeletonNode::s_default_radius = kGEMSphereRadius;  // [m]
    cGELSkeletonNode::s_default_kDampingPos = 2.5;
    cGELSkeletonNode::s_default_kDampingRot = 0.6;
    cGELSkeletonNode::s_default_mass = 0.002; // [kg]
    cGELSkeletonNode::s_default_showFrame = true;
    cGELSkeletonNode::s_default_color.setBlueCornflower();
    cGELSkeletonNode::s_default_useGravity = false;
    cGELSkeletonNode::s_default_gravity.set(0.00, 0.00, 0.00);

    // set default physical properties for links
    cGELSkeletonLink::s_default_kSpringElongation = 25.0;  // [N/m]
    cGELSkeletonLink::s_default_kSpringFlexion = 0.5;   // [Nm/RAD]
    cGELSkeletonLink::s_default_kSpringTorsion = 0.1;   // [Nm/RAD]
    cGELSkeletonLink::s_default_color.setBlueCornflower();
    
    // create a deformable mesh
    for (std::int32_t i = 0; i < kNumSurf; ++i) {
        defObject[i] = new cGELMesh();
        defWorld->m_gelMeshes.push_front(defObject[i]);
        defObject[i]->setLocalPos(0.0, 0.0, 0.0);
        defObject[i]->setEnabled(false, true);//affect children

        // load model
        bool fileload;
        fileload = defObject[i]->loadFromFile(RESOURCE_PATH("../resources/models/box/box.obj"));
        if (!fileload) {
            cout << "Error - 3D Model failed to load correctly." << endl;
            close();
            return (-1);
        }

        // set some material color on the object
        cMaterial mat;
        mat.setWhite();
        mat.setShininess(100);
        defObject[i]->setMaterial(mat, true);

        // let's create a some environment mapping
        shared_ptr<cTexture2d> texture(new cTexture2d());
        if (i == 0) fileload = texture->loadFromFile(RESOURCE_PATH("../resources/images/bio_rev.jpg"));
        else        fileload = texture->loadFromFile(RESOURCE_PATH("../resources/images/water_rev.jpg"));

        if (!fileload) {
            cout << "Error - Texture failed to load correctly." << endl;
            close();
            return (-1);
        }

        // enable environmental texturing
        texture->setEnvironmentMode(GL_DECAL);
        texture->setSphericalMappingEnabled(true);

        defObject[i]->setTexture(texture, true);
        defObject[i]->setUseTexture(true, true);
        // set object to be transparent
        defObject[i]->setTransparencyLevel(0.65, true, true);
        // build dynamic vertices
        defObject[i]->buildVertices();
        // use internal skeleton as deformable model
        defObject[i]->m_useSkeletonModel = true;

        // create an array of nodes
        for (int z = 0; z < kNumNodeZ; z++) {
            for (int y = 0; y < kNumNodeY; y++) {
                for (int x = 0; x < kNumNodeX; x++) {
                    cGELSkeletonNode* newNode = new cGELSkeletonNode();
                    nodes[i][x][y][z] = newNode;
                    defObject[i]->m_nodes.push_front(newNode);
                    newNode->m_pos.set((-kGEMSide + 2.0 * kGEMSphereRadius * (double)x), (-kGEMSide + 2.0 * kGEMSphereRadius * (double)y), (2.0 * kGEMSphereRadius * (double)z));
                    newNode->m_kDampingPos = 0.1 * 0.25 * stiffness[i];
                    // set corner nodes as fixed
                    if ((x == 0 && y == 9) || (x == 9 && y == 0) || (x == 9 && y == 9) || (x == 0 && y == 0)) newNode->m_fixed = true;
                }
            }
        }
           
        // create links between nodes
        for (int z = 0; z < kNumNodeZ; z++) {
            for (int y = 0; y < kNumNodeY-1; y++) {
                for (int x = 0; x < kNumNodeX-1; x++) {
                    cGELSkeletonLink::s_default_kSpringElongation = 0.25 * stiffness[i];
                    cGELSkeletonLink* newLinkX0 = new cGELSkeletonLink(nodes[i][x + 0][y + 0][z], nodes[i][x + 1][y + 0][z]);
                    cGELSkeletonLink* newLinkX1 = new cGELSkeletonLink(nodes[i][x + 0][y + 1][z], nodes[i][x + 1][y + 1][z]);
                    cGELSkeletonLink* newLinkY0 = new cGELSkeletonLink(nodes[i][x + 0][y + 0][z], nodes[i][x + 0][y + 1][z]);
                    cGELSkeletonLink* newLinkY1 = new cGELSkeletonLink(nodes[i][x + 1][y + 0][z], nodes[i][x + 1][y + 1][z]);
                    defObject[i]->m_links.push_front(newLinkX0);
                    defObject[i]->m_links.push_front(newLinkX1);
                    defObject[i]->m_links.push_front(newLinkY0);
                    defObject[i]->m_links.push_front(newLinkY1);
                }
            }
        }
        //
        defObject[i]->scaleXYZ(2.0 * kGEMSide, 2.0 * kGEMSide, 1.0);
        defObject[i]->connectVerticesToSkeleton(false);
        defObject[i]->m_showSkeletonModel = false;
        defObject[i]->setEnabled(false, true);
    }

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();
    cFontPtr font_time = NEW_CFONTCALIBRI72();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);
    labelRates->m_fontColor.setBlack();

    labelTime = new cLabel(font_time);
    camera->m_frontLayer->addChild(labelTime);
    labelTime->m_fontColor.setBlack();
    
    for (std::int32_t i = 0; i < kNumCoins; ++i) {
        cBitmap* tmp_coin = new cBitmap();
        camera->m_frontLayer->addChild(tmp_coin);
        tmp_coin->loadFromFile(RESOURCE_PATH("../resources/surgeons_task/coin_green.png"));
        w_coin = tmp_coin->m_texture->m_image->getWidth();
        h_coin = tmp_coin->m_texture->m_image->getHeight();
        tmp_coin->setSize(0.08 * w_coin, 0.08 * h_coin);
        tmp_coin->setLocalPos(0.0, 0.0);
        tmp_coin->setEnabled(false);
        coins_green[i] = tmp_coin;
    }

    for (std::int32_t i = 0; i < kNumCoins; ++i) {
        cBitmap* tmp_coin = new cBitmap();
        camera->m_frontLayer->addChild(tmp_coin);
        tmp_coin->loadFromFile(RESOURCE_PATH("../resources/surgeons_task/red_coin.png"));
        w_coin = tmp_coin->m_texture->m_image->getWidth();
        h_coin = tmp_coin->m_texture->m_image->getHeight();
        tmp_coin->setSize(0.08 * w_coin, 0.08 * h_coin);
        tmp_coin->setLocalPos(0.0, 0.0);
        tmp_coin->setEnabled(false);
        coins_red[i] = tmp_coin;
    }

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
    // 
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

        if (next_trial) {
            trial_idx++;
            if (trial_idx >= stimuli.size()) glfwSetWindowShouldClose(window, GLFW_TRUE);
            else {
                active_surface      = stimuli[trial_idx][0]-1;
                active_point_x      = stimuli[trial_idx][1];
                active_point_y      = stimuli[trial_idx][2];
                multimodal_feedback = stimuli[trial_idx][3];
            }

            std::cout << "IDX: " << trial_idx << "\n";
            std::cout << "Active Surface: " << active_surface << "\n";
            std::cout << "Active Point X: " << active_point_x << "\n";
            std::cout << "Active Point Y: " << active_point_y << "\n";
            std::cout << "Multi-modal Feedback: " << multimodal_feedback << "\n\n";

            next_trial = false;
        }

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

std::vector< std::vector<std::int32_t>> LoadStimuli(const std::string filename) {
    std::ifstream stimuli_file{ filename };
    std::string data_line;
    std::string data;
    std::vector< std::vector<std::int32_t>> stimuli(0);

    if (stimuli_file.is_open()) {
        /* Discard stimuli file header */
        std::getline(stimuli_file, data_line);
        /* for each row */
        while (std::getline(stimuli_file, data_line)) {
            std::stringstream data_line_stream(data_line);
            /* for each row element */
            std::vector<std::int32_t> stimulus(0);
            while (std::getline(data_line_stream, data, ',')) {
                stimulus.push_back(std::stoi(data));
            }
            stimuli.push_back(stimulus);
        }
    }

   return stimuli;
}

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height) {
    // update window size
    width  = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description) {
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods) {
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT)) {
        return;
    } else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q)) {// option - exit
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    } else if (a_key == GLFW_KEY_S) {// option - show/hide skeleton
        if (active_surface != -1)
            defObject[active_surface]->m_showSkeletonModel = !defObject[active_surface]->m_showSkeletonModel;
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
    else if (a_key == GLFW_KEY_KP_ADD) {
        camera_angle += 0.05;
        double x_pos = r_camera * std::cos(camera_angle);
        double z_pos = r_camera * std::sin(camera_angle);
        camera->set(cVector3d(x_pos, 0.0, z_pos),    // camera position (eye)
                    cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                    cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

        std::cout << "Camera position: " << x_pos << ",0.0," << z_pos << "\n";
    }
    else if (a_key == GLFW_KEY_KP_SUBTRACT) {
        camera_angle -= 0.05;
        double x_pos = r_camera * std::cos(camera_angle);
        double z_pos = r_camera * std::sin(camera_angle);
        camera->set(cVector3d(x_pos, 0.0, z_pos),    // camera position (eye)
            cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
            cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

        std::cout << "Camera position: " << x_pos << ",0.0," << z_pos << "\n";
    }
    else if (a_key == GLFW_KEY_KP_MULTIPLY) {
        r_camera += 0.25;
        double x_pos = r_camera * std::cos(camera_angle);
        double z_pos = r_camera * std::sin(camera_angle);
        camera->set(cVector3d(x_pos, 0.0, z_pos),    // camera position (eye)
            cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
            cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

        std::cout << "Camera position: " << x_pos << ",0.0," << z_pos << "\n";
    }
    else if (a_key == GLFW_KEY_KP_DIVIDE) {
        r_camera -= 0.25;
        double x_pos = r_camera * std::cos(camera_angle);
        double z_pos = r_camera * std::sin(camera_angle);
        camera->set(cVector3d(x_pos, 0.0, z_pos),    // camera position (eye)
            cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
            cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

        std::cout << "Camera position: " << x_pos << ",0.0," << z_pos << "\n";
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

    log_file.close();
    response_file.close();

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

    if (trial_started) labelTime->setText(cStr(max_time - exec_time.getCurrentTimeSeconds(), 0) + "s");
    else labelTime->setText("--- s");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

    labelTime->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), height - 3 * labelRates->getHeight());

    for (std::int32_t i = 0; i < kNumCoins; ++i) {
        w_coin = coins_green[i]->getWidth();
        h_coin = coins_green[i]->getHeight();
        coins_green[i]->setLocalPos(width - 1.1 * w_coin, h_coin * 0.1 * (i + 1));
    }
    
    for (std::int32_t i = 0; i < kNumCoins; ++i) {
        w_coin = coins_red[i]->getWidth();
        h_coin = coins_red[i]->getHeight();
        coins_red[i]->setLocalPos(0.1 * w_coin, h_coin * 0.1 * (i + 1));
    }

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
    //bool iti;
   // const double kITI{ 2.0 };
    std::int32_t num_green_coin;
    std::int32_t num_red_coin;
   // cPrecisionClock iti_timer;
    cMatrix3d rot;
    // initialize precision clock
    cPrecisionClock clock;
    clock.reset();

    exec_time.reset();

    resp_time.reset();

    std::uint32_t user_switches{ 0 };
    std::uint32_t old_user_switches{ 0 };

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    force_over_limit = false;
    max_trial_reached = false;
    max_time_reached = false;
    trial_started = false;
    lost_trial = 0;
    num_green_coin = 0;
    num_red_coin = 0;
    //iti = false;

    // main haptic simulation loop
    while(simulationRunning) {   
        // stop clock
        double time = cMin(0.001, clock.stop());

        // restart clock
        clock.start(true);

        // read position from haptic device
        cVector3d pos;
        hapticDevice->getPosition(pos);
        hapticDevice->getRotation(rot);
        pos.mul(workspaceScaleFactor);
        //device->setLocalPos(pos);
        scalpel->setLocalPos(pos);
        scalpel->setLocalRot(rot);

        if (!next_trial && !trial_started) {
            if (device_start_pos->getLocalPos().distance(pos) <= kStartPosRadius) {
                device_start_pos->setEnabled(false, true);
                defObject[active_surface]->setEnabled(true, true);
                trial_started = true;
                force_over_limit = false;
                max_time_reached = false;
                std::cout << "TRIAL STARTED\n";
                exec_time.start(true);
            }
        }

        cVector3d vel;
        hapticDevice->getLinearVelocity(vel);

        // clear all external forces
        defWorld->clearExternalForces();

        // compute reaction forces
        cVector3d force(0.0, 0.0, 0.0);
 
        if (next_trial) {
            // integrate dynamics
            defWorld->updateDynamics(time);
            // send forces to haptic device
            hapticDevice->setForce(force);
            // signal frequency counter
            freqCounterHaptics.signal(1);
            continue;
        }

        for (int z = 0; z < kNumNodeZ; z++) {
            for (int y = 0; y < kNumNodeY; y++) {
                for (int x = 0; x < kNumNodeX; x++) {
                    cVector3d nodePos = nodes[active_surface][x][y][z]->m_pos;
                    cVector3d f = computeForce(pos, deviceRadius, nodePos, radius, stiffness[active_surface], vel);
                    cVector3d tmpfrc = -1.0 * f;
                    if (force_over_limit || max_time_reached || !trial_started) tmpfrc.zero();
                    nodes[active_surface][x][y][z]->setExternalForce(tmpfrc);
                    force.add(f);
                }
            }
        }

        // integrate dynamics
        defWorld->updateDynamics(time);

        // scale force
        force.mul(deviceForceScale / workspaceScaleFactor);
        
        hapticDevice->getUserSwitches(user_switches);

        if (!trial_started) old_user_switches = user_switches = 0;
        else old_user_switches = user_switches;
        
        std::cout << old_user_switches << "," << user_switches << "\r";

        if (trial_started && !force_over_limit && force.length() > max_force[active_surface]) {
            cMaterial mat;
            mat.setRedCrimson();
            mat.setShininess(100);
            defObject[active_surface]->setMaterial(mat, true);
            force_over_limit = true;
            user_switches = 1;
            std::cout << "FORCE OVER LIMIT\n";
        }

        if (trial_started && !max_time_reached && exec_time.getCurrentTimeSeconds() > max_time) {
          max_time_reached = true;
          user_switches = 1;
          std::cout << "MAX TIME REACHED\n";
        }

        if (trial_started && user_switches == 1) {
            /****/
            cMaterial mat;
            mat.setWhite();
            mat.setShininess(100);
            /****/
            defObject[active_surface]->setEnabled(false, true);
            defObject[active_surface]->setMaterial(mat, true);
            /****/
            if (!force_over_limit && !max_time_reached) {
                reward = fabs(pos.z());
                lost_reward = 0.0;
                coins_green[num_green_coin++]->setEnabled(true);
            } else {
                lost_trial++;
                reward = 0;
                lost_reward = fabs(pos.z());
                coins_red[num_red_coin++]->setEnabled(true);
            }

            if (lost_trial > trial_limit) {
              tot_reward = tot_lost_reward = reward = lost_reward = 0;
              num_green_coin = num_red_coin = 0;
              for (std::int32_t i = 0; i < kNumCoins; ++i) {
                  coins_green[i]->setEnabled(false);
              }
              lost_trial = 0;
              max_trial_reached = true;
            } else {
              tot_reward += reward;
              tot_lost_reward += lost_reward;
              max_trial_reached = false;
            }

            response_file << trial_idx << ","
                << active_surface << ","
                << active_point_x << ","
                << active_point_y << ","
                << multimodal_feedback << ","
                << exec_time.stop() << ","
                << force.x() << ","
                << force.y() << ","
                << force.z() << ","
                << reward << ","
                << lost_reward << ","
                << tot_reward << ","
                << tot_lost_reward << ","
                << max_force[active_surface] << ","
                << max_time << ","
                << trial_limit << ","
                << stiffness[active_surface] << ","
                << force_over_limit << ","
                << max_time_reached << ","
                << max_trial_reached << "\n";

            std::cout << "Total Reward: " << tot_reward << "\n"
                      << "Total LOST Reward: " << tot_lost_reward << "\n";
            trial_started = false;
            //iti = true;
            //iti_timer.start(true);
            device_start_pos->setEnabled(true, true);
            next_trial = true;
        }
        
        /*if (iti && iti_timer.getCurrentTimeSeconds() > kITI) {
            device_start_pos->setEnabled(true, true);
            next_trial = true;
            iti = false;
            iti_timer.stop();
        } else {*/
            if (trial_started) {
                log_file << trial_idx << ","
                    << active_surface << ","
                    << active_point_x << ","
                    << active_point_y << ","
                    << multimodal_feedback << ","
                    << exec_time.getCurrentTimeSeconds() << ","
                    << force.x() << ","
                    << force.y() << ","
                    << force.z() << "\n";
            }
        //}

        //if (multimodal_feedback == 0 || force_over_limit || max_time_reached || !trial_started || iti) force.zero();
        if (multimodal_feedback == 0 || force_over_limit || max_time_reached || !trial_started) force.zero();

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
                       double a_stiffness,
                       const cVector3d& a_cursorVel)
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

    //force = force + 

    // return result
    return (force);
}

//---------------------------------------------------------------------------
