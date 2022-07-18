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
#include <windows.h>

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


bool force_over_limit_{ false };
bool max_trial_reached_{ false };
bool max_time_reached_{ false };
bool trial_started_{ false };
std::int32_t lost_trial_{ 0 };
cPrecisionClock exec_timer_;
double start_time_;
double current_time_;
const std::int16_t kNumInst_{ 2 };
std::int16_t show_inst_cnt_{ 0 };
cLabel* label_inst_[kNumInst_];
cLevel* level_time_;
const std::int32_t kNumSurf_{ 2 };
const std::int32_t kNumNodeX_{ 12 };
const std::int32_t kNumNodeY_{ kNumNodeX_ };
const std::int32_t kNumNodeZ_{ 1 };
std::int32_t active_point_x_{ 0 };
std::int32_t active_point_y_{ 0 };
std::int32_t active_surface_{ -1 };// -1 -> INVALID
std::int32_t multimodal_feedback_{ 0 };
cShapeSphere* active_point_;
const double kGEMSphereRadius_{ 0.05 };
const double kScalpelStartPosRadius_{ 0.05 };
const double kActivePointRadius_{ 0.05 };
const double kScalpelHPRadius_{ 0.005 };
const cMatrix3d kNinePointsStencilActivePointRewardGain_{ cVector3d(4.0, 2.0, 4.0), cVector3d(2.0, 1.0, 2.0), cVector3d(4.0, 2.0, 4.0) };
const cMatrix3d kFivePointsStencilActivePointRewardGain_{ cVector3d(2.0, 0.0, 2.0), cVector3d(0.0, 1.0, 0.0), cVector3d(2.0, 0.0, 2.0) };
cMatrix3d active_point_reward_gain_;
std::vector<std::vector<std::int32_t>> stimuli_(0);
nlohmann::json config_file_json_;
std::ifstream config_file_("./config.json");
std::ofstream log_file_;
std::ofstream response_file_;
std::vector<double> max_force_;
double max_time_;
std::int32_t trial_limit_;
std::vector<double> stiffness_;
bool next_trial_{ true };
std::int32_t trial_idx_{ -1 };
double tot_reward_{ 0.0 };
double reward_{ 0.0 };
double lost_reward_{ 0.0 };
double tot_lost_reward_{ 0.0 };
cGELMesh* def_surf_[kNumSurf_];
cMultiMesh* scalpel_;
cShapeSphere* scalpel_hp_;
//cShapeEllipsoid* scalpel_start_pos_;
cShapeSphere* scalpel_start_pos_;
double camera_radius_{ 2.0 };
double camera_angle_{ 0.785/2.0 };
cBitmap* coin_green_;
cBitmap* coin_red_;
cLabel* label_reward_;
cLabel* label_lost_reward_;
double time_to_level_;
cLabel* label_finished_;
cLabel* label_is_training_;
std::int32_t stencil_points_;
std::int32_t is_training_;
std::int32_t cnt_training_;

//---------------------------------------------------------------------------
// GEL
//---------------------------------------------------------------------------
// deformable world
cGELWorld* defWorld;
// dynamic nodes
cGELSkeletonNode* nodes[kNumSurf_][kNumNodeX_][kNumNodeY_][kNumNodeZ_];

//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

std::vector< std::vector<std::int32_t>> LoadStimuli(const std::string filename = "./stimuli.csv");

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback to handle mouse click
void mouseButtonCallback(GLFWwindow* a_window, int a_button, int a_action, int a_mods);

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

    config_file_ >> config_file_json_;
    config_file_.close();
    
    /////////
    std::string log_file_name;
    std::string resp_file_name;
    std::time_t now = time(0);
    std::tm* gmtm = std::gmtime(&now);
    std::string participant_name = config_file_json_["name"].get<std::string>();
    //std::string root_file_name = std::to_string(gmtm->tm_year) + std::to_string(gmtm->tm_mon) + std::to_string(gmtm->tm_mday);
    std::string root_file_name; // empty string
    log_file_name  = root_file_name + participant_name + "_log.csv";
    resp_file_name = root_file_name + participant_name + "_response.csv";
    if (participant_name == "test") {
        log_file_name  = participant_name + "_log.csv";
        resp_file_name = participant_name + "_response.csv";
    }
    log_file_.open(log_file_name);
    response_file_.open(resp_file_name);

    max_force_ = config_file_json_["force_limit"].get<std::vector<double>>();
    max_time_ = config_file_json_["max_time"].get<double>();
    trial_limit_ = config_file_json_["trial_limit"].get<std::int32_t>();
    stiffness_ = config_file_json_["stiffness"].get<std::vector<double>>();
    stencil_points_ = config_file_json_["stencil_points"].get<std::int32_t>();

    if (stencil_points_ == 5) active_point_reward_gain_ = kFivePointsStencilActivePointRewardGain_;
    else                      active_point_reward_gain_ = kNinePointsStencilActivePointRewardGain_;

    log_file_ << "trial_idx,surface_id,active_region_x,active_region_y,multimodal,stiffness,max_time,max_trial,max_force,"
              << "elap_time,scalpel_x,scalpel_y,scalpel_z,active_point_start_pos_x,active_point_start_pos_y,active_point_start_pos_z,active_point_x,active_point_y,active_point_z,"
              << "active_point_def_x,active_point_def_y,active_point_def_z,"
              << "force_x,force_y,force_z,"
              << "scalpel_vel_x,scalpel_vel_y,scalepl_vel_z\n";

    response_file_  << "trial_idx,surface_id,active_region_x,active_region_y,multimodal,stiffness,max_time,max_trial,max_force,"
                    << "elap_time,scalpel_x,scalpel_y,scalpel_z,active_point_gain,active_point_x,active_point_y,active_point_z,"
                    << "active_point_def_x,active_point_def_y,active_point_def_z,"
                    << "force_x,force_y,force_z,"
                    << "reward,lost_reward,total_reward,total_lost_reward,"
                    << "force_over_limit,max_time_reached,max_trial_reached\n";

    show_inst_cnt_ = 0;

    system("rscript ./stimuli_gen.R");

    stimuli_ = LoadStimuli("./stimuli.csv");

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

    // set mouse button callback
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

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
    camera->set(cVector3d(camera_radius_ * std::cos(camera_angle_), 0.0, camera_radius_ * std::sin(camera_angle_)),    // camera position (eye)
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

    scalpel_ = new cMultiMesh();
    world->addChild(scalpel_);
    scalpel_->loadFromFile(RESOURCE_PATH("../resources/surgeons_task/scalpel.stl"));
    scalpel_->setUseMaterial(true);
    scalpel_->m_material->setGraySilver();
    scalpel_->m_material->setShininess(100);
    scalpel_->setLocalPos(0.0, 0.0, 0.0);
    scalpel_->setShowFrame(false);
    scalpel_->setUseCulling(true);
    scalpel_->computeBoundaryBox();
    scalpel_->setShowBoundaryBox(false);    
    scalpel_->setEnabled(false);

    scalpel_hp_ = new cShapeSphere(kScalpelHPRadius_);
    scalpel_->addChild(scalpel_hp_);
    scalpel_hp_->m_material->setRedLightCoral();
    scalpel_hp_->m_material->setShininess(100);
    scalpel_hp_->setUseTransparency(true);
    scalpel_hp_->setTransparencyLevel(0.5);
    scalpel_hp_->setLocalPos(0.0, 0.0, 0.0);
    scalpel_hp_->setEnabled(false);

    active_point_ = new cShapeSphere(kActivePointRadius_);
    active_point_->m_material->setRedCrimson();
    active_point_->m_material->setShininess(100);
    active_point_->setLocalPos(0.0, 0.0, 0.0);
    active_point_->setEnabled(false);

    coin_green_ = new cBitmap();
    camera->m_frontLayer->addChild(coin_green_);
    coin_green_->loadFromFile(RESOURCE_PATH("../resources/surgeons_task/coin_green.png"));
    coin_green_->setSize(0.8 * coin_green_->m_texture->m_image->getWidth(), 0.8 * coin_green_->m_texture->m_image->getHeight());
    coin_green_->setLocalPos(0.0, 0.0);
    coin_green_->setEnabled(false);

    coin_red_ = new cBitmap();
    camera->m_frontLayer->addChild(coin_red_);
    coin_red_->loadFromFile(RESOURCE_PATH("../resources/surgeons_task/red_coin.png"));
    coin_red_->setSize(0.8 * coin_red_->m_texture->m_image->getWidth(), 0.8 * coin_red_->m_texture->m_image->getHeight());
    coin_red_->setLocalPos(0.0, 0.0);
    coin_red_->setEnabled(false);

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
    cursorWorkspaceRadius = 1.0;//0.7

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    workspaceScaleFactor = cursorWorkspaceRadius / hapticDeviceInfo.m_workspaceRadius;

    // define a scale factor between the force perceived at the cursor and the
    // forces actually sent to the haptic device
    deviceForceScale = 5.0;

    // create a large sphere that represents the haptic device
    //scalpel_start_pos_ = new cShapeEllipsoid(kScalpelStartPosRadius_, kScalpelStartPosRadius_, kScalpelStartPosRadius_ / 1.5);
    scalpel_start_pos_ = new cShapeSphere(kScalpelStartPosRadius_);
    world->addChild(scalpel_start_pos_);
    scalpel_start_pos_->m_material->setGreenLime();
    scalpel_start_pos_->m_material->setShininess(100);
    scalpel_start_pos_->setLocalPos(0.0, 0.0, 0.5);
    //cMatrix3d scalpel_start_pos_rot;
    //scalpel_start_pos_rot.setAxisAngleRotationDeg(cVector3d(0, 1, 0), 45);
    scalpel_start_pos_->setUseTransparency(true);
    scalpel_start_pos_->setTransparencyLevel(0.4);
    //scalpel_start_pos_->setLocalRot(scalpel_start_pos_rot);
    scalpel_start_pos_->setEnabled(false);

    //-----------------------------------------------------------------------
    // COMPOSE THE VIRTUAL SCENE
    //-----------------------------------------------------------------------

    // create a world which supports deformable object
    defWorld = new cGELWorld();
    world->addChild(defWorld);

    // set default properties for skeleton nodes
    cGELSkeletonNode::s_default_radius = kGEMSphereRadius_;  // [m]
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
    for (std::int32_t i = 0; i < kNumSurf_; ++i) {
        def_surf_[i] = new cGELMesh();
        def_surf_[i]->addChild(active_point_);
        defWorld->m_gelMeshes.push_front(def_surf_[i]);
        def_surf_[i]->setLocalPos(0.0, 0.0, 0.0);
        def_surf_[i]->setEnabled(false, true);//affect children

        // load model
        bool fileload;
        fileload = def_surf_[i]->loadFromFile(RESOURCE_PATH("../resources/models/box/box.obj"));
        if (!fileload) {
            cout << "Error - 3D Model failed to load correctly." << endl;
            close();
            return (-1);
        }

        // set some material color on the object
        cMaterial mat;
        mat.setWhite();
        mat.setShininess(100);
        def_surf_[i]->setMaterial(mat, true);

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

        def_surf_[i]->setTexture(texture);
        def_surf_[i]->setUseTexture(true, false);
        // set object to be transparent
        def_surf_[i]->setTransparencyLevel(0.65, true);
        // build dynamic vertices
        def_surf_[i]->buildVertices();
        // use internal skeleton as deformable model
        def_surf_[i]->m_useSkeletonModel = true;

        // create an array of nodes
        for (int z = 0; z < kNumNodeZ_; z++) {
            for (int y = 0; y < kNumNodeY_; y++) {
                for (int x = 0; x < kNumNodeX_; x++) {
                    cGELSkeletonNode* newNode = new cGELSkeletonNode();
                    nodes[i][x][y][z] = newNode;
                    def_surf_[i]->m_nodes.push_front(newNode);
                    newNode->m_pos.set( -kGEMSphereRadius_ * kNumNodeX_ + 2.0 * kGEMSphereRadius_ * (double)x,
                                        -kGEMSphereRadius_ * kNumNodeY_ + 2.0 * kGEMSphereRadius_ * (double)y,
                                        2.0 * kGEMSphereRadius_ * (double)z);
                    newNode->m_kDampingPos = 0.1 * 0.25 * stiffness_[i];
                    // set corner nodes as fixed
                    if ((x == 0 && y == kNumNodeY_ - 1) || 
                        (x == kNumNodeX_ - 1 && y == 0) || 
                        (x == kNumNodeX_ - 1 && y == kNumNodeY_ - 1) || 
                        (x == 0 && y == 0)) newNode->m_fixed = true;
                }
            }
        }
           
        // create links between nodes
        for (int z = 0; z < kNumNodeZ_; z++) {
            for (int y = 0; y < kNumNodeY_ - 1; y++) {
                for (int x = 0; x < kNumNodeX_ - 1; x++) {
                    cGELSkeletonLink::s_default_kSpringElongation = 0.25 * stiffness_[i];
                    cGELSkeletonLink* newLinkX0 = new cGELSkeletonLink(nodes[i][x + 0][y + 0][z], nodes[i][x + 1][y + 0][z]);
                    cGELSkeletonLink* newLinkX1 = new cGELSkeletonLink(nodes[i][x + 0][y + 1][z], nodes[i][x + 1][y + 1][z]);
                    cGELSkeletonLink* newLinkY0 = new cGELSkeletonLink(nodes[i][x + 0][y + 0][z], nodes[i][x + 0][y + 1][z]);
                    cGELSkeletonLink* newLinkY1 = new cGELSkeletonLink(nodes[i][x + 1][y + 0][z], nodes[i][x + 1][y + 1][z]);
                    def_surf_[i]->m_links.push_front(newLinkX0);
                    def_surf_[i]->m_links.push_front(newLinkX1);
                    def_surf_[i]->m_links.push_front(newLinkY0);
                    def_surf_[i]->m_links.push_front(newLinkY1);
                }
            }
        }
        //
        def_surf_[i]->scaleXYZ(2.0 * kGEMSphereRadius_ * kNumNodeX_, 2.0 * kGEMSphereRadius_ * kNumNodeY_, 1.0);
        def_surf_[i]->connectVerticesToSkeleton(false);
        def_surf_[i]->m_showSkeletonModel = false;
        def_surf_[i]->setEnabled(false, true);
    }

    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    cFontPtr font = NEW_CFONTCALIBRI20();
    cFontPtr font_inst = NEW_CFONTCALIBRI40();
    cFontPtr font_reward = NEW_CFONTCALIBRI72();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);
    labelRates->m_fontColor.setBlack();
    labelRates->setEnabled(false);

    for (std::int32_t i = 0; i < kNumInst_; ++i) {
      label_inst_[i] = new cLabel(font_inst);
      camera->m_frontLayer->addChild(label_inst_[i]);
      label_inst_[i]->m_fontColor.setBlack();
      label_inst_[i]->setColor(cColorf(0.0, 0.0, 0.0));
      label_inst_[i]->setEnabled(false);
    }
    
    std::string testo_inst = "Il test BAST (slaBs deformAtion riSk Task) ha lo scopo di misurare le sue capacita' di usare indizi visivi e / o tattili.\n\n";
    testo_inst += "Le verra' chiesto di deformare una serie di lastre gelatinose virtuali che si susseguono. La deformazione puo' essere ottenuta tramite\n";
    testo_inst += "un bisturi controllato attraverso il dispositivo (braccio robotico) posto di fronte a lei.\n";
    testo_inst += "Questo dispositivo puo' erogare o meno un feedback tattile, ovvero una forza che si oppone al movimento di deformazione della lastra.\n\n";
    testo_inst += "Col bisturi deve imprimere una pressione sulla lastra per deformarla.\n";
    testo_inst += "Piu' riuscira' a deformarla senza perforarla, maggiore sara' la ricompensa (monete verdi).\n";
    testo_inst += "Qualora la lastra venisse perforata, le verra' conteggiata una perdita (monete rosse).\n\n";
    testo_inst += "Le varie lastre hanno un punto di rottura diverso, percio' dovra' porre attenzione a quanta pressione esercitera' sulla lastra stessa.\n\n";
    testo_inst += "Il suo compito e' quello di guadagnare piu' monete verdi possibili, minimizzando l'accumulo di monete rosse.\n\n";
    testo_inst += "Prema il tasto sinistro del mouse per andare avanti";

    label_inst_[0]->setText(testo_inst);
    label_inst_[0]->setEnabled(true);

    testo_inst = "Per iniziare la singola prova portare il bisturi all'altezza della sfera verde, portarlo sulla lastra nel punto\n";
    testo_inst += "indicato dalla pallina bianca e iniziare a premere sulla lastra.\n\n";
    testo_inst += "Una volta ritenuto di aver raggiunto il massimo grado di deformazione, prema uno dei due tasti sul braccio robotico.\n";
    testo_inst += "Se invece lei oltrepassa il punto di rottura della lastra, o impiega piu' di 5 secondi per deformare la lastra senza premere uno dei tasti,\n";
    testo_inst += "la singola prova sara' considerata fallita, e le verranno assegnate le monete rosse.\n";
    testo_inst += "Quando fallisce la prova, sentira' un suono basso; quando vince, sentira' un suono acuto.\n";
    testo_inst += "Superata la soglia di 10 errori, il suo guadagno cumulato sara' totalmente perso e sentrira' un suono basso di durata maggiore,\n";
    testo_inst += "e dovra' ricominciare a guadagnare le monete verdi.\n\n";
    testo_inst += "Adesso iniziera' una breve fase di training che servira' a prendere dimestichezza con il compito stesso.\n";
    testo_inst += "Successivamente, iniziera' il compito vero e proprio.\n\n";
    testo_inst += "Prema il tasto sinistro del mouse per andare avanti";

    label_inst_[1]->setText(testo_inst);
    
    /****/
    label_reward_ = new cLabel(font_reward);
    camera->m_frontLayer->addChild(label_reward_);
    label_reward_->m_fontColor.setBlack();
    label_reward_->setColor(cColorf(0.0, 0.0, 0.0));
    label_reward_->setEnabled(false);
    label_reward_->setText("0.0");
    /****/
    label_lost_reward_ = new cLabel(font_reward);
    camera->m_frontLayer->addChild(label_lost_reward_);
    label_lost_reward_->m_fontColor.setBlack();
    label_lost_reward_->setColor(cColorf(0.0, 0.0, 0.0));
    label_lost_reward_->setEnabled(false);
    label_lost_reward_->setText("0.0");

    /****/
    level_time_ = new cLevel();
    level_time_->m_colorActive.setBlue();
    level_time_->m_colorInactive.setGrayDark();
    camera->m_frontLayer->addChild(level_time_);
    level_time_->setRange(0.0, 1.0);
    level_time_->setWidth(50);
    level_time_->setNumIncrements(200);
    level_time_->setSingleIncrementDisplay(false);
    level_time_->setTransparencyLevel(0.8);
    level_time_->setValue(0.0);
    level_time_->rotateWidgetAroundCenterDeg(90);
    level_time_->setEnabled(false);

    time_to_level_ = level_time_->getRangeMax() / max_time_;

    label_finished_ = new cLabel(font_reward);
    camera->m_frontLayer->addChild(label_finished_);
    label_finished_->m_fontColor.setBlack();
    label_finished_->setColor(cColorf(0.0, 0.0, 0.0));
    label_finished_->setEnabled(false);
    label_finished_->setText("ESPERIMENTO TERMINATO");

    label_is_training_ = new cLabel(font_reward);
    camera->m_frontLayer->addChild(label_is_training_);
    label_is_training_->m_fontColor.setBlack();
    label_is_training_->setColor(cColorf(0.0, 0.0, 0.0));
    label_is_training_->setEnabled(false);
    label_is_training_->setText("FASE DI TRAINING - STIMOLO " + std::to_string(cnt_training_) + "/8");

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

void mouseButtonCallback(GLFWwindow* a_window, int a_button, int a_action, int a_mods) {
    if (a_button == GLFW_MOUSE_BUTTON_LEFT && a_action == GLFW_PRESS) {
        switch (show_inst_cnt_) {
            case 0:
                label_inst_[show_inst_cnt_]->setEnabled(false);
                show_inst_cnt_++;
                label_inst_[show_inst_cnt_]->setEnabled(true);
                break;
            case 1:
                label_inst_[show_inst_cnt_]->setEnabled(false);
                scalpel_start_pos_->setEnabled(true);
                scalpel_->setEnabled(true);
                coin_green_->setEnabled(true);
                coin_red_->setEnabled(true);
                label_reward_->setEnabled(true);
                label_lost_reward_->setEnabled(true);
                show_inst_cnt_++;
                label_is_training_->setEnabled(true);
                break;
            default: break;
        }
    }
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
    } else if (a_key == GLFW_KEY_F) {
        // toggle state variable
        fullscreen = !fullscreen;
        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();
        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);
        // set fullscreen or window mode
        if (fullscreen) {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        } else {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    } else if (a_key == GLFW_KEY_KP_ADD) {
        camera_angle_ += 0.05;
        double x_pos = camera_radius_ * std::cos(camera_angle_);
        double z_pos = camera_radius_ * std::sin(camera_angle_);
        camera->set(cVector3d(x_pos, 0.0, z_pos),    // camera position (eye)
                    cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                    cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector
    }
    else if (a_key == GLFW_KEY_KP_SUBTRACT) {
        camera_angle_ -= 0.05;
        double x_pos = camera_radius_ * std::cos(camera_angle_);
        double z_pos = camera_radius_ * std::sin(camera_angle_);
        camera->set(cVector3d(x_pos, 0.0, z_pos),    // camera position (eye)
                    cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                    cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector
    }
    else if (a_key == GLFW_KEY_KP_MULTIPLY) {
        camera_radius_ += 0.25;
        double x_pos = camera_radius_ * std::cos(camera_angle_);
        double z_pos = camera_radius_ * std::sin(camera_angle_);
        camera->set(cVector3d(x_pos, 0.0, z_pos),    // camera position (eye)
                    cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                    cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector
    }
    else if (a_key == GLFW_KEY_KP_DIVIDE) {
        camera_radius_ -= 0.25;
        double x_pos = camera_radius_ * std::cos(camera_angle_);
        double z_pos = camera_radius_ * std::sin(camera_angle_);
        camera->set(cVector3d(x_pos, 0.0, z_pos),    // camera position (eye)
                    cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                    cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector
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

    if (show_inst_cnt_ < kNumInst_) {
      for (std::int32_t i = 0; i < kNumInst_; ++i) {
        label_inst_[i]->setLocalPos(20.0, (height - label_inst_[i]->getHeight()) / 2.0);
      }
    }

    double w_coin = coin_green_->getWidth();
    double h_coin = coin_green_->getHeight();
    coin_green_->setLocalPos(width - 1.1 * w_coin, 0.1 * h_coin);
    coin_red_->setLocalPos(0.1 * w_coin, 0.1 * h_coin);
    
    label_reward_->setText(std::to_string((std::int32_t) round(1000.0 * tot_reward_)));
    label_lost_reward_->setText(std::to_string((std::int32_t) round(1000.0 * tot_lost_reward_)));

    label_reward_->setLocalPos(width - w_coin - 1.5 * label_reward_->getWidth(), 0.8 * h_coin / 2.0);

    label_lost_reward_->setLocalPos(1.2 * w_coin, 0.8 * h_coin / 2.0);

    level_time_->setLocalPos(0.70 * width, 0.85 * height);
    
    double current_time = exec_timer_.getCurrentTimeSeconds() - start_time_;
    level_time_->setValue(time_to_level_ * current_time);

    label_finished_->setLocalPos((width - label_finished_->getWidth()) / 2, (height - label_finished_->getHeight()) / 2);

    label_is_training_->setLocalPos((width - label_is_training_->getWidth()) / 2, label_is_training_->getHeight());
    label_is_training_->setText("FASE DI TRAINING - STIMOLO " + std::to_string(cnt_training_) + "/8");

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
  std::uint32_t user_switches{ 0 };
  double time_clock;
  
  cVector3d haptic_device_pos;
  cMatrix3d haptic_device_rot;
  cVector3d haptic_device_vel;

  cVector3d scalpel_hp_pos;
  cVector3d scalpel_bb_min;

  cVector3d active_point_pos;
  cVector3d active_point_start_pos;
  
  cVector3d force;

  cVector3d node_pose;
  cVector3d computed_force;
  cVector3d node_force;

  cVector3d active_point_def;

  std::int32_t node_coord_x;
  std::int32_t node_coord_y;
  std::int32_t node_coord_z{ 0 };

  cVector3d active_point_col_gain;
  double active_point_gain;

  bool first_trial;

  // initialize precision clock
  cPrecisionClock clock;
  clock.reset();

  exec_timer_.reset();

  // simulation in now running
  simulationRunning  = true;
  simulationFinished = false;

  first_trial = true;
  cnt_training_ = 0;
  force_over_limit_     = false;
  max_trial_reached_    = false;
  max_time_reached_     = false;
  trial_started_        = false;
  tot_reward_           = 0.0;
  reward_               = 0.0;
  tot_lost_reward_      = 0.0;
  lost_reward_          = 0.0;
  lost_trial_           = 0;
  trial_idx_            = 0;
  start_time_ = 0;
  current_time_ = 0;
  active_surface_       = stimuli_[trial_idx_][0] - 1;
  active_point_x_       = stimuli_[trial_idx_][1];
  active_point_y_       = stimuli_[trial_idx_][2];
  multimodal_feedback_  = stimuli_[trial_idx_][3];
  is_training_          = stimuli_[trial_idx_][4];
  /***/
  cnt_training_++;
  /***/
  if (stencil_points_ != 5) active_point_->setLocalPos(8.0 * kGEMSphereRadius_ * active_point_x_, 8.0 * kGEMSphereRadius_ * active_point_y_, kGEMSphereRadius_);
  else active_point_->setLocalPos(6.0 * kGEMSphereRadius_ * active_point_x_, 6.0 * kGEMSphereRadius_ * active_point_y_, kGEMSphereRadius_);
  active_point_->computeGlobalPositionsFromRoot();
  /****/
  switch (active_point_x_) {
      case -1:
        node_coord_x = (stencil_points_ != 5) ? 2 : 3;
        break;
      case 0:
        node_coord_x = 6;
        break;
      case 1:
        node_coord_x = (stencil_points_ != 5) ? 10 : 9;
        break;
      default: break;
  }
  /***/
  switch (active_point_y_) {
      case -1:
        node_coord_y = (stencil_points_ != 5) ? 2 : 3;
        break;
      case 0:
        node_coord_y = 6;
        break;
      case 1:
        node_coord_y = (stencil_points_ != 5) ? 10 : 9;
        break;
      default: break;
  }
  /***/
  node_coord_z = 0;
  /***/
  active_point_start_pos = nodes[active_surface_][node_coord_x][node_coord_y][node_coord_z]->m_pos;

  // main haptic simulation loop
  while(simulationRunning) {   
    // stop clock
    time_clock = cMin(0.001, clock.stop());

    // restart clock
    clock.start(true);

    if (show_inst_cnt_ < kNumInst_) {
      // clear all external forces
      defWorld->clearExternalForces();
      // integrate dynamics
      defWorld->updateDynamics(time_clock);
      // send forces to haptic device
      force.zero();
      hapticDevice->setForce(force);
      // signal frequency counter
      freqCounterHaptics.signal(1);
      continue;
    }

    hapticDevice->getPosition(haptic_device_pos);
    hapticDevice->getRotation(haptic_device_rot);
    hapticDevice->getLinearVelocity(haptic_device_vel);

    haptic_device_pos.mul(workspaceScaleFactor);
    
    scalpel_->setLocalPos(haptic_device_pos);
    scalpel_->setLocalRot(haptic_device_rot);

    scalpel_bb_min = scalpel_->getBoundaryMin();
    scalpel_bb_min.z(0.01);
    scalpel_bb_min.y(-0.001);

    scalpel_hp_pos = scalpel_bb_min;
    scalpel_hp_->setLocalPos(scalpel_hp_pos);
    scalpel_hp_->computeGlobalPositionsFromRoot();

    active_point_->computeGlobalPositionsFromRoot();

    if (!trial_started_) {
      /***/
      if (scalpel_start_pos_->getEnabled() && fabs(scalpel_start_pos_->getLocalPos().z() - scalpel_hp_->getGlobalPos().z()) <= kScalpelStartPosRadius_) {
        scalpel_start_pos_->setEnabled(false);
        def_surf_[active_surface_]->setEnabled(true);
        active_point_->setEnabled(true);
      } 
      if (!scalpel_start_pos_->getEnabled() && fabs(active_point_->getGlobalPos().z() - scalpel_hp_->getGlobalPos().z()) <= kScalpelStartPosRadius_) {
        active_point_->setEnabled(false);
        trial_started_ = true;
        force_over_limit_ = false;
        max_time_reached_ = false;
        level_time_->setValue(level_time_->getRangeMin());
        level_time_->setEnabled(true);
        exec_timer_.start(true);
        start_time_ = exec_timer_.getCurrentTimeSeconds();
      }
      // clear all external forces
      defWorld->clearExternalForces();
      // integrate dynamics
      defWorld->updateDynamics(time_clock);
      // send forces to haptic device
      force.zero();
      hapticDevice->setForce(force);
      // signal frequency counter
      freqCounterHaptics.signal(1);
      continue;
    }
    
    // clear all external forces
    defWorld->clearExternalForces();

    for (std::int32_t z = 0; z < kNumNodeZ_; z++) {
      for (std::int32_t y = 0; y < kNumNodeY_; y++) {
        for (std::int32_t x = 0; x < kNumNodeX_; x++) {
          node_pose = nodes[active_surface_][x][y][z]->m_pos;
          computed_force = computeForce(scalpel_hp_->getGlobalPos(), kGEMSphereRadius_, node_pose, kGEMSphereRadius_, stiffness_[active_surface_], haptic_device_vel);
          node_force = -1.0 * computed_force;
          nodes[active_surface_][x][y][z]->setExternalForce(node_force);
          force.add(computed_force);
        }
      }
    }

    // integrate dynamics
    defWorld->updateDynamics(time_clock);
    // scale force
    force.mul(deviceForceScale / workspaceScaleFactor);
    
    hapticDevice->getUserSwitches(user_switches);

    if (force.length() > max_force_[active_surface_]) {
        force_over_limit_ = true;
        user_switches = 1;
    }

    current_time_ = exec_timer_.getCurrentTimeSeconds() - start_time_;
    if (current_time_ > max_time_) {
      max_time_reached_ = true;
      user_switches = 1;
    }

    if (user_switches == 1 || user_switches == 2) {
      /****/
      def_surf_[active_surface_]->setEnabled(false);
      /***/
      level_time_->setEnabled(false);
      /***/
      scalpel_hp_pos = scalpel_hp_->getGlobalPos();
      /***/
      switch (active_point_x_) {
        case -1:
          node_coord_x = (stencil_points_ != 5) ? 2 : 3;
          active_point_col_gain  = active_point_reward_gain_.getCol0();
          break;
        case 0:
          node_coord_x = 6;
          active_point_col_gain  = active_point_reward_gain_.getCol1();
          break;
        case 1:
          node_coord_x = (stencil_points_ != 5) ? 10 : 9;
          active_point_col_gain  = active_point_reward_gain_.getCol2();
          break;
        default: break;
      }
      /***/
      switch (active_point_y_) {
        case -1:
          node_coord_y = (stencil_points_ != 5) ? 2 : 3;
          active_point_gain = active_point_col_gain.x();
          break;
        case 0:
          node_coord_y = 6;
          active_point_gain = active_point_col_gain.y();
          break;
        case 1:
          node_coord_y = (stencil_points_ != 5) ? 10 : 9;
          active_point_gain = active_point_col_gain.z();
          break;
        default: break;
      }
      /***/
      node_coord_z = 0;
      /***/
      active_point_pos = nodes[active_surface_][node_coord_x][node_coord_y][node_coord_z]->m_pos;
      /***/
      active_point_def = active_point_start_pos - active_point_pos;
      /****/
      if (!force_over_limit_ && !max_time_reached_) {
        Beep(1500, 250);
        reward_ = active_point_gain * active_point_def.length();
        lost_reward_ = 0.0;
      } else {
        lost_trial_++;
        reward_ = 0;
        lost_reward_ = active_point_gain * active_point_def.length();
      }
      /****/
      if (lost_trial_ >= trial_limit_) {
        Beep(500, 500);
      } else {
          if (force_over_limit_ || max_time_reached_) {
              Beep(500, 250);
          }
      }
      /****/
      if (lost_trial_ >= trial_limit_) {
        tot_reward_ = reward_ = 0;
        tot_lost_reward_ += lost_reward_;
        lost_trial_ = 0;
        max_trial_reached_ = true;
      } else {
        tot_reward_ += reward_;
        tot_lost_reward_ += lost_reward_;
        max_trial_reached_ = false;
      }
      /***/
      if (is_training_ == 0) {
          current_time_ = exec_timer_.getCurrentTimeSeconds() - start_time_;
          /***/
          response_file_ << trial_idx_ - cnt_training_ + 1 << ","
              << active_surface_ + 1 << ","
              << active_point_x_ << ","
              << active_point_y_ << ","
              << multimodal_feedback_ << ","
              << stiffness_[active_surface_] << ","
              << max_time_ << ","
              << trial_limit_ << ","
              << max_force_[active_surface_] << ","
              << current_time_ << ","
              << scalpel_hp_pos.x() << ","
              << scalpel_hp_pos.y() << ","
              << scalpel_hp_pos.z() << ","
              << active_point_gain << ","
              << active_point_pos.x() << ","
              << active_point_pos.y() << ","
              << active_point_pos.z() << ","
              << active_point_def.x() << ","
              << active_point_def.y() << ","
              << active_point_def.z() << ","
              << force.x() << ","
              << force.y() << ","
              << force.z() << ","
              << reward_ << ","
              << lost_reward_ << ","
              << tot_reward_ << ","
              << tot_lost_reward_ << ","
              << force_over_limit_ << ","
              << max_time_reached_ << ","
              << max_trial_reached_ << "\n";
      }
      /****/
      trial_started_ = false;
      /***/
      trial_idx_++;
      if (trial_idx_ < (std::int32_t)stimuli_.size()) {
        active_surface_      = stimuli_[trial_idx_][0] - 1;
        active_point_x_      = stimuli_[trial_idx_][1];
        active_point_y_      = stimuli_[trial_idx_][2];
        multimodal_feedback_ = stimuli_[trial_idx_][3];
        is_training_ = stimuli_[trial_idx_][4];
        /****/
        if (is_training_ == 1) cnt_training_++;
        /***/
        if (first_trial && is_training_ == 0) {
            tot_reward_ = 0.0;
            tot_lost_reward_ = 0.0;
            lost_trial_ = 0;
            label_is_training_->setEnabled(false);
            first_trial = false;
        }
        /****/
        if (stencil_points_ != 5) active_point_->setLocalPos(8.0 * kGEMSphereRadius_ * active_point_x_, 8.0 * kGEMSphereRadius_ * active_point_y_, kGEMSphereRadius_);
        else active_point_->setLocalPos(6.0 * kGEMSphereRadius_ * active_point_x_, 6.0 * kGEMSphereRadius_ * active_point_y_, kGEMSphereRadius_);
        /***/
        switch (active_point_x_) {
          case -1:
            node_coord_x = (stencil_points_ != 5) ? 2 : 3;
            break;
          case 0:
            node_coord_x = 6;
            break;
          case 1:
            node_coord_x = (stencil_points_ != 5) ? 10 : 9;
            break;
          default: break;
        }
        /***/
        switch (active_point_y_) {
          case -1:
            node_coord_y = (stencil_points_ != 5) ? 2 : 3;
            break;
          case 0:
            node_coord_y = 6;
            break;
          case 1:
            node_coord_y = (stencil_points_ != 5) ? 10 : 9;
            break;
          default: break;
        }
        /***/
        node_coord_z = 0;
        /***/
        active_point_start_pos = nodes[active_surface_][node_coord_x][node_coord_y][node_coord_z]->m_pos;
        /***/
        scalpel_start_pos_->setEnabled(true);
      } else {
        scalpel_->setEnabled(false);
        simulationRunning = false;
        label_finished_->setEnabled(true);
      }
    }
    /****/
    if (trial_started_) {
      /***/
      scalpel_hp_pos = scalpel_hp_->getGlobalPos();
      /****/
      switch (active_point_x_) {
        case -1:
          node_coord_x = (stencil_points_ != 5) ? 2 : 3;
          break;
        case 0:
          node_coord_x = 6;
          break;
        case 1:
          node_coord_x = (stencil_points_ != 5) ? 10 : 9;
          break;
        default: break;
      }
      /***/
      switch (active_point_y_) {
        case -1:
          node_coord_y = (stencil_points_ != 5) ? 2 : 3;
          break;
        case 0:
          node_coord_y = 6;
          break;
        case 1:
          node_coord_y = (stencil_points_ != 5) ? 10 : 9;
          break;
        default: break;
      }
      /***/
      node_coord_z = 0;
      /***/
      active_point_pos = nodes[active_surface_][node_coord_x][node_coord_y][node_coord_z]->m_pos;
      /***/
      active_point_def = active_point_start_pos - active_point_pos;
      /***/
      if (is_training_ == 0) {
          current_time_ = exec_timer_.getCurrentTimeSeconds() - start_time_;
          /***/
          log_file_ << trial_idx_ - cnt_training_ + 1 << ","
              << active_surface_ + 1 << ","
              << active_point_x_ << ","
              << active_point_y_ << ","
              << multimodal_feedback_ << ","
              << stiffness_[active_surface_] << ","
              << max_time_ << ","
              << trial_limit_ << ","
              << max_force_[active_surface_] << ","
              << current_time_ << ","
              << scalpel_hp_pos.x() << ","
              << scalpel_hp_pos.y() << ","
              << scalpel_hp_pos.z() << ","
              << active_point_start_pos.x() << ","
              << active_point_start_pos.y() << ","
              << active_point_start_pos.z() << ","
              << active_point_pos.x() << ","
              << active_point_pos.y() << ","
              << active_point_pos.z() << ","
              << active_point_def.x() << ","
              << active_point_def.y() << ","
              << active_point_def.z() << ","
              << force.x() << ","
              << force.y() << ","
              << force.z() << ","
              << haptic_device_vel.x() << ","
              << haptic_device_vel.y() << ","
              << haptic_device_vel.z() << "\n";
      }
    }
    /****/
    if (multimodal_feedback_ == 0 || !trial_started_) force.zero();
    /****/
    hapticDevice->setForce(force);
    // signal frequency counter
    freqCounterHaptics.signal(1);
  }
  ////
  log_file_.close();
  response_file_.close();
  // exit haptics thread
  simulationFinished = true;
}

//---------------------------------------------------------------------------

cVector3d computeForce(const cVector3d& a_cursor,
                       double a_cursorRadius,
                       const cVector3d& a_spherePos,
                       double a_radius,
                       double a_stiffness,
                       const cVector3d& a_cursorVel) {
    // compute the reaction forces between the tool and the ith sphere.
    cVector3d force;
    cVector3d vSphereCursor = a_cursor - a_spherePos;

    force.zero();

    if (vSphereCursor.length() < 0.0000001) return (force);
    if (vSphereCursor.length() > (a_cursorRadius + a_radius)) return (force);

    // compute penetration distance between tool and surface of sphere
    double penetrationDistance = (a_cursorRadius + a_radius) - vSphereCursor.length();
    cVector3d forceDirection = cNormalize(vSphereCursor);
    force = cMul( penetrationDistance * a_stiffness, forceDirection);

    return (force);
}

//---------------------------------------------------------------------------
