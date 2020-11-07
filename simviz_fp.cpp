#include "Sai2Model.h"
#include "Sai2Graphics.h"
#include "Sai2Simulation.h"
#include <dynamics3d.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <GLFW/glfw3.h> //must be loaded after loading opengl/glew

#include "force_sensor/ForceSensorSim.h"  // references src folder in sai2-common directory
#include "force_sensor/ForceSensorDisplay.h"

#include <iostream>
#include <string>

#include <signal.h>
bool fSimulationRunning = false;
void sighandler(int){fSimulationRunning = false;}

using namespace std;
using namespace Eigen;

const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/panda_arm_hand.urdf";
const string robot_name = "panda_arm_hand";
const string leg_file = "./resources/human_leg.urdf"; // Added
const string leg_name = "human_leg"; // Added
const string leg_link_name = "link2";
const string camera_name = "camera_fixed";

// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
const std::string GRIPPER_JOINT_ANGLES_KEY = "sai2::cs225a::gripper::sensors::q";
const std::string GRIPPER_JOINT_VELOCITIES_KEY = "sai2::cs225a::gripper::sensors::dq";
const std::string LEG_JOINT_ANGLES_KEY = "sai2::cs225a::leg_robot::sensors::q";
const std::string LEG_JOINT_VELOCITIES_KEY = "sai2::cs225a::leg_robot::sensors::dq";
const std::string EE_FORCE_KEY = "cs225a::sensor::force";
const std::string EE_MOMENT_KEY = "cs225a::sensor::moment";
// - read
const std::string TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNNING_KEY = "sai2::cs225a::controller_running";

RedisClient redis_client;

// force sensor
ForceSensorSim* force_sensor;

// display widget for forces at end effector
ForceSensorDisplay* force_display;

// simulation function prototype
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* leg, Simulation::Sai2Simulation* sim);

// callback to print glfw errors
void glfwError(int error, const char* description);

// callback when a key is pressed
void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods);

// callback when a mouse button is pressed
void mouseClick(GLFWwindow* window, int button, int action, int mods);

// flags for scene camera movement
bool fTransXp = false;
bool fTransXn = false;
bool fTransYp = false;
bool fTransYn = false;
bool fTransZp = false;
bool fTransZn = false;
bool fRotPanTilt = false;

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// start redis client
	redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load graphics scene
	auto graphics = new Sai2Graphics::Sai2Graphics(world_file, true);
	Eigen::Vector3d camera_pos, camera_lookat, camera_vertical;
	graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
	graphics->showLinkFrame(true, leg_name, "link0", 0.15);
	graphics->showLinkFrame(true, robot_name, "link0", 0.15);
	graphics->showLinkFrame(true, robot_name, "link6", 0.15);
	graphics->showLinkFrame(true, robot_name, "link7", 0.15);
	graphics->_world->m_backgroundColor.setWhite();

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->updateKinematics();

	// load leg robot
	auto leg = new Sai2Model::Sai2Model(leg_file, false);
	leg->_q << -45*M_PI/180, 0*M_PI/180;  // fall
	// leg->_q << -90*M_PI/180, 0*M_PI/180;
	leg->updateKinematics();

	// load simulation world
	auto sim = new Simulation::Sai2Simulation(world_file, false);
	sim->setCollisionRestitution(0);
	// sim->setCoeffFrictionStatic(0.8);
	sim->setCoeffFrictionStatic(1.0);

	// read joint positions, velocities, update model
	sim->getJointPositions(robot_name, robot->_q);
	sim->getJointVelocities(robot_name, robot->_dq);
	// cout << "ROBOT Q BEFORE CHANGE: " << robot->_q << "\n";
	robot->_q[3] = -M_PI/6.0; // move EE up away from leg using link4
	robot->_q[7] = 0.09; // start with open gripper
	robot->_q[8] = -0.09; // start with open gripper
	sim->setJointPositions(robot_name, robot->_q);
	// cout << "ROBOT Q AFTER CHANGE: " << robot->_q << "\n";
	robot->updateKinematics();
	// cout << "ROBOT Q AFTER UPDATE: " << robot->_q << "\n";

	sim->setJointPositions(leg_name, leg->_q);
	sim->setJointVelocities(leg_name, leg->_dq);

	// initialize force sensor: needs Sai2Simulation sim interface type
	// MatrixXd force_sensor_tranform(4,4);
	// force_sensor_tranform = Eigen::Affine3d::Identity();
	// force_sensor_tranform[0][3] = 0.0;
	// force_sensor_tranform[1][3] = 0.0;
	// force_sensor_tranform[2][3] = 0.1; // this is the position of the force sensor
	force_sensor = new ForceSensorSim(robot_name, "link7", Eigen::Affine3d::Identity(), robot);
	force_display = new ForceSensorDisplay(force_sensor, graphics);

	/*------- Set up visualization -------*/
	// set up error callback
	glfwSetErrorCallback(glfwError);

	// initialize GLFW
	glfwInit();

	// retrieve resolution of computer display and position window accordingly
	GLFWmonitor* primary = glfwGetPrimaryMonitor();
	const GLFWvidmode* mode = glfwGetVideoMode(primary);

	// information about computer screen and GLUT display window
	int screenW = mode->width;
	int screenH = mode->height;
	int windowW = 0.8 * screenH;
	int windowH = 0.5 * screenH;
	int windowPosY = (screenH - windowH) / 2;
	int windowPosX = windowPosY;

	// create window and make it current
	glfwWindowHint(GLFW_VISIBLE, 0);
	GLFWwindow* window = glfwCreateWindow(windowW, windowH, "SAI2.0 - PandaApplications", NULL, NULL);
	glfwSetWindowPos(window, windowPosX, windowPosY);
	glfwShowWindow(window);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// set callbacks
	glfwSetKeyCallback(window, keySelect);
	glfwSetMouseButtonCallback(window, mouseClick);

	// cache variables
	double last_cursorx, last_cursory;

	redis_client.set(CONTROLLER_RUNNING_KEY, "0");
	fSimulationRunning = true;
	thread sim_thread(simulation, robot, leg, sim);

	// while window is open:
	while (fSimulationRunning)
	{

		// update graphics. this automatically waits for the correct amount of time
		int width, height;
		glfwGetFramebufferSize(window, &width, &height);
		graphics->updateGraphics(robot_name, robot);
		graphics->updateGraphics(leg_name, leg);
		graphics->render(camera_name, width, height);

		// swap buffers
		glfwSwapBuffers(window);

		// wait until all GL commands are completed
		glFinish();

		// check for any OpenGL errors
		GLenum err;
		err = glGetError();
		assert(err == GL_NO_ERROR);

		// poll for events
		glfwPollEvents();

		// move scene camera as required
		// graphics->getCameraPose(camera_name, camera_pos, camera_vertical, camera_lookat);
		Eigen::Vector3d cam_depth_axis;
		cam_depth_axis = camera_lookat - camera_pos;
		cam_depth_axis.normalize();
		Eigen::Vector3d cam_up_axis;
		// cam_up_axis = camera_vertical;
		// cam_up_axis.normalize();
		cam_up_axis << 0.0, 0.0, 1.0; //TODO: there might be a better way to do this
		Eigen::Vector3d cam_roll_axis = (camera_lookat - camera_pos).cross(cam_up_axis);
		cam_roll_axis.normalize();
		Eigen::Vector3d cam_lookat_axis = camera_lookat;
		cam_lookat_axis.normalize();
		if (fTransXp) {
			camera_pos = camera_pos + 0.05*cam_roll_axis;
			camera_lookat = camera_lookat + 0.05*cam_roll_axis;
		}
		if (fTransXn) {
			camera_pos = camera_pos - 0.05*cam_roll_axis;
			camera_lookat = camera_lookat - 0.05*cam_roll_axis;
		}
		if (fTransYp) {
			// camera_pos = camera_pos + 0.05*cam_lookat_axis;
			camera_pos = camera_pos + 0.05*cam_up_axis;
			camera_lookat = camera_lookat + 0.05*cam_up_axis;
		}
		if (fTransYn) {
			// camera_pos = camera_pos - 0.05*cam_lookat_axis;
			camera_pos = camera_pos - 0.05*cam_up_axis;
			camera_lookat = camera_lookat - 0.05*cam_up_axis;
		}
		if (fTransZp) {
			camera_pos = camera_pos + 0.1*cam_depth_axis;
			camera_lookat = camera_lookat + 0.1*cam_depth_axis;
		}
		if (fTransZn) {
			camera_pos = camera_pos - 0.1*cam_depth_axis;
			camera_lookat = camera_lookat - 0.1*cam_depth_axis;
		}
		if (fRotPanTilt) {
			// get current cursor position
			double cursorx, cursory;
			glfwGetCursorPos(window, &cursorx, &cursory);
			//TODO: might need to re-scale from screen units to physical units
			double compass = 0.006*(cursorx - last_cursorx);
			double azimuth = 0.006*(cursory - last_cursory);
			double radius = (camera_pos - camera_lookat).norm();
			Eigen::Matrix3d m_tilt; m_tilt = Eigen::AngleAxisd(azimuth, -cam_roll_axis);
			camera_pos = camera_lookat + m_tilt*(camera_pos - camera_lookat);
			Eigen::Matrix3d m_pan; m_pan = Eigen::AngleAxisd(compass, -cam_up_axis);
			camera_pos = camera_lookat + m_pan*(camera_pos - camera_lookat);
		}
		graphics->setCameraPose(camera_name, camera_pos, cam_up_axis, camera_lookat);
		glfwGetCursorPos(window, &last_cursorx, &last_cursory);
	}

	// stop simulation
	fSimulationRunning = false;
	sim_thread.join();

	// destroy context
	glfwSetWindowShouldClose(window,GL_TRUE);
	glfwDestroyWindow(window);

	// terminate
	glfwTerminate();

	return 0;
}

//------------------------------------------------------------------------------
void simulation(Sai2Model::Sai2Model* robot, Sai2Model::Sai2Model* leg, Simulation::Sai2Simulation* sim)
{
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	VectorXd gravity = VectorXd::Zero(dof);
	redis_client.setEigenMatrixJSON(TORQUES_COMMANDED_KEY, command_torques);

	// setup redis client data container for pipeset (batch write)
	std::vector<std::pair<std::string, std::string>> redis_data(2);  // set with the number of keys to write

	// leg dof
	int leg_dof = leg->dof();
	// sensed forces and moments from sensor
	Eigen::Vector3d sensed_force;
  Eigen::Vector3d sensed_moment;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	unsigned long long simulation_counter = 0;

	while (fSimulationRunning) {
		fTimerDidSleep = timer.waitForNextLoop();

		if(redis_client.get(CONTROLLER_RUNNING_KEY) == "1") {
			gravity.setZero();
		}
		else {
			robot->gravityVector(gravity);
		}

		// read arm torques from redis
		command_torques = redis_client.getEigenMatrixJSON(TORQUES_COMMANDED_KEY);

		// set torques to simulation
		sim->setJointTorques(robot_name, command_torques + gravity);

		// integrate forward
		// double curr_time = timer.elapsedTime();
		// double loop_dt = curr_time - last_time;
		// sim->integrate(loop_dt);
		sim->integrate(0.001);

		// read joint positions, velocities, update model
		sim->getJointPositions(robot_name, robot->_q);
		sim->getJointVelocities(robot_name, robot->_dq);
		robot->updateKinematics();

		sim->getJointPositions(leg_name, leg->_q);
		sim->getJointVelocities(leg_name, leg->_dq);
		leg->updateKinematics();

		// update force sensor readings
		force_sensor->update(sim);
		force_sensor->getForceLocalFrame(sensed_force);  // refer to ForceSensorSim.h in sai2-common/src/force_sensor (can also get wrt global frame)
    force_sensor->getMomentLocalFrame(sensed_moment);

		// write new robot state to redis
		redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY, robot->_q.head(7));
		redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq.head(7));
		redis_client.setEigenMatrixJSON(GRIPPER_JOINT_ANGLES_KEY, robot->_q.tail(2));
		redis_client.setEigenMatrixJSON(GRIPPER_JOINT_VELOCITIES_KEY, robot->_dq.tail(2));
		redis_client.setEigenMatrixJSON(LEG_JOINT_ANGLES_KEY, leg->_q);
		redis_client.setEigenMatrixJSON(LEG_JOINT_VELOCITIES_KEY, leg->_dq);
		redis_data.at(0) = std::pair<string, string>(EE_FORCE_KEY, redis_client.encodeEigenMatrixJSON(sensed_force));
		redis_data.at(1) = std::pair<string, string>(EE_MOMENT_KEY, redis_client.encodeEigenMatrixJSON(sensed_moment));

		redis_client.pipeset(redis_data);

		// update last time
		// last_time = curr_time;

		simulation_counter++;
	}

	double end_time = timer.elapsedTime();
	std::cout << "\n";
	std::cout << "Simulation Loop run time  : " << end_time << " seconds\n";
	std::cout << "Simulation Loop updates   : " << timer.elapsedCycles() << "\n";
	std::cout << "Simulation Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";
}

//------------------------------------------------------------------------------

void glfwError(int error, const char* description) {
	cerr << "GLFW Error: " << description << endl;
	exit(1);
}

//------------------------------------------------------------------------------

void keySelect(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	bool set = (action != GLFW_RELEASE);
	switch(key) {
		case GLFW_KEY_ESCAPE:
			// exit application
			fSimulationRunning = false;
			glfwSetWindowShouldClose(window,GL_TRUE);
			break;
		case GLFW_KEY_RIGHT:
			fTransXp = set;
			break;
		case GLFW_KEY_LEFT:
			fTransXn = set;
			break;
		case GLFW_KEY_UP:
			fTransYp = set;
			break;
		case GLFW_KEY_DOWN:
			fTransYn = set;
			break;
		case GLFW_KEY_A:
			fTransZp = set;
			break;
		case GLFW_KEY_Z:
			fTransZn = set;
			break;
		default:
			break;
	}
}

//------------------------------------------------------------------------------

void mouseClick(GLFWwindow* window, int button, int action, int mods) {
	bool set = (action != GLFW_RELEASE);
	//TODO: mouse interaction with robot
	switch (button) {
		// left click pans and tilts
		case GLFW_MOUSE_BUTTON_LEFT:
			fRotPanTilt = set;
			// NOTE: the code below is recommended but doesn't work well
			// if (fRotPanTilt) {
			// 	// lock cursor
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
			// } else {
			// 	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
			// }
			break;
		// if right click: don't handle. this is for menu selection
		case GLFW_MOUSE_BUTTON_RIGHT:
			//TODO: menu
			break;
		// if middle click: don't handle. doesn't work well on laptops
		case GLFW_MOUSE_BUTTON_MIDDLE:
			break;
		default:
			break;
	}
}
