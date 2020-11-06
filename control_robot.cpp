// This example application loads a URDF world file and simulates two robots
// with physics and contact in a Dynamics3D virtual world. A graphics model of it is also shown using
// Chai3D.

#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include "Sai2Primitives.h"

#include <iostream>
#include <string>

#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
const string leg_file = "./resources/human_leg.urdf";

#define PRE_APPROACH_CONTROLLER	0
#define APPROACH_CONTROLLER			1
#define GRIP_CONTROLLER					2
#define TRAJECTORY_CONTROLLER		3

// redis keys:
// - read:
std::string JOINT_ANGLES_KEY;
std::string JOINT_VELOCITIES_KEY;
std::string GRIPPER_JOINT_ANGLES_KEY;
std::string GRIPPER_JOINT_VELOCITIES_KEY;
std::string JOINT_TORQUES_SENSED_KEY;
std::string LEG_JOINT_ANGLES_KEY;
// - write
std::string JOINT_TORQUES_COMMANDED_KEY;

// - model
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;
std::string ROBOT_GRAVITY_KEY;

unsigned long long controller_counter = 0;

const bool inertia_regularization = true;

// Leg and robot constants
const double l1 = 0.5;
const double l2 = 0.4;

int main() {

	JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
	LEG_JOINT_ANGLES_KEY = "sai2::cs225a::leg_robot::sensors::q"; // added
	GRIPPER_JOINT_ANGLES_KEY = "sai2::cs225a::gripper::sensors::q";
	GRIPPER_JOINT_VELOCITIES_KEY = "sai2::cs225a::gripper::sensors::dq";
	JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
	JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robot
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	cout << "Original initial_q = " << initial_q << "\n";
	initial_q[6] = -35.0*M_PI/180.0; // Rotate link7 to be perpendicular to camera
	robot->updateModel();

	// load gripper
	Vector2d gripper_pos, gripper_vel;
	gripper_pos = redis_client.getEigenMatrixJSON(GRIPPER_JOINT_ANGLES_KEY);
	gripper_vel = redis_client.getEigenMatrixJSON(GRIPPER_JOINT_VELOCITIES_KEY);

	// load human leg
	auto leg = new Sai2Model::Sai2Model(leg_file, false);
	leg->_q = redis_client.getEigenMatrixJSON(LEG_JOINT_ANGLES_KEY);
	VectorXd initial_leg_q = leg->_q;
	cout << "leg->_q = " << leg->_q << "\n";

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);
	Vector2d command_torques_hand = Vector2d::Zero();
	VectorXd command_torques_full = VectorXd::Zero(9); // make a vector of 9 torques to send to the robot with the hand
	MatrixXd N_prec = MatrixXd::Identity(dof, dof);

	// pose task
	const string control_link = "link7";
	const Vector3d control_point = Vector3d(0,0,0.15); //Vector3d(0,0,0.07);
	auto posori_task = new Sai2Primitives::PosOriTask(robot, control_link, control_point);

	// Set initial q1 and q2
	double q1 = initial_leg_q[0];
	double q2 = initial_leg_q[1];

	// task position
	Vector3d X;
	robot->position(X, control_link, control_point);

	// Frame transformation from the world to leg frame
	Vector3d T_world_leg;
	T_world_leg << 1.25, 0.0, 0.15;
	Matrix3d R_world_leg = Matrix3d::Identity();

	// // Frame transformation from the world to robot frame
	Vector3d T_world_robot;
	T_world_robot << 0.5, -0.4, 0.05;
	Matrix3d R_world_robot;
	R_world_robot << 0, -1, 0,
									 1, 0, 0,
									 0, 0, 1;


#ifdef USING_OTG
	posori_task->_use_interpolation_flag = true;
#else
	posori_task->_use_velocity_saturation_flag = true;
#endif

	VectorXd posori_task_torques = VectorXd::Zero(dof);
	posori_task->_kp_pos = 200.0;
	posori_task->_kv_pos = 20.0;
	posori_task->_kp_ori = 200.0;
	posori_task->_kv_ori = 20.0;

	// joint task
	auto joint_task = new Sai2Primitives::JointTask(robot);

#ifdef USING_OTG
	joint_task->_use_interpolation_flag = true;
#else
	joint_task->_use_velocity_saturation_flag = true;
#endif

	VectorXd joint_task_torques = VectorXd::Zero(dof);
	joint_task->_kp = 250.0;
	joint_task->_kv = 15.0;

	// Set initial q position to that which is specified in simviz
	VectorXd q_init_desired = initial_q;
	joint_task->_desired_position = q_init_desired;

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	int state = PRE_APPROACH_CONTROLLER;

	// Helper variables
	int count = 0;
	Matrix3d rot_desired = Matrix3d::Identity();

	// Pre-approach desired position and orientation
	// *** Set the desired position to be above the grip point ***
	// 1. Get X in leg frame
	double vertical_offset = 0.0;
	X(0) = -l1 * sin(q1) - l2 * sin(q1 + q2);
	X(1) = 0.0;
	X(2) = l1 * cos(q1) + l2 * cos(q1 + q2) + vertical_offset;
	// 2. Transform to world frame
	X = X - T_world_leg;
	X = R_world_leg.inverse() * X;
	// 3. Transform to robot base frame
	X = X + T_world_robot;
	X = R_world_robot * X;
	// *** Set the desired orientation to be perpendicular to the leg ***
	rot_desired = AngleAxisd(-(q1 + q2 - M_PI/2.0), Vector3d::UnitY()).toRotationMatrix();
	rot_desired = R_world_robot * (R_world_leg.inverse() * rot_desired);

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);

		// update model
		robot->updateModel();

		// read gripper state from redis
		gripper_pos = redis_client.getEigenMatrixJSON(GRIPPER_JOINT_ANGLES_KEY);
		gripper_vel = redis_client.getEigenMatrixJSON(GRIPPER_JOINT_VELOCITIES_KEY);

		if(state == PRE_APPROACH_CONTROLLER)
		{
			// update task model and set hierarchy
			N_prec.setIdentity();
			joint_task->updateTaskModel(N_prec);

			// compute torques
			joint_task->computeTorques(joint_task_torques);
			command_torques = joint_task_torques;

			// cout << "In the pre-approach controller\n";

			if ((robot->_q - q_init_desired).norm() < 0.15) {
				cout << "Reached initial robot position. Setting pre-approach posori desired.\n";

				posori_task->reInitializeTask();
				posori_task->_desired_position = X; // Updated X to approach position
				posori_task->_desired_orientation = rot_desired; // Updated to approach orientation

				// Go to next controller
				state = APPROACH_CONTROLLER;
			}

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques;

			// compute gripper torques
			Vector2d gripper_pos_des = Vector2d::Zero();
			gripper_pos_des(0) = 0.09;
			gripper_pos_des(1) = -0.09;
			command_torques_hand = -20.0 * (gripper_pos - gripper_pos_des) - 5.0 * gripper_vel;

		}

		else if (state == APPROACH_CONTROLLER) {
			// cout << "APPROACH_CONTROLLER\n";

			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			// Maybe don't need this
			// posori_task->reInitializeTask();
			posori_task->_desired_position = X; // Updated X to approach position
			posori_task->_desired_orientation = rot_desired; // Updated to approach orientation

			// Continue until you reach the approach position
			Vector3d pos;
			robot->position(pos, control_link, control_point);
			Matrix3d rot;
			robot->rotation(rot, control_link);
			if ((pos - X).norm() < 0.15 && (rot - rot_desired).norm() < 0.15) {
				cout << "Reached approach position.";
			}

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);
			command_torques = posori_task_torques + joint_task_torques;

			// compute gripper torques
			Vector2d gripper_pos_des = Vector2d::Zero();
			gripper_pos_des(0) = 0.09;
			gripper_pos_des(1) = -0.09;
			command_torques_hand = -20.0 * (gripper_pos - gripper_pos_des) - 5.0 * gripper_vel;
		}

		else if (state == GRIP_CONTROLLER) {
			cout << "GRIP_CONTROLLER\n";
		}

		else if(state == TRAJECTORY_CONTROLLER)
		{
			cout << "TRAJECTORY_CONTROLLER\n";
			// update task model and set hierarchy
			N_prec.setIdentity();
			posori_task->updateTaskModel(N_prec);
			N_prec = posori_task->_N;
			joint_task->updateTaskModel(N_prec);

			Vector3d pos;
			robot->position(pos, control_link, control_point);

			double vertical_offset = 0.2;

			if( (pos - X).norm() < 0.15 )
			{
				cout << "Gripped leg, following trajectory.\n";
				count += 1;
				double q1 = (45.0*M_PI/180)*sin(6*M_PI*count/100 - M_PI/2.0) - (15.0*M_PI/180);
				double q2 = (-30.0*M_PI/180)*sin(6*M_PI*count/100 - M_PI/2.0) - (90.0*M_PI/180);

				X(0) = -l1 * sin(q1) - l2 * sin(q1 + q2);
				X(1) = 0.0;
				X(2) = l1 * cos(q1) + l2 * cos(q1 + q2) + vertical_offset;

				// original qs are in base-of-leg frame
				// 1. transform from leg to world frame (subtract world->leg)
				X = X - T_world_leg;
				X = R_world_leg * X;

				// 2. transform from world to robot frame (add world->robot)
				X = X + T_world_robot;
				X = R_world_robot * X;

				posori_task->reInitializeTask();
				posori_task->_desired_position = X;
			}

			// compute torques
			posori_task->computeTorques(posori_task_torques);
			joint_task->computeTorques(joint_task_torques);

			command_torques = posori_task_torques + joint_task_torques;
			// command_torques = posori_task_torques;

			// GRIPPER CONTROLLER
			Vector2d gripper_pos_des = Vector2d::Zero();
			gripper_pos_des(0) = 0.05 + 0.05 * sin(time);
			gripper_pos_des(1) = -0.05 - 0.05 * sin(time);
			command_torques_hand = -20.0 * (gripper_pos - gripper_pos_des) - 5.0 * gripper_vel;
			// command_torques_hand = -10.0 * gripper_vel; // -10.0 is kv
		}

		// send to redis
		// cout << "Sending torques to redis\n";
		// redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
		command_torques_full << command_torques, command_torques_hand;
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques_full);

		controller_counter++;
	}

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";

	return 0;
}
