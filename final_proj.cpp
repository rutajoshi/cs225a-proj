#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <fstream>
#include <string>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

// Get the robot
const string robot_file = "./resources/panda_arm.urdf";
const string robot_name = "panda_arm";

// Get the human leg
const string leg_file = "./resources/human_leg.urdf";
const string leg_name = "human_leg";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// load leg
	auto human_leg = new Sai2Model::Sai2Model(leg_file, false);
	human_leg->_q << 0.349, 0.349;
	VectorXd initial_leg_q = human_leg->_q;
	human_leg->updateModel();

	// prepare robot controller
	int dof = robot->dof();
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);

	// Model quantities for robot operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// prepare human leg controller
	int dof_leg = human_leg->dof();
	const string link_name_leg = "link2";
	const Vector3d pos_in_link_leg = Vector3d(0, 0, 0.15);

	// Model quantities for human leg operational space control
	MatrixXd Jv_leg = MatrixXd::Zero(3,dof_leg);
	MatrixXd Lambda_leg = MatrixXd::Zero(3,3);
	MatrixXd J_bar_leg = MatrixXd::Zero(dof_leg,3);
	MatrixXd N_leg = MatrixXd::Zero(dof_leg,dof_leg);

	human_leg->Jv(Jv_leg, link_name_leg, pos_in_link_leg);
	human_leg->taskInertiaMatrix(Lambda_leg, Jv_leg);
	human_leg->dynConsistentInverseJacobian(J_bar_leg, Jv_leg);
	human_leg->nullspaceMatrix(N_leg, Jv_leg);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");

	// Logic for debugging with prints
	ofstream leg_joint_outputs;
	leg_joint_outputs.open("../../final_project/outputs/leg_joint_outputs.txt");

	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		int controller_number = QUESTION_1;

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			// Scalar control gains
			double kp = 100.0;
			double kv = 20.0;
			double kpj = 50.0;
			double kvj = 14.0;

			// Jacobian and Lambda
			human_leg->Jv(Jv_leg, link_name_leg, pos_in_link_leg);
			human_leg->taskInertiaMatrix(Lambda_leg, Jv_leg);
			human_leg->dynConsistentInverseJacobian(J_bar_leg, Jv_leg);
			human_leg->nullspaceMatrix(N_leg, Jv_leg);

			// Jacobian and Lambda
			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);

			// Position, velocity, desired position
			Vector3d x;
			robot->position(x, link_name, pos_in_link);
			Vector3d x_dot;
			robot->linearVelocity(x_dot, link_name, pos_in_link);
			Vector3d xd;
			xd << 0.3, 0.1, 0.5;
			Vector3d additional;
			additional << sin(M_PI*time), cos(M_PI*time), 0.0;
			xd = xd + 0.1*additional;

			// Gravity in joint space
			Eigen::VectorXd g(dof);
			robot->gravityVector(g);

			// Gravity in operational space
			Eigen::VectorXd p(dof);
			p = J_bar.transpose()*g;

			// Desired q
			VectorXd qd(dof);
			qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

			// xd_dot and xd_ddot
			Vector3d xd_dot;
			Vector3d xd_ddot;
			xd_dot << 0.1*M_PI*cos(M_PI*time), -0.1*M_PI*sin(M_PI*time), 0.0;
			xd_ddot << -0.1*M_PI*M_PI*sin(M_PI*time), -0.1*M_PI*M_PI*cos(M_PI*time), 0.0;

			// Control equations
			VectorXd F(dof);
			F = Lambda*(xd_ddot - kp*(x - xd) - kv*(x_dot - xd_dot));
			command_torques = (Jv.transpose() * F) - N.transpose()*((robot->_M)*(kpj*(robot->_q - qd) + kvj*robot->_dq)) + g;
		}
		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	// Close files
	leg_joint_outputs.close();

	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
