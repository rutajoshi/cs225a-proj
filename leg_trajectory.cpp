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
	// auto robot = new Sai2Model::Sai2Model(robot_file, false);
	// robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	// VectorXd initial_q = robot->_q;
	// robot->updateModel();

	// load leg
	auto human_leg = new Sai2Model::Sai2Model(leg_file, false);
	human_leg->_q << 0.5235987756, -1.0471975512;
	VectorXd initial_leg_q = human_leg->_q;
	human_leg->updateKinematics();

	// prepare robot controller
	// int dof = robot->dof();
	// const string link_name = "link7";
	// const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
	// VectorXd command_torques = VectorXd::Zero(dof);

	// Model quantities for robot operational space control
	// MatrixXd Jv = MatrixXd::Zero(3,dof);
	// MatrixXd Lambda = MatrixXd::Zero(3,3);
	// MatrixXd J_bar = MatrixXd::Zero(dof,3);
	// MatrixXd N = MatrixXd::Zero(dof,dof);
	//
	// robot->Jv(Jv, link_name, pos_in_link);
	// robot->taskInertiaMatrix(Lambda, Jv);
	// robot->dynConsistentInverseJacobian(J_bar, Jv);
	// robot->nullspaceMatrix(N, Jv);

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

	VectorXd q1(7);
	q1 << (30/180.0)*M_PI, (45/180.0)*M_PI, (60/180.0)*M_PI,
				(75/180.0)*M_PI, (90/180.0)*M_PI, (105/180.0)*M_PI,(120/180.0)*M_PI;

	VectorXd q2(7);
	q2 << (-60/180.0)*M_PI, (-67.5/180.0)*M_PI, (-75/180.0)*M_PI,
				(-82.5/180.0)*M_PI, (-90/180.0)*M_PI, (-105/180.0)*M_PI,(-120/180.0)*M_PI;

	for (int i = 0; i < 7; i++) {
		VectorXd tempq(2);
		tempq << q1[i], q2[i];
		human_leg->_q = tempq;
		human_leg->updateKinematics();
		Vector3d xt(3);
		human_leg->positionInWorld(xt, link_name_leg, pos_in_link_leg);
		leg_joint_outputs << xt << "\n\n";
	}

	// while (runloop) {
	// 	// wait for next scheduled loop
	// 	timer.waitForNextLoop();
	// 	double time = timer.elapsedTime() - start_time;
	//
	// 	// read robot state from redis
	// 	// robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	// 	// robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	// 	// robot->updateModel();
	// 	human_leg->updateKinematics();
	//
	// 	// **********************
	// 	// WRITE YOUR CODE AFTER
	// 	// **********************
	// 	// **********************
	// 	// WRITE YOUR CODE BEFORE
	// 	// **********************
	//
	// 	// send to redis
	// 	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	//
	// 	controller_counter++;
	//
	// }

	// Close files
	leg_joint_outputs.close();

	// command_torques.setZero();
	// redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
