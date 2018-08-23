#ifndef GRIPPER3F_CONTROL
#define GRIPPER3F_CONTROL
//ROS
#include <ros/ros.h>
//Load gripper Messages
#include <robotiq_s_model_control/SModel_robot_input.h>
#include <robotiq_s_model_control/SModel_robot_output.h>

//Using a class works better and is more organized :D

/** @brief This class controls the 3-finger robotiq gripper.
	
	This class uses rostopics SModelRobotInput and SModelRobotOut to control the s-model robotiq gripper. This class subscribes to SmodelRobotInput in order to retrieve gripper's status information and publishes commands to the SModelOutput topic.
	@author Barrero Lizarazo, Nicolas
	@date August 2018
	*/ 

class Gripper3f{
	
	private:
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::NodeHandle n;
		bool init_ok;
		bool init_start;
		std::vector<uint8_t> data;
		std::vector<uint8_t> command;
		
		void publisher();
		void reset();
		int setGoal(int goal);
		void init_warn();
		void init_error();	
	
	public:
	
		Gripper3f();
		
		void gripper_statusCallback(const robotiq_s_model_control::SModel_robot_input::ConstPtr& msg);
			
		bool init();
		
		void close(int tolerance=2);
		
		void open(int p=0);
		
		void moveto(int goal, int tolerance=1);
		
		void setMode(int m);
		
		void setSpeed(int speed);
		
		void setForce(int force);
			
		int getMode();
		
		int getForce();
		
		int getSpeed();
		
		int getPose();
};
#endif
