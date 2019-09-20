#ifndef GRIPPER3F_CONTROL
#define GRIPPER3F_CONTROL
//ROS
#include <ros/ros.h>
//Load gripper Messages
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h>

//Using a class works better and is more organized :D

/** @brief This class controls the 3-finger robotiq gripper.
	
	This class uses rostopics SModelRobotInput and SModelRobotOut to control the s-model robotiq gripper. This class subscribes to SmodelRobotInput in order to retrieve gripper's status information and publishes commands to the SModelOutput topic.
	Boost and robotiq_s_model_control are components required.
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
	/** The constructor initializes the subscriptor and publisher functions. Advertices the SModelRobotInput. Resizes vectors.
	*/
		Gripper3f();
	/** This is the regular Callback from a ros node, this function updates the data vector that stores the gripper status.
		@param msg is the message type the nodes subscribes to: const robotiq_s_model_control::SModel_robot_input::ConstPtr&
	*/		
		void gripper_statusCallback(const robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput::ConstPtr& msg);
	/** The init function must be executed at first. It resets the gripper and executes the initialization routine. This function overrides other configured parameters previously defined. Once the init routine is done the gripper will move.
		@return returns true when the routine is executed succesfully
	*/		
		bool init();
	/** This function closes the gripper according to the different mode in which the gripper is set.
		@param (Optional) tolerance its a position goal interval +/- tolerance with respect to goal position (default 2)
	*/
		
		void close(int tolerance=2);
	/**This function opens the gripper according to the different mode in which the gripper is set. Must be called after init()
		@param (Optional) p p=1 for not printing messages otherwise 0 (default 0)
	*/
		
		void open(int p=0);
	/** This function moves the gripper to a given position between 0-255 and also has an optional tolerance position parameter executes the trayectory and waits until the goal is reached. Must be called after init()
		@param goal 0-255 where 0 is totally open and 255 totally closed
		@param tolerance (optional) tolerance interval where +/-tolerance whith respect goal position is allowed
	*/
		void moveto(int goal, int tolerance=1);
	/** Detects objects in gripper's fingers
		@return true if and object is detected false otherwise.	
	*/	
		bool objectDetected();
	/** Sets four different gripper's mode. Must be called after init()
		0:basic mode
		1:pinch mode
		2:wide mode
		3:scissors mode
		@param int mode 0-3 according desired mode (default basic mode)	
	*/
		void setMode(int m);
	/**This function sets gripper's speed
		@param int speed 0-255 where 255 is max speed (default 150)
	*/
		void setSpeed(int speed);
	/**This function sets gripper's force
		@param int force 0-255 where 255 is max force (default 100)
	*/	
		void setForce(int force);
	/**Gets the current gripper's mode
		@return int 0-3 see setMode()
	*/	
		int getMode();
	/**Gets the current gripper's force
		@return int 0-255 see setForce()
	*/	
		int getForce();
	/**Gets the current gripper's speed
		@return int 0-255 see setSpeed()
	*/	
		int getSpeed();
	/**Gets the current gripper's position
		@return int 0-255 see moveto()
	*/	
		int getPose();
};
#endif
