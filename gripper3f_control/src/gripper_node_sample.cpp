#include "../include/gripper3f_control.h" //include the control class

int main(int argc, char*argv[]){ //main

	ros::init(argc,argv,"gripper_move_group"); //init a node
	ros::NodeHandle n; //handle
	Gripper3f right_gripper; //right_gripper
	ros::Duration(1).sleep(); // This is neccesary !
	if(right_gripper.init()){ //verify gripper init
		right_gripper.setSpeed(20);	//set speed
		right_gripper.setForce(50); //set force
		right_gripper.setMode(1); //change to pinch mode
		right_gripper.close();	//close gripper
		right_gripper.setMode(4); //change an nonexistent mode
		right_gripper.moveto(150); //move to
		right_gripper.setSpeed(200); //change speed
		right_gripper.setMode(1); //back to pinch mode
		right_gripper.moveto(200); //moveto
		right_gripper.moveto(0); //open or moveto
		right_gripper.setMode(2); //wide mode
		right_gripper.moveto(250); //move to
		right_gripper.setSpeed(30); //change speed
		right_gripper.moveto(50); //moveto
		right_gripper.open(); //open
		right_gripper.setSpeed(100); //speed again
		right_gripper.close(); //close
		right_gripper.moveto(0); // moveto 
		right_gripper.setMode(3); //set scissors mode
		right_gripper.moveto(255); //moveto
		right_gripper.open();//open
		right_gripper.setMode(0);//basic mode
	}
	ros::spin();
	return 0;
}
