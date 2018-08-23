#include "../include/gripper3f_control.h"

int main(int argc, char*argv[]){

	ros::init(argc,argv,"gripper_move_group");
	ros::NodeHandle n;
	Gripper3f right_gripper;
	ros::Duration(1).sleep(); //THis SHIT solves everything!
	if(right_gripper.init()){
		right_gripper.setSpeed(20);
		right_gripper.setForce(50);
		right_gripper.setMode(1);
		right_gripper.close();
		right_gripper.setMode(4);
		right_gripper.moveto(150);
		right_gripper.setSpeed(200);
		right_gripper.setMode(1);
		right_gripper.moveto(200);
		right_gripper.moveto(0);
		right_gripper.setMode(2);
		right_gripper.moveto(250);
		right_gripper.setSpeed(30);
		right_gripper.moveto(50);
		right_gripper.open();
		right_gripper.setSpeed(100);
		right_gripper.close();
		right_gripper.moveto(0);
		right_gripper.setMode(3);
		right_gripper.moveto(255);
		right_gripper.open();
		right_gripper.setMode(0);
	}
	ros::spin();
	return 0;
}
