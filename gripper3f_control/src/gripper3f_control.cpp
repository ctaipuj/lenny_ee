#include "../include/gripper3f_control.h"

Gripper3f::Gripper3f(){
	sub=n.subscribe("SModelRobotInput",10, &Gripper3f::gripper_statusCallback,this);
	pub=n.advertise<robotiq_s_model_control::SModel_robot_output>("SModelRobotOutput",50);
	data.resize(22);
	command.resize(19,0);
	init_ok=0;
}

void Gripper3f::gripper_statusCallback(const robotiq_s_model_control::SModel_robot_input::ConstPtr& msg){
					
	data[0]=msg->gACT;
	data[1]=msg->gMOD;
	data[2]=msg->gGTO;
	data[3]=msg->gIMC;
	data[4]=msg->gSTA;
	data[5]=msg->gDTA;
	data[6]=msg->gDTB;
	data[7]=msg->gDTC;
	data[8]=msg->gDTS;
	data[9]=msg->gFLT;
	data[10]=msg->gPRA;
	data[11]=msg->gPOA;
	data[12]=msg->gCUA;
	data[13]=msg->gPRB;
	data[14]=msg->gPOB;
	data[15]=msg->gCUB;
	data[16]=msg->gPRC;
	data[17]=msg->gPOC;
	data[18]=msg->gCUC;
	data[19]=msg->gPRS;
	data[20]=msg->gPOS;
	data[21]=msg->gCUS;			
}

bool Gripper3f::init(){
	reset();	
	init_start=1;		
	if(data[0]==0){
		ROS_INFO("gripper3f init routine has begun");
		setSpeed(150);
		setForce(100);
		command[0]=1;
		command[2]=1;
		publisher();
	}
	while(data[4]!=3){
		ros::spinOnce();
		ROS_WARN_ONCE("Activating 3 finger gripper!");
	}	
	if(data[4]==3){ 
		ROS_INFO("INIT OK, now you can command the gripper");
		command[2]=1;
		publisher();
		init_ok=1;
		return 1;
		}
}

void Gripper3f::close(int tolerance){
	int mode=command[1];
	if(init_ok!=1){
		init_error();
		goto end;
	}
	switch(mode){
		case 0:
			command[7]=241; //When basic mode
			publisher();
			while(data[11]<241-tolerance || data[11]>241+tolerance)
				ros::spinOnce();
			ROS_INFO("gripper 3f Goal Position: CLOSE reached");
			break;
			
		case 1:
			command[7]=113; //When pinch mode
			publisher();
			while(data[11]<113-tolerance || data[11]>113+tolerance)
				ros::spinOnce();
			ROS_INFO("gripper 3f Goal Position: CLOSE reached");
			break;
		
		case 2:
			command[7]=241; //When wide mode
			publisher();
			while(data[11]<241-tolerance || data[11]>241+tolerance){
				ros::spinOnce();}
			ROS_INFO("gripper 3f Goal Position: CLOSE reached");
			break;
			
		case 3:
			command[7]=233; //When scissormode
			publisher();
			while(data[20]<233-tolerance)
				ros::spinOnce();
			ROS_INFO("gripper 3f Goal Position: CLOSE reached");
			break;
		
		default:
			ROS_ERROR("Something went wrong with the gripper!");
	}
	end:;
}

void Gripper3f::open(int p){
	int mode=command[1];
	if(init_ok!=1){
		init_error();
		goto end;
	}
	switch(mode){
		case 0:
			command[7]=0; //When basic mode
			publisher();
			while(data[11]>7)
				ros::spinOnce();
			if(p!=1)
				ROS_INFO("gripper 3f Goal Position: OPEN reached");
			break;
			
		case 1:
			command[7]=0; //When pinch mode
			publisher();
			while(data[11]>7)
				ros::spinOnce();
			if(p!=1)
				ROS_INFO("gripper 3f Goal Position: OPEN reached");
			break;
		
		case 2:
			command[7]=0; //When wide mode
			publisher();
			while(data[11]>7)
				ros::spinOnce();
			if(p!=1)
				ROS_INFO("gripper 3f Goal Position: OPEN reached");
			break;
			
		case 3:
			command[7]=0; //When scissormode
			publisher();
			while(data[20]>16)
				ros::spinOnce();
			if(p!=1)
				ROS_INFO("gripper 3f Goal Position: OPEN reached");
			break;
			
		default:
			ROS_ERROR("Something went wrong with the gripper!");
	}
	end:;
}

void Gripper3f::moveto(int goal, int tolerance){
	int goal_pose=setGoal(goal);
	int mode=command[1];
	if(init_ok!=1){
		init_error();
		goto end;
	}
	publisher();
	switch(mode){
		case 3:
			while(data[20]<goal_pose-tolerance || data[20]>goal_pose+tolerance)
				ros::spinOnce();
			ROS_INFO("gripper 3f Goal Position: %i reached",goal);
			break;
		default:
			while(data[11]<goal_pose-tolerance || data[11]>goal_pose+tolerance)
				ros::spinOnce();
			ROS_INFO("gripper 3f Goal Position: %i reached",goal);
			break;
		}
	end:;
}

void Gripper3f::setMode(int m){
	if(init_ok!=1){
		init_error();
		goto end;
	}
	open(1);
	switch(m){
		case 0: //basic
			command[1]=0;
			publisher();
			while(data[20]<136 ||data[20]>138)
				ros::spinOnce();
			ROS_INFO("Change to basic mode OK");
			break;
		
		case 1: //Pinch
			command[1]=1;
			publisher();
			while(data[20]<119 || data[20]>221)
				ros::spinOnce();
			ROS_INFO("Change to pinch mode OK");
			break;
			
		case 2: //Wide
			command[1]=2;
			publisher();
			while(data[20]<22 || data[20]>24)
				ros::spinOnce();
			ROS_INFO("Change to Wide mode OK");
			break;
	
		case 3: //Scissor
			command[1]=3;
			publisher();
			ros::Duration(0.2).sleep();
			ros::spinOnce();
			while(data[4]!=3)
				ros::spinOnce();
			ROS_INFO("Change to Scissor mode OK");
			break;
		
		default:
			command[1]=0;
			publisher();
			while(data[20]<135 ||data[20]>139)
				ros::spinOnce();
			ROS_INFO("Setting Default mode OK");
			break;
	}
	end:;
}

void Gripper3f::setSpeed(int speed){
	if(init_ok!=1 && init_start!=1)
		init_warn();
	command[8]=speed;
	if(speed>255)
		command[8]=255;
	if(speed<0)
		command[8]=0;
	publisher();
}

void Gripper3f::setForce(int force){
	if(init_ok!=1 && init_start!=1)
		init_warn();
	command[9]=force;
	if(force>250)
		command[9]=250;
	if(force<0)
		command[9]=0;
	publisher();
}

int Gripper3f::getMode(){
	return command[1];
}

int Gripper3f::getForce(){
	return command[9];
}

int Gripper3f::getSpeed(){
	return command[8];
}

int Gripper3f::getPose(){
	int mode=command[1];
	switch(mode){
		case 3:
			return data[20];
		default:
			return data[11];
		}
}





void Gripper3f::publisher(){
	robotiq_s_model_control::SModel_robot_output cmd;
	
	cmd.rACT=command[0];
	cmd.rMOD=command[1];
	cmd.rGTO=command[2];
	cmd.rATR=command[3];
	cmd.rGLV=command[4];
	cmd.rICF=command[5];
	cmd.rICS=command[6];
	cmd.rPRA=command[7];
	cmd.rSPA=command[8];
	cmd.rFRA=command[9];
	cmd.rPRB=command[10];
	cmd.rSPB=command[11];
	cmd.rFRB=command[12];
	cmd.rPRC=command[13];
	cmd.rSPC=command[14];
	cmd.rFRC=command[15];
	cmd.rPRS=command[16];
	cmd.rSPS=command[17];
	cmd.rFRS=command[18];
	
	pub.publish(cmd);
}

void Gripper3f::reset(){
	init_ok=0;
	init_start=0;
	for(int i=0;i<data.size();i++)
		data[i]=0;
	publisher();
}

int Gripper3f::setGoal(int goal){
	int mode=command[1];
	switch(mode){
		case 0:
			command[7]=goal;
			if(goal>241)
				command[7]=241;
			if(goal<6)
				command[7]=6;
			break;
		
		case 1:
			command[7]=goal;
			if(goal>113)
				command[7]=113;
			if(goal<6)
				command[7]=6;
			break;
		
		case 2:
			command[7]=goal;
			if(goal>241)
				command[7]=241;
			if(goal<6)
				command[7]=6;
			break;
		
		case 3:
			command[7]=goal;
			if(goal>232)
				command[7]=232;
			if(goal<15)
				command[7]=15;
			break;						
	}
	return command[7];
}

void Gripper3f::init_warn(){
	ROS_WARN("This parameter is overridden by gripper init routine. Make sure you init the gripper first.");
}

void Gripper3f::init_error(){
	ROS_ERROR("Something went wrong with gripper3f initialization!");
	ROS_WARN("Make sure you init the gripper first");
}
