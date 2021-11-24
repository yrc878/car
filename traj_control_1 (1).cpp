#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>

#include <stdio.h>
#include <termios.h>       //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>        //STDIN_FILENO

ros::Publisher speed_pub;
ros::Publisher steering_pub;
ros::Subscriber yaw_sub;

std_msgs::Int16 speed;
std_msgs::UInt8 steering;
float yaw;
float mesures_sensors_esquerre;
float mesures_sensors_centre;
float mesures_sensors_dret;
// Non blocking keyboard input

struct termios orig_termios;

void reset_terminal_mode()
{
    tcsetattr(0, TCSANOW, &orig_termios);
}

void set_conio_terminal_mode()
{
    struct termios new_termios;

    /* take two copies - one for now, one for later */
    tcgetattr(STDIN_FILENO, &orig_termios);
    memcpy(&new_termios, &orig_termios, sizeof(new_termios));

    /* register cleanup handler, and set the new terminal mode */
    atexit(reset_terminal_mode);
    new_termios.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

// End Non blocking keyboard input

void poseCallback(const std_msgs::Float32::ConstPtr& msg)
{
  yaw = msg->data;
}

void mseCallback(const std_msgs::Float32::ConstPtr& msg)
{
  mesures_sensors_esquerre = msg->data;
}

void mscCallback(const std_msgs::Float32::ConstPtr& msg)
{
  mesures_sensors_centre = msg->data;
}

void msdCallback(const std_msgs::Float32::ConstPtr& msg)
{
  mesures_sensors_dret = msg->data;
}

void speed_variation(int _val)
{
	speed.data = speed.data + _val;
	std::cout <<  " speed at: " << speed.data << "; yaw = " << yaw << std::endl;
}

int main(int argc, char **argv) {

	set_conio_terminal_mode();  // to allow non-blocking terminal input

	ros::init(argc, argv, "trajectory_control");
	ros::NodeHandle n;
  
	speed_pub = n.advertise<std_msgs::Int16>("speed", 0);
	steering_pub = n.advertise<std_msgs::UInt8>("steering", 110);
	yaw_sub = n.subscribe("/yaw", 1, poseCallback);
	mse_sub = n.subscribe("/mesures_sensors_esquerre", 1, poseCallback);
	msc_sub = n.subscribe("/mesures_sensors_centre", 1, poseCallback);
	msd_sub = n.subscribe("/mesures_sensors_dret", 1, poseCallback);

	speed.data = 0;
	steering.data = 110;

	ros::Rate loop_rate(20);

	std::cout << std::endl << "w to run faster, s to run slower, SPACE to stop, ^C to exit" << std::endl << "Press key to start... " << std::endl;
	while (ros::ok())
	{
	  int c = getchar();   // call your non-blocking input function
	  if (c == 'w') { speed_variation(100); }
	  else if (c == 's') { speed_variation(-100); }
	  else if (c == ' ')
	  {
	  	speed.data=0;
		std::cout <<  "Car stopped! " << "; yaw = " << yaw << std::endl;
	  }
	  speed_pub.publish(speed); 

// steering control algorithm

	  steering_pub.publish(steering);
          //spin
	  ros::spinOnce();
	  loop_rate.sleep();
	  
	  // define the sensors
	  int SensorLeft,SensorLeftPrevious;
	  int SensorMid,SensorMidPrevious;
	  int SensorRight,SensorRightPrevious;
	  // car at center
	  if (mesures_sensors_esquerre>=5 && mesures_sensors_centre<5 && mesures_sensors_dret>=5) {
	      SensorMid = 1;
	      SensorRight = 0;
	      SensorLeft = 0;
	  } // car at a bit left
	  else if (mesures_sensors_esquerre<5 && mesures_sensors_centre<5 && mesures_sensors_dret>=5) {
	      SensorMid = 1;
	      SensorRight = 0;
	      SensorLeft = 1;
	  } // car at far left
	  else if (mesures_sensors_esquerre<5 && mesures_sensors_centre>=5 && mesures_sensors_dret>=5) {
	      SensorMid = 0;
	      SensorRight = 0;
	      SensorLeft = 1;
	  } // car at a bit right
	  else if (mesures_sensors_esquerre>=5 && mesures_sensors_centre<5 && mesures_sensors_dret<5) {
	      SensorMid = 1;
	      SensorRight = 1;
	      SensorLeft = 0;
	  } // car at far right 
	  else if (mesures_sensors_esquerre>=5 && mesures_sensors_centre>=5 && mesures_sensors_dret<5) {
	      SensorMid = 0;
	      SensorRight = 1;
	      SensorLeft = 0;
	  } else {
	      SensorMid = 1;
	      SensorRight = 0;
	      SensorLeft = 0;
	  }
	  
	  // control rules
	  int Angle
	  // center to center 
	  if (SensorMidPrevious==1 && SensorRightPrevious==0 && SensorLeftPrevious==0 && SensorMid==1 && SensorRight==0 && SensorLeft==0) {
	      Angle = 0; 
	  } // center to left 
	  else if (SensorMidPrevious==1 && SensorRightPrevious==0 && SensorLeftPrevious==0 && SensorMid==1 && SensorRight==0 && SensorLeft==1) {
	      Angle = pi/(36*2);
	  } // center to far left
	  else if (SensorMidPrevious==1 && SensorRightPrevious==0 && SensorLeftPrevious==0 && SensorMid==0 && SensorRight==0 && SensorLeft==1) {
	      Angle = pi/(36*1); 
	  } // left to far left
	  else if (SensorMidPrevious==1 && SensorRightPrevious==0 && SensorLeftPrevious==1 && SensorMid==0 && SensorRight==0 && SensorLeft==1) {
	      Angle = pi/(36*1); 
	  } // far left to left
	  else if (SensorMidPrevious==0 && SensorRightPrevious==0 && SensorLeftPrevious==1 && SensorMid==1 && SensorRight==0 && SensorLeft==1) {
	      Angle = pi/(36*2);
	  } // left to center
	  else if (SensorMidPrevious==1 && SensorRightPrevious==0 && SensorLeftPrevious==1 && SensorMid==1 && SensorRight==0 && SensorLeft==0) {
	      Angle = 0;
	  } // left to left
	  else if (SensorMidPrevious==1 && SensorRightPrevious==0 && SensorLeftPrevious==1 && SensorMid==1 && SensorRight==0 && SensorLeft==1) {
	      Angle = pi/(36*2);
	  } // left to right
	  else if (SensorMidPrevious==1 && SensorRightPrevious==0 && SensorLeftPrevious==1 && SensorMid==1 && SensorRight==1 && SensorLeft==0) {
	      Angle = -pi/(36*1);
	  } // left to far right
	  else if (SensorMidPrevious==1 && SensorRightPrevious==0 && SensorLeftPrevious==1 && SensorMid==0 && SensorRight==1 && SensorLeft==0) {
	      Angle = -pi/(36*0.5);
	  } // far left to center
	  else if (SensorMidPrevious==0 && SensorRightPrevious==0 && SensorLeftPrevious==1 && SensorMid==1 && SensorRight==0 && SensorLeft==0) {
	      Angle = -pi/(36*2);
	  } // far left to right
	  else if (SensorMidPrevious==0 && SensorRightPrevious==0 && SensorLeftPrevious==1 && SensorMid==1 && SensorRight==1 && SensorLeft==0) {
	      Angle = -pi/(36*1);
	  } // far left to far right
	  else if (SensorMidPrevious==0 && SensorRightPrevious==0 && SensorLeftPrevious==1 && SensorMid==0 && SensorRight==1 && SensorLeft==0) {
	      Angle = -pi/(36*0.25);
	  } // far left to far left
	  else if (SensorMidPrevious==0 && SensorRightPrevious==0 && SensorLeftPrevious==1 && SensorMid==0 && SensorRight==0 && SensorLeft==1) {
	      Angle = pi/(36*0.5);
	  } // center to right
	  else if (SensorMidPrevious==1 && SensorRightPrevious==0 && SensorLeftPrevious==0 && SensorMid==1 && SensorRight==1 && SensorLeft==0) {
	      Angle = -pi/(36*2);
	  } // center to far right
	  else if (SensorMidPrevious==1 && SensorRightPrevious==0 && SensorLeftPrevious==0 && SensorMid==0 && SensorRight==1 && SensorLeft==0) {
	      Angle = -pi/(36*1);
	  } // right to far right
	  else if (SensorMidPrevious==1 && SensorRightPrevious==1 && SensorLeftPrevious==0 && SensorMid==0 && SensorRight==1 && SensorLeft==0) {
	      Angle = -pi/(36*1);
	  } // far right to right
	  else if (SensorMidPrevious==0 && SensorRightPrevious==1 && SensorLeftPrevious==0 && SensorMid==1 && SensorRight==1 && SensorLeft==0) {
	      Angle = -pi/(36*2);
	  } // right to center
	  else if (SensorMidPrevious==1 && SensorRightPrevious==1 && SensorLeftPrevious==0 && SensorMid==1 && SensorRight==0 && SensorLeft==0) {
	      Angle = 0;
	  } //right to left
	  else if (SensorMidPrevious==1 && SensorRightPrevious==1 && SensorLeftPrevious==0 && SensorMid==1 && SensorRight==0 && SensorLeft==1) {
	      Angle = pi/(36*1);
	  } // right to right
	  else if (SensorMidPrevious==1 && SensorRightPrevious==1 && SensorLeftPrevious==0 && SensorMid==1 && SensorRight==1 && SensorLeft==0) {
	      Angle = -pi/(36*2);
	  } // right to far left
	  else if (SensorMidPrevious==1 && SensorRightPrevious==1 && SensorLeftPrevious==0 && SensorMid==0 && SensorRight==0 && SensorLeft==1) {
	      Angle = pi/(36*0.5);
	  } // far right to center
	  else if (SensorMidPrevious==0 && SensorRightPrevious==1 && SensorLeftPrevious==0 && SensorMid==1 && SensorRight==0 && SensorLeft==0) {
	      Angle = pi/(36*2);
	  } // far right to left
	  else if (SensorMidPrevious==0 && SensorRightPrevious==1 && SensorLeftPrevious==0 && SensorMid==1 && SensorRight==0 && SensorLeft==1) {
	      Angle = pi/(36*1);
	  } // far right to far left
	  else if (SensorMidPrevious==0 && SensorRightPrevious==1 && SensorLeftPrevious==0 && SensorMid==0 && SensorRight==0 && SensorLeft==1) {
	      Angle = pi/(36*0.25);
	  } // far right to far right
	  else if (SensorMidPrevious==0 && SensorRightPrevious==1 && SensorLeftPrevious==0 && SensorMid==0 && SensorRight==1 && SensorLeft==0) {
	      Angle = -pi/(36*0.5);
	  }
	  else {
	      Angle = 0;
	  }
	 SensorMidPrevious=SensorMid;
	 SensorRightPrevious=SensorRight;
	 SensorLeftPrevious=SensorLeft;
	 steering.data = Angle;
	  
	}
}




