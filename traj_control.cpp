
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

#include <stdio.h>
#include <termios.h>       //termios, TCSANOW, ECHO, ICANON
#include <unistd.h>        //STDIN_FILENO

ros::Publisher speed_pub;
std_msgs::Int16 speed;

ros::Publisher steering_pub;
std_msgs::UInt8 steering;

ros::Publisher position_pub;
std_msgs::UInt8 position;

ros::Subscriber mse_sub;
ros::Subscriber msc_sub;
ros::Subscriber msd_sub;

ros::Subscriber yaw_sub;
ros::Subscriber puntual_sub;

/////////////////////////////////////////////////
//      	Initializing variables  		   //
/////////////////////////////////////////////////

float yaw;
float mesures_sensors_esquerre;
float mesures_sensors_centre;
float mesures_sensors_dret;

int vel = 0;
int mensaje_puntual;
int Angle = 90;
int recta=1, corbad=0, corbai=0;

/////////////////////////////////////////////////
//      	Non blocking keyboard input		   //
/////////////////////////////////////////////////


struct termios orig_termios,new_termios;

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
//    tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);
}

int getch()
{
  tcsetattr( STDIN_FILENO, TCSANOW, &new_termios);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &orig_termios);  // restore old settings
  return c;
}

/////////////////////////////////////////////////
//					CALLBACKS				   //
/////////////////////////////////////////////////

void poseCallback(const std_msgs::Float32::ConstPtr& msg)
{
  yaw = msg->data;
}

void PuntualCallback(const std_msgs::Float32::ConstPtr& msgpunt)
{
	mensaje_puntual = msgpunt->data;

	if (mensaje_puntual==0){

	std::cout << "Mensaje Puntual =   "<< mensaje_puntual << std::endl;
	}

	else if (mensaje_puntual!=0){
	std::cout << "                                       Mensaje Puntual = " << mensaje_puntual << std::endl;
	}
}

void mseCallback(const std_msgs::Float32::ConstPtr& msgSE)
{
  mesures_sensors_esquerre = msgSE->data;

//std::cout << "E = " << mesures_sensors_esquerre << std::endl;

}

void mscCallback(const std_msgs::Float32::ConstPtr& msgSC)
{
  mesures_sensors_centre = msgSC->data;

//std::cout << "C = " << mesures_sensors_centre << std::endl;

}

void msdCallback(const std_msgs::Float32::Ptr& msgSD)
{
  mesures_sensors_dret = msgSD->data;

//std::cout << "D = " << mesures_sensors_dret << std::endl;

}

/////////////////////////////////////////////////
//      		   Other VOIDS  			   //
/////////////////////////////////////////////////

void teclado(){

   int c = getch();   // call your non-blocking input function
	 
   if (c == 'k') { 
	corbad=1;
	corbai=0;
	recta=0;
	Angle=40;
	}

   if(c == 'o'){

	corbad=0;
	corbai=0;
	recta=1;	

	}

   if(c == 'l'){

	corbad=0;
	corbai=1;
	recta=0;
	Angle=140;
	}

   if (c == 'w') { 
	vel=70;
	std::cout <<  " speed at: " << vel << "; yaw = " << yaw << std::endl;
	}
	
   else if (c == 's') { 

	vel=45;
	std::cout <<  " speed at: " << vel << "; yaw = " << yaw << std::endl;
	 }

    else if (c == ' ')
	{
		vel=0;
		std::cout <<  " Car stopped! " << "; yaw = " << yaw << std::endl;
	}
    else if (c == 'x')
	{
		vel=-45;
		std::cout <<  " Car stopped! " << "; yaw = " << yaw << std::endl;
	}
	
	/*else if (c=='a') {
		if (Angle<=150 and Angle>=30){
				Angle=Angle+10;
				std::cout <<  " speed at: " << vel << "; yaw = " << yaw << std::endl;
				}
		else if (Angle>=150) Angle=150;
		else if (Angle<=30) Angle=30;
	}	

	else if (c=='d') {
		if (Angle<=150 and Angle>=30){
				Angle=Angle-10;
				std::cout <<  " speed at: " << vel << "; yaw = " << yaw << std::endl;}

		else if (Angle>=150) Angle=150;
		else if (Angle<=30) Angle=30;
	}*/
}

/////////////////////////////////////////////////
//      				MAIN 				   //
/////////////////////////////////////////////////

int main(int argc, char **argv) {

	int Angle=90;
    vel=0;
	int Position=3;
	int SignalPrevious=3;
	int SignalCurrent=3;
	int vector[5];
	int uno=0, cinco=0, centre=0;


	int s, a, m, cont=0;	

	set_conio_terminal_mode();  // to allow non-blocking terminal input

	ros::init(argc, argv, "trajectory_control");
	ros::NodeHandle n;

	speed_pub = n.advertise<std_msgs::Int16>("speed", 0);
	steering_pub = n.advertise<std_msgs::UInt8>("steering", 110);
	position_pub = n.advertise<std_msgs::UInt8>("position", 110);

	yaw_sub = n.subscribe("/yaw", 1, poseCallback);
	puntual_sub = n.subscribe("mensaje_puntual", 1, PuntualCallback);

	mse_sub = n.subscribe("mesures_sensors_esquerre", 1, mseCallback);
	msc_sub = n.subscribe("mesures_sensors_centre", 1, mscCallback);
	msd_sub = n.subscribe("mesures_sensors_dret", 1, msdCallback);

	//speed.data = 45;
	steering.data = 90;

	ros::Rate loop_rate(100);

	while (ros::ok())
	{
		teclado();

	//Speed Control with Punctual Magnets
						  if (mensaje_puntual==1)
		  				  {
							  vel=42;
							  std::cout <<  " speed at: " << vel << "; yaw = " << yaw << std::endl;
							  	corbad=1;
								corbai=0;
								recta=0;
								Angle=110;
							  mensaje_puntual = 0;
						  }
						  if (mensaje_puntual==2)
						  {
							  vel=60;
							  std::cout <<  " speed at: " << vel << "; yaw = " << yaw << std::endl;
							  	corbad=0;
								corbai=0;
								recta=1;
								Angle=90;
							  mensaje_puntual = 0;
						  }
						   if (mensaje_puntual==3)
						  {
							  vel=42;
							  std::cout <<  " speed at: " << vel << "; yaw = " << yaw << std::endl;
							  	corbad=0;
								corbai=1;
								recta=0;
								Angle=70;
							  mensaje_puntual = 0;
						  }
						   
						   if (mensaje_puntual==4)
						  {
							  vel = 0;
							  std::cout << "Speed at:" << vel << std::endl << "     Car stopped! " << std::endl;
							  mensaje_puntual = 0;
						  }

	  speed.data = vel;
	  speed_pub.publish(speed); 
	  std::cout << "Output speed: " << vel << std::endl;


	//Steering Control

		///////////////////////////////////////////SIGNAL CURRENT

			// car at far right 
			  if (mesures_sensors_dret<7.5) {
					  
					  SignalCurrent = 1;
					  
				  }
			 // car at a bit right
			  /*if (mesures_sensors_esquerre>5 && mesures_sensors_centre<7 && mesures_sensors_dret<5 ) {
					  
					  SignalCurrent = 2 ;

				  }
			 // car at center	  
			  if (mesures_sensors_esquerre>5 && mesures_sensors_centre<7 && mesures_sensors_dret>5 ) {
				 
					  SignalCurrent = 3;
					  
				  }
			 // car at a bit left
			  if (mesures_sensors_esquerre<5 && mesures_sensors_centre<7 && mesures_sensors_dret>5) {
					  
					  SignalCurrent = 4;
					  
				  }*/
			 // car at far left
			  if (mesures_sensors_esquerre<7.5) {
					 
					  SignalCurrent = 5;
					  
				  }

			  if (mesures_sensors_esquerre<5 && mesures_sensors_centre<5 && mesures_sensors_dret<5){

					SignalCurrent = SignalPrevious;
				
				  }	
				


std::cout << "Signal Previous: " << SignalPrevious << std::endl;
std::cout << "Signal Current: " << SignalCurrent << std::endl;


		if (corbad!=1 && corbai!=1){
				if (SignalPrevious==1 && SignalCurrent==5){

				Angle=80;

				}

				if (SignalPrevious==5 && SignalCurrent==1){

				Angle=100;

				}
				if (SignalPrevious==1 && SignalCurrent==1){
	
					if(Angle<100) Angle=Angle+6;

				}

				if (SignalPrevious==5 && SignalCurrent==5){

					if(Angle>80) Angle=Angle-6;

				}

		}


	  if(corbai==1){


		
				if (SignalPrevious==1 && SignalCurrent==5){

				Angle=50;

				}

				if (SignalPrevious==5 && SignalCurrent==1){

				Angle=75;

				}

				if (SignalPrevious==1 && SignalCurrent==1){
	
				if(Angle<75) Angle=Angle+5;

				}

				if (SignalPrevious==5 && SignalCurrent==5){

				if(Angle>50)Angle=Angle-5;

				}

	   }

	if(corbad==1){



	
				if (SignalPrevious==1 && SignalCurrent==5){

				Angle=130;

				}

				if (SignalPrevious==5 && SignalCurrent==1){

				Angle=105;

				}

				if (SignalPrevious==1 && SignalCurrent==1){

					if(Angle<130) Angle=Angle+5;

				}

				if (SignalPrevious==5 && SignalCurrent==5){

					if(Angle>105) Angle=Angle-5;

				}

	}

	if(Angle>150) Angle=150;

	if(Angle<30) Angle=30;
/*//RETURNING TO THE CENTRE

				if (SignalPrevious==4 && SignalCurrent==3){

				Position=3;

				}

				if (SignalPrevious==5 && SignalCurrent==4){

				Position=4;

				}

				if (SignalPrevious==2 && SignalCurrent==3){

				Position=3;

				}

				if (SignalPrevious==1 && SignalCurrent==2){

				Position=2;

				}

				if (SignalPrevious==1 && SignalCurrent==3){

				Position=4;

				}


// TO THE LEFT FROM CENTRE

				if (SignalPrevious==3 && SignalCurrent==3){

				Position=3;
	
				}

				if (SignalPrevious==4 && SignalCurrent==4){

				Position=4;
	
				}

				if (SignalPrevious==5 && SignalCurrent==5){

				Position=5;
	
				}

				if (SignalPrevious==3 && SignalCurrent==4){

				Position=4;

				}

				if (SignalPrevious==3 && SignalCurrent==1 && Angle>90){

				Position=4;

				}

				if (SignalPrevious==3 && SignalCurrent==2 && Angle>90){

				Position=4;

				}

				if (SignalPrevious==4 && SignalCurrent==1 && Angle>90){

				Position=4;
	
				}
				
				if (SignalPrevious==4 && SignalCurrent==5 && Angle>90){

				Position=5;
	
				}

				if (SignalPrevious==4 && SignalCurrent==2 && Angle>90){

				Position=4;
	
				}

				if (SignalPrevious==2 && SignalCurrent==5 && Angle>90){

				Position=5;
	
				}

				if (SignalPrevious==1 && SignalCurrent==5 && Angle>90){

				Position=5;
	
				}

// TO THE RIGHT FROM CENTRE

				if (SignalPrevious==3 && SignalCurrent==3){

				Position=3;
	
				}

				if (SignalPrevious==2 && SignalCurrent==2){

				Position=2;
	
				}

				if (SignalPrevious==1 && SignalCurrent==1){

				Position=1;
	
				}

				if (SignalPrevious==3 && SignalCurrent==2){

				Position=2;
	
				}

				if (SignalPrevious==3 && SignalCurrent==4 && Angle<90){

				Position=2;
	
				}

				if (SignalPrevious==3 && SignalCurrent==5 && Angle<90){

				Position=2;
	
				}

				if (SignalPrevious==2 && SignalCurrent==4 && Angle<90){

				Position=2;
	
				}
				
				if (SignalPrevious==2 && SignalCurrent==1 && Angle<90){

				Position=1;
	
				}

				if (SignalPrevious==2 && SignalCurrent==5 && Angle<90){

				Position=2;
	
				}

				if (SignalPrevious==4 && SignalCurrent==1 && Angle<90){

				Position=1;
	
				}

				if (SignalPrevious==5 && SignalCurrent==1 ){

				Position=1;
	
				}*/

		
	    	SignalPrevious=SignalCurrent;

			//vector[cont]=Position;
			//cont++;


		/*if(cont==5){

			m=0;


		for (int i=0;i<5;i++){	

		s=0;

			for(int j=0; j<5;j++){

				if(vector[i]==vector[j] &&i!=j){

				s=s+1;	

				}

			}
											
			if(s>=m){

			m=s;

			a=i;
			
			}

		}	
			
			Position=vector[a];

			if (Position==1){

				Angle=50;

				}

				if (Position==2){

				Angle=75;

				}

				if (Position==3){

				Angle=90;

				}

				if (Position==4){

				Angle=105;

				}

				if (Position==5){

				Angle=130;

				}*/

			steering.data = Angle;	
			steering_pub.publish(steering);
			//position.data = Position;	
			//position_pub.publish(position);
			std::cout << "Output angle: " << Angle << std::endl;
			//std::cout << "Output position: " << Position << std::endl;
		
			/*for (int i=0;i<5;i++) std::cout <<  vector[i] << ",";
			std::cout << std::endl;
			cont=0;*/
				
	//}

	ros::spinOnce();
	loop_rate.sleep();
	}
}
