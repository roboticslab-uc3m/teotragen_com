/***********************************************************************
	______      _           _   _            _           _     
	| ___ \    | |         | | (_)          | |         | |    
	| |_/ /___ | |__   ___ | |_ _  ___ ___  | |     __ _| |__  
	|    // _ \| '_ \ / _ \| __| |/ __/ __| | |    / _` | '_ \ 
	| |\ \ (_) | |_) | (_) | |_| | (__\__ \ | |___| (_| | |_) |
	\_| \_\___/|_.__/ \___/ \__|_|\___|___/ \_____/\__,_|_.__/ 

*************************************************************************/
/*
Author: Domingo Esteban
Universidad Carlos III de Madrid
Leganes, Madrid, Spain, 2014
*/

// Standard headers
/*
#include <stdint.h>
#include <string.h>
#include <inttypes.h> //PARA IMPRIMIR LOS ENTEROS UINT32_T
#include <math.h> //Para redondear. ceil
*/
#include <stdio.h>          
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

// Sockets header
#include <arpa/inet.h>

// Thread header
#include <pthread.h>

// ROS header files
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include "teo_msgs/SupportFoot.h"

// TF header files
//#include "tf/transform_datatypes.h"
//#include "LinearMath/btMatrix3x3.h"


// Parameter
#define CMD_BUFFER_LEN 1024
#define SENSOR_TYPE_LINE_MAX 80 // Same value than Autopilot
#define UNIT2MILIUNIT 1000
#define MILIUNIT2UNIT 1/1000
#define MAX_UDP_DATA_POINTS 205	//ASUMIMOS QUE EL TOPE SERA DE 512 Bytes.
#define BACKLOG 1 // El número de conexiones permitidas
#define NUM_JOINTS 26 // 26 -1
#define BUF_SIZE 210 // 8*NUM_JOINTS + 2(support_foot) 

using namespace std;


//Structures used.
struct Humanoid_joints_structure{
  double joints[NUM_JOINTS];
  int32_t support_foot; // Es 32 porque hay "struct alignment" por lo que su valor minimo sera siempre 4 bytes (asi sea char)
};

//Variables
Humanoid_joints_structure TEO_joints;
sensor_msgs::JointState joints_to_send;
teo_msgs::SupportFoot support_foot_to_send;
ros::Time now;
unsigned long int num_config_received;

//MAIN FUNCTION
int main(int argc, char **argv)
{

  //Variables needed to send data
  int client_sockfd, server_sockfd; // Descriptores de fichero
  struct sockaddr_in client_addr; // Estructura address para el cliente
  struct sockaddr_in server_addr; // Estructura address para el servidor

  socklen_t sin_size; //Tamano de estructura address
  char buffer_received[BUF_SIZE];
  int numbytes_received;

  // Initialize Ros Node
  ros::init(argc, argv, "TEOTraGen_com");
  ros::NodeHandle n;	
  ros::Time::init();

  // Initialize some things
  client_sockfd = -1;
  num_config_received = 0;
  memset(buffer_received, '0', sizeof(buffer_received));

  // PUBLISHER
  ros::Publisher configuration_to_ROS = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher support_foot_to_ROS = n.advertise<teo_msgs::SupportFoot>("support_foot", 1);

  ros::Publisher joint0_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/r_hip_yaw_position_controller/command", 1);
  ros::Publisher joint1_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/r_hip_roll_position_controller/command", 1);
  ros::Publisher joint2_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/r_hip_pitch_position_controller/command", 1);
  ros::Publisher joint3_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/r_knee_pitch_position_controller/command", 1);
  ros::Publisher joint4_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/r_ankle_pitch_position_controller/command", 1);
  ros::Publisher joint5_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/r_ankle_roll_position_controller/command", 1);
  ros::Publisher joint6_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/l_hip_yaw_position_controller/command", 1);
  ros::Publisher joint7_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/l_hip_roll_position_controller/command", 1);
  ros::Publisher joint8_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/l_hip_pitch_position_controller/command", 1);
  ros::Publisher joint9_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/l_knee_pitch_position_controller/command", 1);
  ros::Publisher joint10_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/l_ankle_pitch_position_controller/command", 1);
  ros::Publisher joint11_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/l_ankle_roll_position_controller/command", 1);
  ros::Publisher joint12_command_to_ROS = n.advertise<std_msgs::Float64>("/teo/waist_yaw_position_controller/command", 1);

  std_msgs::Float64 joint_command;



/*
  ROS_WARN("Tamanio de char: %lu \n",  sizeof(char));
  ROS_WARN("Tamanio de %lu int8_t \n",  sizeof(int8_t));
  ROS_WARN("Tamanio de %lu TEO_joints \n", sizeof(TEO_joints));
*/

  // Get the teotragen_com_port number from the ROS parameter server and assign it to port_download variable
  int teotragen_com_port;
  if (n.getParam("teotragen_com_port", teotragen_com_port)){
    ROS_INFO("Ready 'teotragen_com_port' parameter: %d \n", teotragen_com_port);
  }
  else {
    ROS_ERROR("Error reading 'teotragen_com_port' parameter \n");
    exit(-1);
  }

  // Open server socket with connection (TCP)
  if ((server_sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1 ) {  
    ROS_ERROR("Error opening server socket()\n");
    exit(-1);
  }

  // Fill server address structre
  server_addr.sin_family = AF_INET;         
  server_addr.sin_port = htons(teotragen_com_port); // Convert teotragen_com_port
  server_addr.sin_addr.s_addr = INADDR_ANY; // Macro "address of this machine"
  bzero(&(server_addr.sin_zero),8); // Fill other members with zeros

  // Notify O.S. and asociate socket with port
  if(bind(server_sockfd,(struct sockaddr*)&server_addr,
           sizeof(struct sockaddr))==-1) {
    ROS_ERROR("Error in bind(). Please wait some seconds before rerun \n");
    exit(-1);
  }

  // O.S. waits for connections
   if(listen(server_sockfd,BACKLOG) == -1) { 
      ROS_ERROR("Error in listen()\n");
      exit(-1);
   }

  // ROS
  while(n.ok()) {
    if (client_sockfd == -1) {
      //Ask O.S. for conections and accept one
      sin_size = sizeof(struct sockaddr_in);
      ROS_INFO("Waiting for TEOTraGen trajectories...");
      if ((client_sockfd = accept(server_sockfd,(struct sockaddr *)&client_addr,
                          &sin_size))==-1) {
        ROS_ERROR("Error in accept()\n");
      	close(server_sockfd);
        exit(-1);
      }
      ROS_INFO("Receiving trajectories from TEOTraGen ...");
      usleep(2000); // Wait 2 ms
    }
    else {
      if ((numbytes_received = recv(client_sockfd, buffer_received, sizeof(struct Humanoid_joints_structure), 0)) == -1){  
        printf("Error in recv() \n");
	      close(server_sockfd); close(client_sockfd);
        client_sockfd=-1;
        exit(-1);
      }
      if (numbytes_received == 0) {
        ROS_WARN("Comunication with TEOTraGen lost... \n");
        close(client_sockfd);
        client_sockfd=-1;

        // Temporal

        usleep(2000000); // Wait 2 s
        joint_command.data = 0;
        for (int ii=0; ii<14; ii++) {
            joint0_command_to_ROS.publish(joint_command);

            joint1_command_to_ROS.publish(joint_command);

            joint2_command_to_ROS.publish(joint_command);

            joint3_command_to_ROS.publish(joint_command);

            joint4_command_to_ROS.publish(joint_command);

            joint5_command_to_ROS.publish(joint_command);

            joint6_command_to_ROS.publish(joint_command);

            joint7_command_to_ROS.publish(joint_command);

            joint8_command_to_ROS.publish(joint_command);

            joint9_command_to_ROS.publish(joint_command);

            joint10_command_to_ROS.publish(joint_command);

            joint11_command_to_ROS.publish(joint_command);

            joint12_command_to_ROS.publish(joint_command);
        }


      }
      else {
        num_config_received += 1;
        TEO_joints = *(struct Humanoid_joints_structure *) &buffer_received;
        //printf("Recibido dato %lu con %d Bytes\n",num_config_received, numbytes_received);

        joints_to_send.header.stamp = ros::Time::now();
        //ROS_WARN("SF es %d TEO_joints.support_foot \n", TEO_joints.support_foot);

        joints_to_send.name.resize ( 0 );
        joints_to_send.position.resize ( 0 );
        joints_to_send.velocity.resize ( 0 );
        joints_to_send.effort.resize ( 0 );

        for (int ii=0; ii<26; ii++) {
          joint_command.data = TEO_joints.joints[ii];

          switch (ii) {
            case 0:
              joints_to_send.name.push_back("r_hip_yaw");
              joint0_command_to_ROS.publish(joint_command);
              break;
            case 1:
              joints_to_send.name.push_back("r_hip_roll");
              joint1_command_to_ROS.publish(joint_command);
              break;
            case 2:
              joints_to_send.name.push_back("r_hip_pitch");
              joint2_command_to_ROS.publish(joint_command);
              break;
            case 3:
              joints_to_send.name.push_back("r_knee_pitch");
              joint3_command_to_ROS.publish(joint_command);
              break;
            case 4:
              joints_to_send.name.push_back("r_ankle_pitch");
              joint4_command_to_ROS.publish(joint_command);
              break;
            case 5:
              joints_to_send.name.push_back("r_ankle_roll");
              joint5_command_to_ROS.publish(joint_command);
              break;
            case 6:
              joints_to_send.name.push_back("l_hip_yaw");
              joint6_command_to_ROS.publish(joint_command);
              break;
            case 7:
              joints_to_send.name.push_back("l_hip_roll");
              joint7_command_to_ROS.publish(joint_command);
              break;
            case 8:
              joints_to_send.name.push_back("l_hip_pitch");
              joint8_command_to_ROS.publish(joint_command);
              break;
            case 9:
              joints_to_send.name.push_back("l_knee_pitch");
              joint9_command_to_ROS.publish(joint_command);
              break;
            case 10:
              joints_to_send.name.push_back("l_ankle_pitch");
              joint10_command_to_ROS.publish(joint_command);
              break;
            case 11:
              joints_to_send.name.push_back("l_ankle_roll");
              joint11_command_to_ROS.publish(joint_command);
              break;
            case 12:
              joints_to_send.name.push_back("waist_yaw");
              joint12_command_to_ROS.publish(joint_command);
              break;
            case 13:
              joints_to_send.name.push_back("waist_pitch");
              break;
            case 14:
              joints_to_send.name.push_back("r_shoulder_pitch");
              break;
            case 15:
              joints_to_send.name.push_back("r_shoulder_roll");
              break;
            case 16:
              joints_to_send.name.push_back("r_shoulder_yaw");
              break;
            case 17:
              joints_to_send.name.push_back("r_elbow_pitch");
              break;
            case 18:
              joints_to_send.name.push_back("r_wrist_yaw");
              break;
            case 19:
              joints_to_send.name.push_back("r_wrist_pitch");
              break;
            case 20:
              joints_to_send.name.push_back("l_shoulder_pitch");
              break;
            case 21:
              joints_to_send.name.push_back("l_shoulder_roll");
              break;
            case 22:
              joints_to_send.name.push_back("l_shoulder_yaw");
              break;
            case 23:
              joints_to_send.name.push_back("l_elbow_pitch");
              break;
            case 24:
              joints_to_send.name.push_back("l_wrist_yaw");
              break;
            case 25:
              joints_to_send.name.push_back("l_wrist_pitch");
              break;
          }

            joints_to_send.position.push_back(TEO_joints.joints[ii]);
            joints_to_send.velocity.push_back(0.0);
            joints_to_send.effort.push_back(0.0);

        }
        support_foot_to_send.support_foot = TEO_joints.support_foot;

        // Publish to the ROS Topics
        configuration_to_ROS.publish(joints_to_send);
        support_foot_to_ROS.publish(support_foot_to_send);

      }
    }

    ros::spinOnce();

  }

  // Close the socket
  close(server_sockfd);
  close(client_sockfd); 

  return 0;
}


