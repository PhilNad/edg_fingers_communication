#include <string>
#include <math.h>
#include <vector>
#include <array>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <chrono>
#include "ros/ros.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"

using namespace std;

//Used to record the data read from the serial line.
vector<char> serialDataBuffer;

//Recordings of the preivous positions, see below.
int previous_finger1_pos, previous_finger2_pos;
//This timestamp is used to calculate the derivative of the position in order
//to get the velocity.
chrono::high_resolution_clock::time_point data_read_timestamp;

//The node handle needs to be global so its accessible in all the functions
//Needs to be instantiated in the main once ros::init is done
ros::NodeHandle *thisNode;

//This is the handle to the publishers that are going to be used.
ros::Publisher finger1_position_pub;
ros::Publisher finger1_velocity_pub;
ros::Publisher finger1_pressure_pub;
ros::Publisher finger2_position_pub;
ros::Publisher finger2_velocity_pub;
ros::Publisher finger2_pressure_pub;

//Publishes onto ROS topics the data extracted from the serial communication.
void publishData(uint16_t finger1_pos, uint16_t finger1_pressure, uint16_t finger2_pos, uint16_t finger2_pressure){
  std_msgs::UInt16 msg_position;
  std_msgs::UInt16 msg_pressure;
  std_msgs::Float32 msg_velocity;

  //Time in microseconds since the last time data was processed.
  chrono::high_resolution_clock::time_point now = chrono::high_resolution_clock::now();
  chrono::duration<double> time_span            = chrono::duration_cast<chrono::duration<double>>(now - data_read_timestamp);

  msg_position.data = finger1_pos;
  finger1_position_pub.publish(msg_position);

  msg_pressure.data = finger1_pressure;
  finger1_pressure_pub.publish(msg_pressure);

  //We calculate the velocity by dividing the position difference by the time difference
  //and interpreting it in seconds instead of microseconds.
  msg_velocity.data = (float) (finger1_pos-previous_finger1_pos)/time_span.count();
  finger1_velocity_pub.publish(msg_velocity);

  msg_position.data = finger2_pos;
  finger2_position_pub.publish(msg_position);

  msg_pressure.data = finger2_pressure;
  finger2_pressure_pub.publish(msg_pressure);

  //We calculate the velocity by dividing the position difference by the time difference
  //and interpreting it in seconds instead of microseconds.
  msg_velocity.data = (float) (finger2_pos-previous_finger2_pos)/time_span.count();
  finger2_velocity_pub.publish(msg_velocity);

  //Overwrite the previous positions with these ones.
  previous_finger1_pos = finger1_pos;
  previous_finger2_pos = finger2_pos;
}

//This function tries to parse the data accumulated in the buffer and
//publishes it on ROS topics so other nodes can read it.
void processData(){
    //The Arduino Uno's integer is made of 2 bytes, the Most Significant Byte (MSB)
    //and the Least Significant Byte (LSB). As four integers are sent separated by
    //commas (1 byte each), the whole message should be 4*2+3=11 bytes long.
    if(serialDataBuffer.size() == 11){
        //A valid message should have commas at these positions
        if(serialDataBuffer[2] == ',' && serialDataBuffer[5] == ',' && serialDataBuffer[8] == ',' ){
            char MSB = 0;
            char LSB = 0;

            //Intepret the MSB and LSB as a single 16 bits integer.
            //To do so, we take the MSB and shift it 8 bits to the left and
            //then use a OR bitwise operation to set the LSB.
            MSB = serialDataBuffer[0];
            LSB = serialDataBuffer[1];
            uint16_t finger1_pos      = (MSB<<8) | LSB;

            MSB = serialDataBuffer[3];
            LSB = serialDataBuffer[4];
            uint16_t finger2_pos      = (MSB<<8) | LSB;

            MSB = serialDataBuffer[6];
            LSB = serialDataBuffer[7];
            uint16_t finger1_pressure = (MSB<<8) | LSB;

            MSB = serialDataBuffer[9];
            LSB = serialDataBuffer[10];
            uint16_t finger2_pressure = (MSB<<8) | LSB;

            //ROS_INFO("Successfully extracted data elements from serial communication.");
            publishData(finger1_pos, finger1_pressure, finger2_pos, finger2_pressure);

        }else{ROS_WARN("Received message is malformated.");}
    }else{ROS_WARN("Received message's length is incorrect.");}
}

int main(int argc, char **argv){
    //The first argument of the service must represent the file descriptor of the
    // TTY being used. Most of the time, it is /dev/ttyACM0 but it can be mapped
    // to /dev/ttyACM1, for example, if the RObotiq 2f-140 gripper is plugged in
    // and is already using that file path.
    if(argc != 2){
      ROS_ERROR("The TTY path must be given as the sole argument.");
      return 1;
    }
    char* serialTtyPath = argv[1];
    FILE* file_pointer;
    file_pointer = fopen(serialTtyPath,"r");

    //Initialize the ROS node
    ros::init(argc, argv, "edg_fingers_com_node");

    //Instantiate the handle, needs to be done after ros::init
    thisNode = new ros::NodeHandle();

    //Publish the data on these ROS topics
    finger1_position_pub  = thisNode->advertise<std_msgs::UInt16>("fingers/1/position", 1);
    finger1_velocity_pub  = thisNode->advertise<std_msgs::Float32>("fingers/1/velocity", 1);
    finger1_pressure_pub  = thisNode->advertise<std_msgs::UInt16>("fingers/1/pressure", 1);

    finger2_position_pub  = thisNode->advertise<std_msgs::UInt16>("fingers/2/position", 1);
    finger2_velocity_pub  = thisNode->advertise<std_msgs::Float32>("fingers/2/velocity", 1);
    finger2_pressure_pub  = thisNode->advertise<std_msgs::UInt16>("fingers/2/pressure", 1);

    ROS_INFO("Ready to listen for any EDG data on the serial line.");

    //Initialize the timestamp.
    data_read_timestamp = chrono::high_resolution_clock::now();


    //If ROS is being closed by the user, we want to stop this process.
    while(ros::ok()){
        //Reads the content of the file descriptor of the Serial communication
        if(file_pointer){
          char byteRead = fgetc(file_pointer);
          //If nothing can be read at the moment, byteRead will equal -1 or EOF
          if(byteRead >= 0){
            //Upon the reception of a newline character, we process the serial data.
            if(byteRead == '\n'){
              //Publishes the accumulated data on ROS topics.
              processData();
              //Remove all accumulated data from the buffer.
              serialDataBuffer.clear();
            }else{
              //Append the byte read to our buffer
              serialDataBuffer.push_back(byteRead);
            }
        }
      }
    }

    //Close the file before exiting the program.
    if(file_pointer)
      fclose(file_pointer);

    return 0;
}
