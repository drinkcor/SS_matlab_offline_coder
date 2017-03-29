#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/time.h> 
#include <std_msgs/Float64.h>
#include <baxter_core_msgs/EndpointState.h>
using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc,argv,"force_publisher"); 	// name of this node will be "force_publisher"
  ros::NodeHandle n; 				// two lines to create a publisher object that can talk to ROS
  ros::Publisher force_publisher_object = n.advertise<baxter_core_msgs::EndpointState>("/robot/limb/right/endpoint_state",1);

  //"/robot/limb/right/endpoint_state" is the name of the topic to which we will publish
  // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
  baxter_core_msgs::EndpointState input_force;		//create a variable of type "EndpointState", as defined in: /opt/ros/fuerte/share/std_msgs

  // any message published on a ROS topic must have a pre-defined format, so subscribers know how to
  // interpret the serialized data transmission
  input_force.header.stamp   = ros::Time(0.0);
  input_force.wrench.force.x = 0.0;
  input_force.wrench.force.y = 0.0;
  input_force.wrench.force.z = 0.0;
  input_force.wrench.torque.x = 0.0; 
  input_force.wrench.torque.y = 0.0;
  input_force.wrench.torque.z = 0.0;


  fstream f;
  f.open(argv[1],ios::in);

  int index,j;
  float force[3000][7];
  
  index=0;   

  while(!f.eof()){
    for (j=0; j<7; j++){
      f>>force[index][j];
      cout<<force[index][j]<<" ";
    }
    cout<<endl;
    index++;
  }
  f.close();



  // ROS Rates
  ros::Rate loopRate(200); 	       		// Create a ros::Rate object. Set the time for a 1Hz sleeper timer.
  int i = 0;
  while(ros::ok()&&i<=index) 				// do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    {
      // this loop has no sleep timer, and thus it will consume excessive CPU time
      // expect one core to be 100% dedicated (wastefully) to this small task

      input_force.header.stamp   = ros::Time(force[i][0]);
      input_force.wrench.force.x = force[i][1];
      input_force.wrench.force.y = force[i][2];
      input_force.wrench.force.z = force[i][3];
      input_force.wrench.torque.x = force[i][4]; 
      input_force.wrench.torque.y = force[i][5];
      input_force.wrench.torque.z = force[i][6];
      i++;

      force_publisher_object.publish(input_force);  // publish the value--of type EndpointState-- to the topic "topic1"
      loopRate.sleep();				// The call to sleep makes this node's while loop to be called every 1Hz.
    }

  return 0;
  
}
