#include <fstream>
#include <iostream>
#include <unistd.h>
#include <ros/ros.h>
#include <baxter_core_msgs/EndpointState.h>
using namespace std;
int main(int argc, char **argv)
{
  ros::init(argc,argv,"force_publisher"); 	// name of this node will be "force_publisher"
  ros::NodeHandle n; 				// two lines to create a publisher object that can talk to ROS
  ros::Publisher force_publisher_object = n.advertise<baxter_core_msgs::EndpointState>("topic1",1);
  //"topic1" is the name of the topic to which we will publish
  // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
  baxter_core_msgs::EndpointState input_force;		//create a variable of type "EndpointState", as defined in: /opt/ros/fuerte/share/std_msgs
  // any message published on a ROS topic must have a pre-defined format, so subscribers know how to
  // interpret the serialized data transmission
  input_force.wrench.force.x = 0.0;
  input_force.wrench.force.y = 0.0;
  input_force.wrench.force.z = 0.0;
  input_force.wrench.torque.x = 0.0; 
  input_force.wrench.torque.y = 0.0;
  input_force.wrench.torque.z = 0.0;

   char buf[1000];
   getcwd(buf,1000);  //得到当前工作路径
   cout<<buf<<endl;

  // Read Torque data
  fstream f;
  f.open("Torques.dat",ios::in);
  int i,j,k,notneed;
  bool finished_read = false;
  double force[1365][7];
  for (i=0; i<1365; i++){
      for (j=0; j<7; j++){
	    f>>force[i][j];
      }
      if(i==1364)    finished_read=true;
  };
  f.close();

  // ROS Rates
  ros::Rate loopRate(50); 	       		// Create a ros::Rate object. Set the time for a 1Hz sleeper timer.
  k=0;
  while(ros::ok()&&finished_read&&k<1365) 				// do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    {
      // this loop has no sleep timer, and thus it will consume excessive CPU time
      // expect one core to be 100% dedicated (wastefully) to this small task


      input_force.wrench.force.x = force[k][1];
      input_force.wrench.force.y = force[k][2];
      input_force.wrench.force.z = force[k][3];
      input_force.wrench.torque.x = force[k][4]; 
      input_force.wrench.torque.y = force[k][5];
      input_force.wrench.torque.z = force[k][6];
      k++;

      force_publisher_object.publish(input_force);  // publish the value--of type EndpointState-- to the topic "topic1"
      loopRate.sleep();				// The call to sleep makes this node's while loop to be called every 1Hz.
    }
}
