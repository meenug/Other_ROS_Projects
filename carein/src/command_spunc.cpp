/**
 * Node Type: Publisher and Subscriber
 * Publishes a mesage at a time interval.
 * 
 * Node name: command_spunv
 * Subscriber Topic: cmd_spunc
 * Publisher Topic: cmd_term
 * Run:
 *   rosrun <pkg> command_spunc
 *   e.g. rosrun carein command_spunc
 *
*/
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


// Subscriber CAllback function
void messageCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("[term to spunc %s]", msg->data.c_str());
} // messageCallback()

int main(int argc, char **argv)
{
  ros::init(argc, argv, "command_spunc");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("cmd_spunc", 10, messageCallback);  // TODO: subscribe to spunc topic
  ros::Publisher pub = n.advertise<std_msgs::String>("cmd_term", 1); // TODO: publish to term topic

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;
   std::stringstream ss;
   ss << "hello from spunc ";
    msg.data = ss.str();


    if (count > 100){

       pub.publish(msg);
       count = 0;
    }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
} //main()

