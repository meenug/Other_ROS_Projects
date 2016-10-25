#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * Node Type: Publisher and Subscriber
 * Create a dumb terminal and get user input and publish it to a topic
 * Any string published on this subscriber is displayed on the dumb terminal
 * 
 * Node name: command_term
 * Subscriber Topic: cmd_term
 * Publisher Topic: cmd_spunc
 * Run:
 *   rosrun <pkg> command_term
 *   e.g. rosrun carein command_term
 *
 * to change the format of the output with ROS_INFO use env. var. Default is-
 * export ROSCONSOLE_FORMAT='[${severity}] [${time}]: ${message}'
 */


// CAllback function
// Display the messge on the console
void messageCallback(const std_msgs::String::ConstPtr& msg)
{
  //std::cout<< "[" << msg->data << "]" << std::endl;
  ROS_INFO("[%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "command_term");

  ros::NodeHandle n;

  // subscriber
  ros::Subscriber sub = n.subscribe("cmd_term", 10, messageCallback);  // TODO: subscribe to term topic
  // publisher
  // publishing a message of type std_msgs/String on the topic cmd_spunc
  // since we are not publishing fast, we can have a buffer of 1 message only
  ros::Publisher pub = n.advertise<std_msgs::String>("cmd_spunc", 1); // TODO: publish to spunc topic

  ros::Rate loop_rate(10);

  
  std::cout<< "Press Ctrl+C to exit." << std::endl;
  while (ros::ok())
  {
    /**
     * message object. stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::string input_line;
    std::cout << "> ";
    std::cin.clear(); 
    // creating a asynchronous spin so that it does not hold on and wait on user input, as we are getting messages on topic cmd_term that needs to be printed.
    //getline(cin, ss);
    ros::AsyncSpinner spinner(1); // Use 1 threads
    //start the thread
    spinner.start(); 
    // get user input
    std::getline(std::cin, input_line);

    // stop the thread
    spinner.stop(); 
    msg.data = input_line;

    // publish the mesage
    pub.publish(msg);

    // call the callback func.
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

