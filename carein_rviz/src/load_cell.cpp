// Publisher and Subscriber
// Publish topic: loadcell
// Subscribw topic: loadcell_sub
// Usage: 
// $rosrun rviz rviz
// Set on rviz Fixed Frame:loadcell_frame
//             Add: Marker
//                  Topic: loadcell
// $rostopic pub /loadcell_sub carein_rviz/LoadCell 234 [4,3,2,0,0,5,0,0] --once
// $rosrun carein_rviz load_cell
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <carein_rviz/LoadCell.h>
#include <cmath>
#include <string>
#include "std_msgs/String.h"

class LoadCellHandler
{
private:
  //std::string tf_frame;

  ros::NodeHandle nh;
  ros::Publisher  pub;
  ros::Subscriber sub ;

  unsigned int star;
  unsigned int chksum;
  unsigned int type;
  unsigned int len;
  unsigned int secs_time_stamp;
  int          decipounds[8];

  int carein_total_weight; // total weight on both feet is 64lbs
  
  int total_weight[2];     // weight on each foot

  // left foot Blue (L), right foot Red(R)
  // Front of foot - F
  // Back of foot - B
  // Inside of foot - I
  // Outside of foot - O
  enum foot_type { LFI=0, LBO, LBI, RFI, RBO, LFO, RBI, RFO};
  enum foot_side { LEFT=0, RIGHT};

  // distance from center of the foot
  int foot_dist_x[8]; // for left and right foot
  int foot_dist_y[8]; // for left and right foot

  int foot_weight_x[8]; // for left and right foot
  int foot_weight_y[8]; // for left and right foot

  float center_of_mass_x[2]; // for left and right foot
  float center_of_mass_y[2]; // for left and right foot

  float radius[2];

public:
  // 
  // Constructor
  //
  LoadCellHandler() {
     carein_total_weight = 64;
     
     center_of_mass_x[LEFT] = 0.0;
     center_of_mass_y[LEFT] = 0.0;
 
     center_of_mass_x[RIGHT] = 0.0;
     center_of_mass_y[RIGHT] = 0.0;

     radius[LEFT]  = 0.0;
     radius[RIGHT] = 0.0;

     // left foot
     foot_dist_x[LBI] = -6;
     foot_dist_y[LBI] = -6;
     foot_dist_x[LFI] = -6;
     foot_dist_y[LFI] = 6;
     foot_dist_x[LBO] = 6;
     foot_dist_y[LBO] = -6;
     foot_dist_x[LFO] = 6;
     foot_dist_y[LFO] = 6;
 
     // right foot
     foot_dist_x[RBI] = 6;
     foot_dist_y[RBI] = -6;
     foot_dist_x[RFI] = 6;
     foot_dist_y[RFI] = 6;
     foot_dist_x[RBO] = -6;
     foot_dist_y[RBO] = -6;
     foot_dist_x[RFO] = -6;
     foot_dist_y[RFO] = 6;
  
     pub = nh.advertise<visualization_msgs::MarkerArray>("loadcell", 1);
     sub = nh.subscribe("loadcell_sub", 1000, &LoadCellHandler::loadcellCallback, this);

  } // LoadCellHandler()

  // 
  // Main loop to set and publish markers.
  //
  bool spin ()
  {
    ros::Rate r(30);
    float f = 0.0;

    while(nh.ok ())
    {

      visualization_msgs::MarkerArray cube_sphere_marker;

      cube_sphere_marker.markers.resize(4);

      cube_sphere_marker.markers[0].header.frame_id = cube_sphere_marker.markers[1].header.frame_id = cube_sphere_marker.markers[2].header.frame_id = cube_sphere_marker.markers[3].header.frame_id = "/loadcell_frame";

      cube_sphere_marker.markers[0].header.stamp = cube_sphere_marker.markers[1].header.stamp = cube_sphere_marker.markers[2].header.stamp = cube_sphere_marker.markers[3].header.stamp = ros::Time::now();
      cube_sphere_marker.markers[0].action = cube_sphere_marker.markers[1].action = cube_sphere_marker.markers[2].action = cube_sphere_marker.markers[3].action = visualization_msgs::Marker::ADD;
      cube_sphere_marker.markers[0].ns = cube_sphere_marker.markers[1].ns = cube_sphere_marker.markers[2].ns = cube_sphere_marker.markers[3].ns = "load_cell";
      cube_sphere_marker.markers[0].pose.orientation.w = cube_sphere_marker.markers[1].pose.orientation.w = cube_sphere_marker.markers[2].pose.orientation.w = cube_sphere_marker.markers[3].pose.orientation.w = 1.0;

      cube_sphere_marker.markers[0].id = 0; // left cube - blue
      cube_sphere_marker.markers[1].id = 1; // left sphere
      cube_sphere_marker.markers[2].id = 2; // right cube - red
      cube_sphere_marker.markers[3].id = 3; // right sphere
           
      cube_sphere_marker.markers[0].type = visualization_msgs::Marker::CUBE;
      cube_sphere_marker.markers[1].type = visualization_msgs::Marker::SPHERE;
      cube_sphere_marker.markers[2].type = visualization_msgs::Marker::CUBE;
      cube_sphere_marker.markers[3].type = visualization_msgs::Marker::SPHERE;
      
      // markers use x and y scale for width/height respectively
      // Each foot is 12"X12". Scaling it down to 4 cells in the grid of rviz
      cube_sphere_marker.markers[0].scale.x = 4;
      cube_sphere_marker.markers[0].scale.y = 4;
      cube_sphere_marker.markers[0].scale.z = 0.05; // make this less than sphere, so sphere can jot out
           
      cube_sphere_marker.markers[0].color.b = 1.0; // LEFT CUBE is blue
      cube_sphere_marker.markers[0].color.a = 1.0;

      cube_sphere_marker.markers[1].scale.x = radius[LEFT];
      cube_sphere_marker.markers[1].scale.y = radius[LEFT];
      cube_sphere_marker.markers[1].scale.z = 0.1;      
      
      cube_sphere_marker.markers[1].color.g = 1.0f; // green sphere
      cube_sphere_marker.markers[1].color.a = 1.0;

      // left foot: center of square is 0,0 for circle so the offset is 3 cells on x-axis, (x,y)=(-3,0) on sphere == 0,0 on grid
      cube_sphere_marker.markers[1].pose.position.x = -(center_of_mass_x[LEFT]*1.0/3 + 3); // divisible by 3: each grid cell=3 points with total of 12 points for 12" of foot
      cube_sphere_marker.markers[1].pose.position.y = center_of_mass_y[LEFT]*1.0/3;
      cube_sphere_marker.markers[1].pose.position.z = 0;
      
      cube_sphere_marker.markers[0].pose.position.x = -3;
      cube_sphere_marker.markers[0].pose.position.y = 0;
      cube_sphere_marker.markers[0].pose.position.z = 0;
      
      // markers use x and y scale for width/height respectively
      cube_sphere_marker.markers[2].scale.x = 4;
      cube_sphere_marker.markers[2].scale.y = 4;
      cube_sphere_marker.markers[2].scale.z = 0.05; // make this less than sphere, so sphere can jot out
      
      cube_sphere_marker.markers[2].color.r = 1.0; // RIGHT CUBE Red
      cube_sphere_marker.markers[2].color.a = 1.0;

      cube_sphere_marker.markers[3].scale.x = radius[RIGHT];
      cube_sphere_marker.markers[3].scale.y = radius[RIGHT];
      cube_sphere_marker.markers[3].scale.z = 0.1;
      
      cube_sphere_marker.markers[3].color.g = 1.0f; // green sphere
      cube_sphere_marker.markers[3].color.a = 1.0;

      // right foot: center of square is 0,0 for circle so the offset is 3 cells on x-axis, (x,y)=(-3,0) on sphere == 0,0 on grid
      cube_sphere_marker.markers[3].pose.position.x = (center_of_mass_x[RIGHT]*(-1.0)/3 + 3); // make x as -x, divisible by 3: each grid cell=3 points with total of 12 points for 12" of foot
      cube_sphere_marker.markers[3].pose.position.y = center_of_mass_y[RIGHT]*1.0/3;
      cube_sphere_marker.markers[3].pose.position.z = 0;
      
      cube_sphere_marker.markers[2].pose.position.x = 3;
      cube_sphere_marker.markers[2].pose.position.y = 0;
      cube_sphere_marker.markers[2].pose.position.z = 0;

      
      // Publish the marker
      while (pub.getNumSubscribers() < 1)
      {
         if (!ros::ok())
         {
           return 0;
         }
         ROS_WARN_ONCE("Please create a subscriber to the marker");
         sleep(1);
      }

      pub.publish(cube_sphere_marker);    
            
      ros::spinOnce();

      r.sleep();
    }
    return (true);
  } // spin()

  //
  // callback function for subscriber
  //
  void loadcellCallback(const carein_rviz::LoadCell::ConstPtr& msg)
  {
    //ROS_INFO("I heard: [%s]", msg->data.c_str());
     
     secs_time_stamp = msg->secs_time_stamp;

     decipounds[LFI] = msg->decipounds[LFI]; // LFI 
     decipounds[LBO] = msg->decipounds[LBO]; // LBO 
     decipounds[LBI] = msg->decipounds[LBI]; // LBI 
     decipounds[RFI] = msg->decipounds[RFI]; // RFI
     decipounds[RBO] = msg->decipounds[RBO]; // RBO
     decipounds[LFO] = msg->decipounds[LFO]; // LFO
     decipounds[RBI] = msg->decipounds[RBI]; // RBI
     decipounds[RFO] = msg->decipounds[RFO]; // RFO
     ROS_INFO("I heard: [%d %d %d %d %d %d %d %d ]", decipounds[LFI], decipounds[LBO] ,decipounds[LBI] ,decipounds[RFI], decipounds[RBO], decipounds[LFO], decipounds[RBI], decipounds[RFO]);
          
     // left foot: some number scaled to its position and weight
     foot_weight_x[LBI] = foot_dist_x[LBI] * decipounds[LBI];
     foot_weight_y[LBI] = foot_dist_y[LBI] * decipounds[LBI];
     foot_weight_x[LFI] = foot_dist_x[LFI] * decipounds[LFI];
     foot_weight_y[LFI] = foot_dist_y[LFI] * decipounds[LFI];
     foot_weight_x[LBO] = foot_dist_x[LBO] * decipounds[LBO];
     foot_weight_y[LBO] = foot_dist_y[LBO] * decipounds[LBO];
     foot_weight_x[LFO] = foot_dist_x[LFO] * decipounds[LFO];
     foot_weight_y[LFO] = foot_dist_y[LFO] * decipounds[LFO];
     ROS_INFO("LEFT foot_weight_x: [%d %d %d %d ]", foot_weight_x[LFI], foot_weight_x[LBO] ,foot_weight_x[LBI] , foot_weight_x[LFO]);
     ROS_INFO("LEFT foot_weight_y: [%d %d %d %d ]", foot_weight_y[LFI], foot_weight_y[LBO] ,foot_weight_y[LBI] , foot_weight_y[LFO]);
    
     total_weight[LEFT] = decipounds[LFI] + decipounds[LBO] + decipounds[LBI] + decipounds[LFO];
     ROS_INFO("LEFT total_weight: [%d]", total_weight[LEFT]);

     if (total_weight[LEFT] > 0) {
        center_of_mass_x[LEFT] = (foot_weight_x[LBI] + foot_weight_x[LFI] + foot_weight_x[LBO] + foot_weight_x[LFO])*1.0/total_weight[LEFT]; // convert to float (*1.0)
        center_of_mass_y[LEFT] = (foot_weight_y[LBI] + foot_weight_y[LFI] + foot_weight_y[LBO] + foot_weight_y[LFO])*1.0/total_weight[LEFT]; 
     }
     else {
        center_of_mass_x[LEFT] = 0.0; // convert to float (*1.0)
        center_of_mass_y[LEFT] = 0.0; 
     }
     

     ROS_INFO("LEFT COM_x:COM_y: [%f %f]",center_of_mass_x[LEFT], center_of_mass_y[LEFT]);
    
     radius[LEFT] = total_weight[LEFT]*1.0/carein_total_weight;
     ROS_INFO("LEFT radius:carein_total_weight: [%f %d]",radius[LEFT], carein_total_weight);

     // right foot: some number scaled to its position and weight
     foot_weight_x[RBI] = foot_dist_x[RBI] * decipounds[RBI];
     foot_weight_y[RBI] = foot_dist_y[RBI] * decipounds[RBI];
     foot_weight_x[RFI] = foot_dist_x[RFI] * decipounds[RFI];
     foot_weight_y[RFI] = foot_dist_y[RFI] * decipounds[RFI];
     foot_weight_x[RBO] = foot_dist_x[RBO] * decipounds[RBO];
     foot_weight_y[RBO] = foot_dist_y[RBO] * decipounds[RBO];
     foot_weight_x[RFO] = foot_dist_x[RFO] * decipounds[RFO];
     foot_weight_y[RFO] = foot_dist_y[RFO] * decipounds[RFO];
     ROS_INFO("RIGHT foot_weight_x: [%d %d %d %d ]", foot_weight_x[RFI], foot_weight_x[RBO], foot_weight_x[RBI], foot_weight_x[RFO]);
     ROS_INFO("RIGHT foot_weight_y: [%d %d %d %d ]", foot_weight_y[RFI], foot_weight_y[RBO], foot_weight_y[RBI], foot_weight_y[RFO]);

     total_weight[RIGHT] = decipounds[RFI] +decipounds[RBO] + decipounds[RBI] + decipounds[RFO] ;
     ROS_INFO("RIGHT total_weight: [%d]", total_weight[RIGHT]);
     
     if (total_weight[RIGHT] > 0) {
        center_of_mass_x[RIGHT] = (foot_weight_x[RBI] + foot_weight_x[RFI] + foot_weight_x[RBO] + foot_weight_x[RFO])*1.0/total_weight[RIGHT]; // convert to float (*1.0)
        center_of_mass_y[RIGHT] = (foot_weight_y[RBI] + foot_weight_y[RFI] + foot_weight_y[RBO] + foot_weight_y[RFO])*1.0/total_weight[RIGHT]; 
     }
     else {
        center_of_mass_x[RIGHT] = 0.0; // convert to float (*1.0)
        center_of_mass_y[RIGHT] = 0.0; 
     }

     ROS_INFO("RIGHT COM_x:COM_y: [%f %f]",center_of_mass_x[RIGHT], center_of_mass_y[RIGHT]);
    
     radius[RIGHT] = (total_weight[RIGHT])*1.0/carein_total_weight; 
     ROS_INFO("RIGHT radius: [%f]",radius[RIGHT]);

  } // loadcellCallback()

  bool waitForSubscribers(ros::Publisher & pub, ros::Duration timeout)
  {
    if(pub.getNumSubscribers() > 0)
        return true;
    ros::Time start = ros::Time::now();
    ros::Rate waitTime(0.5);
    while(ros::Time::now() - start < timeout) {
        waitTime.sleep();
        if(pub.getNumSubscribers() > 0)
            break;
    }
    return pub.getNumSubscribers() > 0;
  } // waitForSubscribers()

};

int main(int argc, char* argv[])
{
  ros::init (argc, argv, "load_cell");
 
  LoadCellHandler loadcell_handler;
  loadcell_handler.spin();

  return 0;
}
