/***
*   This controller does the thruster identification data acquisiton.
*     It will drive the thruster through the complete 'positive' range.
*     For the inverse a second run is necessary, limitation is the BBB ADC
*     The driver will keep a thrust comand until the force feedback signal is stable.
*     The status will be published on a ros topic, with a flag if it was stable.
*
*   raphael.nagel (#) posteo de
*   23/Sept/2014
*///////////////////////////////////////////////////////////////////////


#include <controller_interface/controller.h>
#include <controller_manager_msgs/SwitchController.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>

#include "geometry_msgs/Vector3.h"

#include <labust/math/NumberManipulation.hpp>
#include <ros_control_iso/relay_with_hysteresis.hpp>


#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

namespace ros_control_iso{

  /** init() gets called when the relay ros_controller is being loaded
  *
  *
  * \author Raphael Nagel
  * \date 23/Sept/2014
  **************************************** */
  bool thruster_ident_driver::init(hardware_interface::EffortJointInterface* hw, ros::NodeHandle &n)
  {

    finished = FALSE;
    stable = FALSE;

    ///set the output to 0;
    joint_.setCommand(0);


    // get joint name from the parameter server
    if (!n.getParam("/ros_control_iso/joint", my_joint)){
      ROS_ERROR("ros_control - thruster_ident_driver: Could not find joint name\n");
      return EXIT_FAILURE;
    }

    // get the joint object to use in the realtime loop
    try{
    joint_ = hw->getHandle(my_joint);  // throws on failure

    }
    catch (...) {
      ROS_ERROR("ros_control - thruster_ident_driver: Exception happened - Could not get handle of the joint");
    }


    /// find the calibration length
    if (!n.getParam("/ros_control_iso/thruster_ident_driver/calibration_length", calibration_length)){
      ROS_ERROR("ros_control - thruster_ident_driver: Could not find calibration_length, assuming 10s\n");
      calibration_length = 10;
      n.setParam("/ros_control_iso/thruster_ident_driver/calibration_length", calibration_length);
    }

    /// find the update_rate
    if (!n.getParam("/ros_control_iso/thruster_ident_driver/update_rate", update_rate)){
      ROS_ERROR("ros_control - thruster_ident_driver: Could not find update_rate, assuming 50Hz\n");
      update_rate = 50;
      n.setParam("/ros_control_iso/thruster_ident_driver/update_rate", update_rate);
    }  

    ///calculate the number of calibration samples:
    calibration_length_num_of_samples = calibration_length * update_rate;
  
    /// find out if we are driving forward or backwards
    if (!n.getParam("/ros_control_iso/thruster_ident_driver/direction", direction)){
      ROS_ERROR("ros_control - thruster_ident_driver: Could not find direction\n");
      return EXIT_FAILURE;
    }   

  
    /// Find the command list
    if (!n.getParam("/ros_control_iso/thruster_ident_driver/command_list", command_list)){
      ROS_ERROR("ros_control - thruster_ident_driver: Could not find command_list\n");
      return EXIT_FAILURE;
    }   

    /// find the acquisiton length
    if (!n.getParam("/ros_control_iso/thruster_ident_driver/min_acquisiton_length", min_acquisiton_length)){
      ROS_ERROR("ros_control - thruster_ident_driver: Could not find min_acquisiton_length, assuming 20s\n");
      min_acquisiton_length = 50;
      n.setParam("/ros_control_iso/thruster_ident_driver/min_acquisiton_length", min_acquisiton_length);
    }       
    
    thruster_ident_driver::rolling_average_setup();


    /// Start realtime state publisher
    ROS_INFO("ros_control - thruster_ident_driver: Loaded all parameters, starting the realtime publisher.\n");
    controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<control_msgs::JointControllerState>(n, "thruster_ident_data", 1) );
    ROS_INFO("ros_control - ros_control_iso: RealtimePublisher started\n");

    ///Starting the subscriber
    subscriber = n.subscribe<geometry_msgs::Vector3>("thruster_ident_force", 20, &thruster_ident_driver::subs_callback, this);

    /// start in calibration mode
    calibration_flag = TRUE;
    while(~calibration_complete){
        sleep(1);
    }

    return EXIT_SUCCESS;
  }

  /** rolling_average_callback(...) setup the boost accumulator
  *
  *
  * \author Raphael Nagel
  * \date 23/Sept/2014
  **************************************** */
  void thruster_ident_driver::rolling_average_setup(void){
  unsigned int r_mean_window_size;
  /// find the update_rate
  if (!n.getParam("/ros_control_iso/thruster_ident_driver/r_mean_window_size", r_mean_window_size)){
    ROS_ERROR("ros_control - thruster_ident_driver: Could not find r_mean_window_size, assuming 50\n");
    r_mean_window_size = 50;
    n.setParam("/ros_control_iso/thruster_ident_driver/r_mean_window_size", r_mean_window_size);
  }  

    r_mean( tag::rolling_window::window_size = r_mean_window_size);

    old_rolling_mean = 0;
    new_rolling_mean = 0;

  };


  /** subs_callback(...) acts on incoming messages
  *
  *
  * \author Raphael Nagel
  * \date 23/Sept/2014
  **************************************** */
  void thruster_ident_driver::subs_callback(const geometry_msgs::Vector3::ConstPtr& message){
    static num_of_samples = 0;
    static accumulator = 0;

    ///run the calibration routine
    if(calibration_flag == TRUE){
      accumulator = accumulator + message->x;
    
      ///End the calibration once enough samples, calc the offset
      if(num_of_samples >= calibration_length_num_of_samples){
        offset = accumulator/num_of_samples;
        calibration_complete = TRUE;
      }

      num_of_samples ++;
    
    ///run the normal routine with stability test
    }else{
      ADC_data = message->x;

      thruster_ident_driver::stable_average();
    }


  }

  void thruster_ident_driver::stable_average(){
    r_mean(ADC_data);

    if(update_counter >= (unsigned int) (min_acquisiton_length / update_rate) ){
      if( abs(ADC_data - rolling_mean(r_mean) <= 20){
        stable == TRUE;
      }
    }

  }




  /** update(...) this is run everytime the controller updates
  *
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  void thruster_ident_driver::update(const ros::Time& time, const ros::Duration& period){
    //ROS_INFO("ros_control - ros_control_iso: Updating the controller output.\n");
    int command_out = 0;

    ///act on finished state
    if(finished == TRUE){

      //Unload the relay
      controller_manager_msgs::SwitchController switcher;
      switcher.request.stop_controllers.push_back("thruster_ident_driver");
      switcher.request.strictness = STRICT; //STRICT==2
      ros::service::call("/controller_manager/switch_controller", switcher);    
    }else 
    ///response has stabilised --> change demand
    if(stable == TRUE){
      demand_list_index ++:
      stable == FALSE;

      ///if we have finished all commands, prevent overflow and set finished flag
      if(demand_list_index >= (command_list.size() - 1) ){
        demand_list_index = (command_list.size() - 1);
        finished == TRUE;
      }
      update_counter = 0;
    }
    


    ///Send the next command
    if(direction == FORWARD){
      command_out = command_list[demand_list_index];
      joint_.setCommand(command_out);
    }else if(direction == BACKWARD){
      command_out = command_list[demand_list_index] * -1;
      joint_.setCommand(command_out);
    }



    ///Publish the current state
    if(controller_state_publisher_ && controller_state_publisher_->trylock())
        {
          controller_state_publisher_->msg_.header.stamp = ros::Time::now();

          controller_state_publisher_->msg_.vector.x = command_out;
          controller_state_publisher_->msg_.vector.y = (ADC_data - offset);
          controller_state_publisher_->msg_.vector.z = stable;
          controller_state_publisher_->unlockAndPublish();
        }    
  }


  /** starting() is called when the controller starts, it resets the variables to be identified
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  void thruster_ident_driver::starting(const ros::Time& time) { 
    int command_out;

    ROS_INFO("ros_control - thruster_ident_driver: starting the controller. \n");

   
    finished = FALSE;
    stable = FALSE;
    demand_list_index = 0;


    ///Send the first command
    if(direction == FORWARD){
      command_out = command_list[demand_list_index];
      joint_.setCommand(command_out);
    }else if(direction == BACKWARD){
      command_out = command_list[demand_list_index] * -1;
      joint_.setCommand(command_out);
    }


    ROS_INFO("ros_control - thruster_ident_driver: driving now\n");
  }



  /** stopping() gets called when the controller is being stopped, it sets output to 0
  * 
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  void thruster_ident_driver::stopping(const ros::Time& time) { 
    joint_.setCommand(0);
    ROS_INFO("ros_control - thruster_ident_driver: Finished the thruster identification data acquisiton, output set to 0\n");
  }


  
}//end of namespace

PLUGINLIB_EXPORT_CLASS( ros_control_iso::relay_with_hysteresis, controller_interface::ControllerBase)
