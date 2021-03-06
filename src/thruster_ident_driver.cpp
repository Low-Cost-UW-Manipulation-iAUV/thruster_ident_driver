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

#include <thruster_ident_driver/thruster_ident_driver.hpp>



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
    calibration_complete = FALSE;
    ADC_data = 0;

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


    ///find the lever_factor
    if (!n.getParam("/ros_control_iso/thruster_ident_driver/lever_factor", lever_factor)){
      ROS_ERROR("ros_control - thruster_ident_driver: Could not find lever_factor, assuming 1\n");
      lever_factor = 1;
      n.setParam("/ros_control_iso/thruster_ident_driver/lever_factor", lever_factor);
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
      ROS_ERROR("ros_control - thruster_ident_driver: Could not find min_acquisiton_length, assuming 30s\n");
      min_acquisiton_length = 30;
      n.setParam("/ros_control_iso/thruster_ident_driver/min_acquisiton_length", min_acquisiton_length);
    }       
    

    /// Start realtime state publisher
    ROS_INFO("ros_control - thruster_ident_driver: Loaded all parameters, starting the realtime publisher.\n");
    controller_state_publisher_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::Vector3Stamped>(n, "thruster_ident_data", 1) );
    ROS_INFO("ros_control - thruster_ident_driver: RealtimePublisher started\n");

    ///Starting the subscriber
    subscriber = n.subscribe<geometry_msgs::Vector3>("/thruster_ident_adc", 20, &thruster_ident_driver::subs_callback, this);



    return 1; //It does not like EXIT_SUCCESS...
  }



  /** subs_callback(...) acts on incoming messages
  *
  *
  * \author Raphael Nagel
  * \date 23/Sept/2014
  **************************************** */
  void thruster_ident_driver::subs_callback(const geometry_msgs::Vector3::ConstPtr& message){
    ///run the calibration routine
    if(calibration_complete == FALSE){
      static unsigned int num_of_samples = 1;
      static unsigned int accumulator = 0;      
      accumulator = accumulator + message->x;
    
      ///End the calibration once enough samples, calc the offset
      if(num_of_samples >= calibration_length_num_of_samples){
        ROS_INFO("before offset = accumulator/num_of_samples;");

        offset = accumulator/num_of_samples;
        ros::param::set("/ros_control_iso/thruster_ident_driver/signal_offset", offset);

        calibration_complete = TRUE;
        ROS_INFO("ros_control_iso - thruster_ident_driver: Starting thruster with a demand of %f",command_list[demand_list_index]);

      }
      ROS_INFO("ros_control_iso - thruster_ident_driver: num_of_samples: %d  of calibration_length_num_of_samples: %d\n",num_of_samples, calibration_length_num_of_samples);
      num_of_samples ++;
    
    ///run the normal routine with stability test
    }else{
      ADC_data = message->x;

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
    
    ///act on finished state
    if(finished == TRUE){

      //Unload the relay
      controller_manager_msgs::SwitchController switcher;
      switcher.request.stop_controllers.push_back("/ros_control_iso/thruster_ident_driver");
      switcher.request.strictness = STRICT; //STRICT==2
      ros::service::call("/controller_manager/switch_controller", switcher);    

    }else 
    ///response has stabilised --> change demand
    if(stable == TRUE){
      demand_list_index ++;
      stable = FALSE;
      ROS_INFO("ros_control_iso - thruster_ident_driver: Changing demand to %f",command_list[demand_list_index]);
      ///if we have finished all commands, prevent overflow and set finished flag
      if(demand_list_index > (command_list.size() - 1) ){
        demand_list_index = (command_list.size() - 1);
        finished = TRUE;
      }
    }
    

    /// start in calibration mode
  
    if(calibration_complete == TRUE){
      double command_out = 0;

      ///Send the next command
      if(direction == FORWARD){   

        command_out = command_list[demand_list_index];
        joint_.setCommand(command_out);

      }else if(direction == BACKWARD){

        command_out = command_list[demand_list_index] * -1;
        joint_.setCommand(command_out);

      }
      update_counter++;

    ///Publish the current state
      if(controller_state_publisher_ && controller_state_publisher_->trylock())
      {
        controller_state_publisher_->msg_.header.stamp = ros::Time::now();

        controller_state_publisher_->msg_.vector.x = command_out;
        controller_state_publisher_->msg_.vector.y = (ADC_data - offset)/lever_factor;
        controller_state_publisher_->msg_.vector.z = stable;
        controller_state_publisher_->unlockAndPublish();
      } 

    }



    ///run each step for x seconds.
    if(update_counter >= (unsigned int) (min_acquisiton_length * update_rate) ) {
      stable = TRUE;
      update_counter = 0;

    }



  }//end of function


  /** starting() is called when the controller starts, it resets the variables to be identified
  *
  * \author Raphael Nagel
  * \date 18/Aug/2014
  **************************************** */
  void thruster_ident_driver::starting(const ros::Time& time) { 

    ROS_INFO("ros_control - thruster_ident_driver: starting the controller. \n");

    calibration_complete = FALSE;
    finished = FALSE;
    stable = FALSE;
    demand_list_index = 0;
    update_counter = 0;
    ADC_data = 0;
    ROS_INFO("ros_control - thruster_ident_driver: Calibrating...\n");

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

PLUGINLIB_EXPORT_CLASS( ros_control_iso::thruster_ident_driver, controller_interface::ControllerBase)
