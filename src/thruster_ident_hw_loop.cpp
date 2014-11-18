/**********************************************************************
*   Copyright 2014 GNU License
*   This node is the hardware loop to be used for identifying the thrusters
*   of the BMT submarine.
*
*
*   Author Raphael Nagel
*   first.lastname (#) posteo de
*   14/Oct/2014
*
*   To do:
*       watchdog timer
*///////////////////////////////////////////////////////////////////////


#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>


#include "ros/ros.h"
#include <sstream>
#include <stdio.h>
#include <signal.h>

#include "thruster_ident_driver/thruster_ident_hw_loop.hpp"


namespace UWEsub {

    phoenix_hw_interface::phoenix_hw_interface() {
        ///set the controller output to 0
        cmd[0] = 0.0;
       
        // and the controller input / current position
        pos[0] = 0.0;

        vel[0] = 0.0;

        eff[0] = 0.0;        

        sequence = 0;
        terminate_flag = false;
        safe = false;


        /// Create a panic command line panic button that can be used like this: rosservice call stop
        panic_stopper = nh_.advertiseService("/stop", &phoenix_hw_interface::panic, this);
        if(panic_stopper) {
            ROS_INFO("thruster_ident_hw_loop: Panic button online");
        } else {
            ROS_ERROR("thruster_ident_hw_loop: Panic Button not functioning");
        }        

        /// connect and register the joint state interface for the 6 DOF
        hardware_interface::JointStateHandle state_handle_x("thruster", &pos[0], &vel[0], &eff[0]);
        joint_state_interface.registerHandle(state_handle_x);

        /// register the joint_state_interface
        registerInterface(&joint_state_interface);


        /// connect and register the joint effort interface for the 6 potentially controlled DOF
        hardware_interface::JointHandle pos_handle_x(joint_state_interface.getHandle("x"), &cmd[0]);
        jnt_eff_interface.registerHandle(pos_handle_x);
         

        /// register the joint effort interface
        registerInterface(&jnt_eff_interface);


        /// Subscribe to the Feedback Signal
        feedback_fused = nh_.subscribe<geometry_msgs::Vector3>("thruster_ident_adc", 1, &phoenix_hw_interface::sub_callback, this);

        ///Initialise the controller manager
        ROS_INFO("thruster_ident_hw_loop: Loading the controller manager");
        controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));

        /// Set up the real time safe publisher
        thruster_driver_command_publisher_.reset(new realtime_tools::RealtimePublisher<std_msgs::Float32MultiArray>(nh_, "motorVal", 1) );

        /// Get the required variables from the parameter server and set standard values if not available
        loop_hz_ = 0;
        if (!nh_.getParam("/thruster_interface/update_rate", loop_hz_)) {

            ROS_ERROR("thruster_ident_hw_loop: Could not find update rate, assuming 50. \n");
            loop_hz_ = 50;
            nh_.setParam("thruster_interface/update_rate", loop_hz_);
        }

        /// Set up the control loop by creating a timer and a connected callback
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        timer_update = nh_.createTimer(update_freq, &phoenix_hw_interface::update, this);
    }

    /** Destructor: 
    */
    phoenix_hw_interface::~phoenix_hw_interface() {}


    /**terminate(): Will try to stop the thrusters before shutting down
                    Should send at least 3 messages with 0 commands before we shutdown
    */
    void phoenix_hw_interface::terminate(void) {
        timer_update.stop();
        for (int x = 0; x < write_command.size(); x++) {
            write_command[x] = 0;
        }
        
        write();

        while ( write() != EXIT_SUCCESS ) {
            usleep(100);
            for (int x = 0; x < write_command.size(); x++) {
                write_command[x] = 0;
            }
            ROS_ERROR("thruster_ident_hw_loop: still trying to stop the thrusters, then shutting down");
        } 
        ROS_ERROR("thruster_ident_hw_loop: shutting down, set the thrusters to 0"); 
        safe = true;
    }

    /** panic(): the service call that will stop all thrusters
            usage: rosservice call stop
    */
    bool phoenix_hw_interface::panic( std_srvs::Empty::Request& request,  std_srvs::Empty::Response& response) {
        /// Stopping the ros::timer
        timer_update.stop();
        for (int x = 0; x < write_command.size(); x++) {
            write_command[x] = 0;
        }

        while ( write() != EXIT_SUCCESS ) {
            usleep(100);
            for (int x = 0; x < write_command.size(); x++) {
                write_command[x] = 0;
            }
            ROS_ERROR("thruster_ident_hw_loop: still trying to stop the thrusters");
        }

        ROS_ERROR("thruster_ident_hw_loop: You have panic stopped the controller. All thrusters have stopped and the hardware loop needs restarting");
        // Just to be safe change to the panic_loop which just sends 0s out.
        ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
        timer_update = nh_.createTimer(update_freq, &phoenix_hw_interface::panic_loop, this);

    }

    /** panic_loop(): once a panic stop has been registered this loop will replace update()
            It will continously send a value of 0 to the thrusters.
    */
    void phoenix_hw_interface::panic_loop(const ros::TimerEvent& event) {

        for (int x = 0; x < write_command.size(); x++) {
            write_command[x] = 0;
        }
        write();
    }


    /** sub_callback_...() listens to the ADC signal from the loadcell
    The signal is stored unaltered.
    */
    void phoenix_hw_interface::sub_callback(const geometry_msgs::Vector3::ConstPtr& message) {
        pos[0] = message->x;
    }


    /** Update():  the real action happens here: run the controllers and update the actuator output
    */
    void phoenix_hw_interface::update(const ros::TimerEvent& event) {
        if(terminate_flag == true){
            terminate();
        } else {
            /// Update the time since the last update
            ros::Duration elapsed_time_ = ros::Duration(event.current_real - event.last_real);

            /// Let the controller do its work
            controller_manager_->update(ros::Time::now(), elapsed_time_);


            // Write the new command to the motor drivers
            write();
        }
    }


    /** Write(): writes the command to the actual hardware driver
    *       This is done by sending a message of type std_msgs::Float32MultiArray
    *       to the ros topic motorVal.
    *       When running on the BMT BRD then make sure there aren't more than 6 commands
    */
    int phoenix_hw_interface::write(void) {
        // send the message in a realtime safe fashion
       if ( thruster_driver_command_publisher_ && thruster_driver_command_publisher_->trylock() ) {
            // resize the array
            thruster_driver_command_publisher_->msg_.data.clear();
            thruster_driver_command_publisher_->msg_.data.resize(write_command.size(),0);
            // fill the message with the commands to be written
            for (int x = 0; x < write_command.size(); x++) {
                thruster_driver_command_publisher_->msg_.data[x] = write_command[x];
            }
            // send it off
            thruster_driver_command_publisher_->unlockAndPublish();
            return EXIT_SUCCESS;
       } else {
            ROS_ERROR("thruster_ident_hw_loop: could not send message to the thruster driver");
            return EXIT_FAILURE;
       }
    }


}  // namespace


bool UWEsub::phoenix_hw_interface::terminate_flag = 0;
bool UWEsub::phoenix_hw_interface::safe = 0;

/** set_terminate_flag(): sets the static terminate_flag inside the phoenix_hw_interface.
                            The set flag causes the object to initiate a safe shutdown which sends
                            0 commands to the thrusters.
*/
void set_terminate_flag(int sig) {
    UWEsub::phoenix_hw_interface::terminate_flag = true;
    while(UWEsub::phoenix_hw_interface::safe == false){
        usleep(100);
    }
    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "thruster_ident_hw_loop");
    ros::NodeHandle nh;
    /// An Async spinner creates another thread which will handle the event of this node being executed.
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // create the instance of the class
    UWEsub::phoenix_hw_interface hw_loop;

    // register the 
    signal(SIGINT, set_terminate_flag);
    ros::spin();


    ROS_INFO("thruster_ident_hw_loop: Shutting down hardware interface");
}
