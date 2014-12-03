/**********************************************************************
*   This controller header file belongs to
*	 relay with hysterisis. It is used to identify
*   system parameters by following the identification by self oscillation
*   described by Miskovic DOI: 10.1002/rob.20374
*
*   This controller is based on: 
*   https://github.com/ros-controls/ros_control/wiki/controller_interface
*   and
*   https://github.com/labust/labust-ros-pkg/tree/master/ident_so
*
*   further modified by Raphael Nagel
*   raphael.nagel (#) posteo de
*   18/Aug/2014
*///////////////////////////////////////////////////////////////////////
#ifndef __RELAY_W_HYSTERESIS__
#define __RELAY_W_HYSTERESIS__


#include <ros/node_handle.h>
//#include <urdf/model.h>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>



#include <realtime_tools/realtime_publisher.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <control_msgs/JointControllerState.h>
#include <control_msgs/JointControllerState.h>
#include <realtime_tools/realtime_buffer.h>

#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Vector3Stamped.h"


#define TRUE 1
#define FALSE 0
#define FORWARD true
#define BACKWARD false

#define STRICT 2



namespace ros_control_iso{

	class thruster_ident_driver : public controller_interface::Controller<hardware_interface::EffortJointInterface>
	{
	public:
	 	bool init(hardware_interface::EffortJointInterface* , ros::NodeHandle& );
		void update(const ros::Time& , const ros::Duration& );
		void starting(const ros::Time& );
		void stopping(const ros::Time& ); 
		void subs_callback(const geometry_msgs::Vector3::ConstPtr& );

	private:

		std::vector<double> command_list;

		ros::Subscriber subscriber;

		unsigned int ADC_data;	

		bool finished;
		double min_acquisiton_length;


		std::string my_joint;
		double lever_factor;
		double update_rate;
		double calibration_length;
		double offset;
		bool direction;
		bool stable;
		bool calibration_complete;
		unsigned int demand_list_index;
		unsigned int update_counter;
		unsigned int calibration_length_num_of_samples;


		


		hardware_interface::JointHandle joint_;
 		//realtime_tools::RealtimeBuffer<Commands> command_;
 		//Commands command_struct_; // pre-allocated memory that is re-used to set the realtime buffer 		
		boost::scoped_ptr <realtime_tools::RealtimePublisher <geometry_msgs::Vector3Stamped> > controller_state_publisher_ ;

	};
};

#endif