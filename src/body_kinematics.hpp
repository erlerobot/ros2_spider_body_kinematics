
#ifndef BODY_KINEMATICS_HPP_
#define BODY_KINEMATICS_HPP_

//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp> 
#include <kdl_parser/kdl_parser.hpp>

//see http://design.ros2.org/articles/generated_interfaces_cpp.html
//#include <spider_msgs/GetLegIKSolver.h>
#include <spider_msgs/srv/get_leg_ik_solver.hpp>  
//#include <spider_msgs/LegsJointsState.h>
#include <spider_msgs/msg/legs_joints_state.hpp>
//#include <spider_msgs/BodyState.h>
#include <spider_msgs/msg/body_state.hpp>
//#include <spider_msgs/BodyCommand.h>
#include <spider_msgs/msg/body_command.hpp>


#define NUM_LEGS 6
#define NUM_JOINTS 3

class BodyKinematics {
	public:
		BodyKinematics();
		bool init();

	private:
		//CREATE NODE
		//ros::NodeHandle node;
		rclcpp::Node::SharedPtr node;
		std::string root_name, tip_name;
		//std::vector<KDL::Frame> frames;
		//spider_msgs::BodyState bs;
		spider_msgs::msg::BodyState bs;           //the folders have changed
		//spider_msgs::LegsJointsState legs;
		spider_msgs::msg::LegsJointsState legs;
		//spider_msgs::GetLegIKSolver srv;
		spider_msgs::srv::GetLegIKSolver srv;
		const static unsigned int num_joints = NUM_JOINTS;
		const static unsigned int num_legs = NUM_LEGS;
		double z;

		/*KDL::Rotation rotation;
		KDL::Frame tibia_foot_frame, femur_frame;
		KDL::Vector offset_vector, rotate_correction, final_vector [num_legs];*/

		//ros::ServiceClient client;
		rclcpp::client::ClientBase::SharedPtr client;
		//ros::Publisher joints_pub;
		rclcpp::publisher::Publisher::SharedPtr joints_pub;
		//ros::Subscriber body_move_sub;
		rclcpp::subscription::SubscriptionBase::SharedPtr body_move_sub;
		//ros::Subscriber body_cmd_sub;
		rclcpp::subscription::SubscriptionBase::SharedPtr body_cmd_sub;

		bool loadModel(const std::string xml);
		//bool calculateKinematics (spider_msgs::BodyState* body_ptr);
		bool calculateKinematics (spider_msgs::msg::BodyState* body_ptr);
		//bool callService (KDL::Vector* vector);
		//void teleopBodyMove (const spider_msgs::BodyStateConstPtr &body_state);
		void teleopBodyMove (const spider_msgs::msg::BodyState::SharedPtr &body_state);
		//void teleopBodyCmd (const spider_msgs::BodyCommandConstPtr &body_cmd);
		void teleopBodyCmd (const spider_msgs::msg::BodyCommand::SharedPtr &body_cmd);   //Puting CnstPtr warnings occur
};



#endif /* BODY_KINEMATICS_HPP_ */
