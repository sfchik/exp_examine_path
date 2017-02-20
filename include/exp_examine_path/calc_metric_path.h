#ifndef CAL_METRIC_PATH_H
#define CAL_METRIC_PATH_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/Twist.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <cstdint>
#include <tf/transform_datatypes.h>
#include <math.h>

#define TRUE 1
#define FALSE 0
#define HZ 100
#define PI 3.142f

class PedestrianMetric
{
private:
	float culmulative_heading_change_, yaw_angle_change_prev_, yaw_angle_absolute_;
	float path_length_;
	float goal_x_;
	float goal_y_;
	float start_x_;
	float start_y_;
	float path_deviation_;
	float time_;
	float twist_x_prev_;
	float twist_y_prev_;

	int zero_cross_x_;
	int zero_cross_y_;

	geometry_msgs::Pose pedestrian_odom_prev_;

	bool REACHED_;
	bool INIT_PATH_LENGHT_;
	bool INIT_YAW_;
	bool POSITIVE_X_;
	bool POSITIVE_Y_;

	//dummy counter
	int filter_angle_;

public:
	PedestrianMetric(){
		REACHED_ = false;
		INIT_PATH_LENGHT_ = false;
		INIT_YAW_ = false;
		POSITIVE_X_ = true;
		POSITIVE_Y_ = true;
		culmulative_heading_change_ = 0.0;
		yaw_angle_change_prev_ = 0.0;
		yaw_angle_absolute_ = 0.0;
		path_length_ = 0.0;
		goal_x_ = 0.0;
		goal_y_ = 0.0;
		start_x_ = 0.0;
		start_y_ = 0.0;
		path_deviation_ = 0.0;
		time_ = 0.0;
		twist_x_prev_ = 0.0;
		twist_y_prev_ = 0.0;

		zero_cross_x_ = 0;
		zero_cross_y_ = 0;
	
		filter_angle_ = 0;
	}

	void configureStartAndGoal(float start_x, float start_y, float goal_x, float goal_y) {
		goal_x_ = goal_x;
		goal_y_ = goal_y;
		start_x_ = start_x;
		start_y_ = start_y;	
	}

	void calculatePedsimMetric(const geometry_msgs::Pose& current_pose, const geometry_msgs::Twist& current_twist) {
		if(REACHED_ != true) {
			if(INIT_PATH_LENGHT_ != true) {
				pedestrian_odom_prev_.position.x = current_pose.position.x;
				pedestrian_odom_prev_.position.y = current_pose.position.y;
				INIT_PATH_LENGHT_ = true;
			}

			float x_difference = current_pose.position.x - pedestrian_odom_prev_.position.x;
			float y_difference = current_pose.position.y - pedestrian_odom_prev_.position.y;
			path_length_ += sqrt(x_difference*x_difference + y_difference*y_difference);
			pedestrian_odom_prev_ = current_pose;	

			double yaw_rad, dummy_pitch, dummy_roll;  //must be double, or else will have error
			tf::Stamped<tf::Pose> quat;
			geometry_msgs::PoseStamped msg_convert;

			if(filter_angle_ > 20) {
				msg_convert.pose = current_pose;
				poseStampedMsgToTF(msg_convert, quat);
				quat.getBasis().getEulerYPR(yaw_rad, dummy_pitch, dummy_roll);
				if(INIT_YAW_ != true) {
					yaw_angle_absolute_ = 180.0*fabs(yaw_rad)/PI;
					yaw_angle_change_prev_ = yaw_angle_absolute_;
					INIT_YAW_ = true;
				}
				yaw_angle_absolute_ = 180.0*fabs(yaw_rad)/PI;
				culmulative_heading_change_ += fabs(yaw_angle_absolute_ - yaw_angle_change_prev_);
				yaw_angle_change_prev_ = yaw_angle_absolute_;
			} else {
				++filter_angle_;
			}

			float difference = current_twist.linear.x - twist_x_prev_;
			if(!POSITIVE_X_ && difference > 0.05) {
				++zero_cross_x_;
				POSITIVE_X_ = true;
			} else if(POSITIVE_X_ && difference < 0.05) { 
				++zero_cross_x_;
				POSITIVE_X_ = false;
			}
			
			x_difference = fabs(goal_x_ - current_pose.position.x);
			if(x_difference < 0.2) {
//				REACHED_ = true;
			}
		}
	} 

	void printPedsimMetric()
	{
//		ROS_INFO("%f", yaw_angle_absolute_);
		ROS_INFO("%f %f %d", path_length_, culmulative_heading_change_, zero_cross_x_);
	}
};

class CalcMetricPath
{

private:
	ros::NodeHandle nh_;

	//subscribers
	ros::Subscriber sub_odom_;	
	ros::Subscriber sub_localization_;
	ros::Subscriber sub_goal_;
	ros::Subscriber sub_twist_;
	ros::Subscriber sub_pedsim_;

	//some dummy variables and counters
	int counter_start_;  ///< @brief just to filter off NaN data from the heading change
	int counter_printer_;
	double time_stop_;
	double time_store_;

	union
	{
		uint16_t flag;
		struct 
		{
			unsigned FIRST_ODOM 	: 1;
			unsigned FIRST_GOAL 	: 1;
			unsigned FIRST_TWIST	: 1;
			unsigned FIRST_PEDSIM	: 1;
			unsigned ROBOT_STOP		: 1;
			unsigned unknown		: 11;
		};
	}flag_;

public:
	/**
	* @brief Constructor of the CalcMetricPath
	*/
	CalcMetricPath() 
	{
		initialize();
	}

	/**
	* @brief Initialize the ros handle
	* @param name, Ros NodeHandle name
	*/
	void initialize();

	/**
	* @brief Main program loop
	* @return void
	*/
	void loop();

	/**
	* @brief callbackRobotPosition, Read the robot odometry
	* @return void
	*/
  	void callbackRobotOdometry(const nav_msgs::Odometry::ConstPtr& msg);

	/**
	* @brief callbackRobotPosition, Read the robot localization position
	* @return void
	*/
	void callbackLocalization(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

	/**
	* @brief callbackGoal, Read the robot goal position
	* @return void
	*/
	void callbackGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg);

	/**
	* @brief callbackTwist, Read the robot twist
	* @return void
	*/
	void callbackTwist(const geometry_msgs::Twist::ConstPtr& msg);

	/**
	* @brief callbackPedsim, Read the pedsim data
	* @return void
	*/
	void callbackPedsim(const pedsim_msgs::TrackedPersons::ConstPtr& msg);

	/**
	* @brief calculate path length
	* @return path length
	*/	
	float calculatePathLength(const geometry_msgs::Point& current, geometry_msgs::Point& previous);

	/**
	* @brief calculate cumulative heading change
	* @return path length
	*/	
	float calculateHeadingChange(const nav_msgs::Odometry& current, float& yaw_degree_previous);

	/**
	* @brief calculate the path deviation
	* @return void
	*/	
	void calculatePathDeviation(double goal_y, const nav_msgs::Odometry& odom);

	/**
	* @brief calculate the path smoothness through zero crossing
	* @return 1 or 0 for if there is zero crossing
	*/	
	double calculateSmoothness(double del_vel, bool& positive);
	bool initialized_; ///<  @brief check if the metric calculation is initialized
	bool twist_sign_robot_;  ///<  @zero crossing sign

	nav_msgs::Odometry robot_odometry_current_;  ///< @brief to store current robot odometry	
	nav_msgs::Odometry robot_odometry_previous_;  ///< @brief to store previous robot odometry
	geometry_msgs::PoseWithCovarianceStamped robot_localization_;
	move_base_msgs::MoveBaseActionGoal robot_goal_;
	geometry_msgs::Twist robot_twist_, robot_twist_prev_;
	pedsim_msgs::TrackedPersons pedsim_tracked_;

	double zero_crossing_robot_;

	float path_length_; 
	float yaw_degree_previous_;
	float cumulative_heading_change_, cumulative_heading_change_ped_;
	float path_deviation_;
	int stop_number_;
	double vel_prev_;

	double robot_init_x_;
	double robot_init_y_;
	double robot_init_yaw_;
	double start_x_extra_;
	double robot_goal_x_;
	double robot_goal_y_;
	double robot_goal_yaw_;
	double res_vel_;
	double del_vel_;		

	PedestrianMetric pedestrian_1_;
};



#endif
