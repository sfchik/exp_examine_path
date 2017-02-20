#include <exp_examine_path/calc_metric_path.h>

void CalcMetricPath::initialize()
{
	//variable need to be initialized eg. cumulative_heading_change or else will get NaN error
	counter_start_ = 0;
	counter_printer_ = 0;
	stop_number_ = 0;
	path_length_ = 0.0; 
	path_deviation_ = 0.0;
	yaw_degree_previous_ = 0.0;
	cumulative_heading_change_ = 0.0;
	path_deviation_ = 0.0;
	zero_crossing_robot_ = 0.0; //this is to minus off the last goal stopping
	vel_prev_ = 0.0;	
	res_vel_ = 0.0;
	del_vel_ = 0.0;	
	time_stop_ = 0.0;
	twist_sign_robot_ = true;

	nh_.param("/calc_metric_path/robot_init_x",	robot_init_x_, 2.0);
	nh_.param("/calc_metric_path/robot_init_y",	robot_init_y_, 4.0);
	nh_.param("/calc_metric_path/robot_init_yaw", robot_init_yaw_, 0.0);
	nh_.param("/calc_metric_path/robot_goal_x",	robot_goal_x_, 2.0);
	nh_.param("/calc_metric_path/robot_goal_y",	robot_goal_y_, 4.0);
	nh_.param("/calc_metric_path/robot_goal_yaw", robot_goal_yaw_, 0.0);	
	nh_.param("/calc_metric_path/start_x_extra", start_x_extra_, 2.0);  //TO DO, currently applied to filter x distance only
	start_x_extra_ += robot_init_x_;
	ROS_INFO("start_x_extra_ %f", start_x_extra_);	
	//initialize for pedestrian
	// pedestrian_1_.configureStartAndGoal(25.0, 3.5, 5.0, 3.5);

	flag_.flag = 0;

	sub_odom_ = nh_.subscribe("/robot_position/corrected", 1, &CalcMetricPath::callbackRobotOdometry, this);  //odom
	sub_localization_ = nh_.subscribe("/amcl_pose", 1, &CalcMetricPath::callbackLocalization, this);
	sub_goal_ = nh_.subscribe("/move_base/goal", 1, &CalcMetricPath::callbackGoal, this);
	sub_twist_ = nh_.subscribe("/pedbot/control/cmd_vel", 1, &CalcMetricPath::callbackTwist, this);
	sub_pedsim_ = nh_.subscribe("/pedsim/tracked_persons", 1, &CalcMetricPath::callbackPedsim, this);
}

void CalcMetricPath::loop()
{
	if(flag_.FIRST_ODOM == TRUE && flag_.FIRST_TWIST == TRUE && flag_.FIRST_PEDSIM == TRUE) {

		if(robot_odometry_current_.pose.pose.position.x < start_x_extra_) {
			//Do not update path length until certain position x
			calculatePathLength(robot_odometry_current_.pose.pose.position, robot_odometry_previous_.pose.pose.position);
			calculateHeadingChange(robot_odometry_current_, yaw_degree_previous_);
		} else {
			path_length_ += calculatePathLength(robot_odometry_current_.pose.pose.position, robot_odometry_previous_.pose.pose.position);
			cumulative_heading_change_ += calculateHeadingChange(robot_odometry_current_, yaw_degree_previous_);	

			if(std::hypot(robot_odometry_current_.pose.pose.position.x - robot_goal_x_, 
											robot_odometry_current_.pose.pose.position.y - robot_goal_y_) > 2.0) {
				// res_vel_ = std::hypot(robot_odometry_current_.twist.twist.linear.x, 
				// 								robot_odometry_current_.twist.twist.linear.y);
				res_vel_ = robot_odometry_current_.twist.twist.linear.x;
				del_vel_ = (res_vel_ - vel_prev_)*100.0;
				vel_prev_ = res_vel_;
				zero_crossing_robot_ = zero_crossing_robot_ + calculateSmoothness(del_vel_, twist_sign_robot_);	
				
				if(res_vel_ < 0.1 && !flag_.ROBOT_STOP) {
					time_store_ = ros::Time::now().toSec(); 
					flag_.ROBOT_STOP = true;
				} else if (res_vel_ > 0.1 && flag_.ROBOT_STOP) {
					double time_diff = ros::Time::now().toSec() - time_store_;
					time_stop_ = (time_diff > time_stop_)?time_diff:time_stop_;
					++stop_number_;
					flag_.ROBOT_STOP = false;
				}
			}					
		}
		calculatePathDeviation(robot_goal_y_, robot_odometry_current_);

		// pedestrian_1_.calculatePedsimMetric(pedsim_tracked_.tracks[0].pose.pose, pedsim_tracked_.tracks[0].twist.twist);
		if(counter_printer_ == 100) {
		
				//pedestrian_1_.printPedsimMetric();
			//pl=%f CHC=%f dev=%f zc=%d
			ROS_INFO("%f,%f,%f,%f,%f,%d", path_length_, cumulative_heading_change_,
						path_deviation_, zero_crossing_robot_, time_stop_, stop_number_);//for path examine
//			ROS_INFO("x=%f y=%f", robot_localiztion_.pose.pose.position.x, robot_localiztion_.pose.pose.position.y);
			counter_printer_ = 0;
		} else {
			++counter_printer_;
		}
	}
}

void CalcMetricPath::callbackRobotOdometry(const nav_msgs::Odometry::ConstPtr& msg)
{
	flag_.FIRST_ODOM = TRUE;
	robot_odometry_current_ = *msg;
}

void CalcMetricPath::callbackLocalization(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
	robot_localization_ = *msg;
}

void CalcMetricPath::callbackGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr& msg)
{
	flag_.FIRST_GOAL = TRUE;
	robot_goal_ = *msg;
}

void CalcMetricPath::callbackTwist(const geometry_msgs::Twist::ConstPtr& msg)
{
	robot_twist_ = *msg;
	if(robot_twist_.linear.x > 0.001) {
		flag_.FIRST_TWIST = TRUE;
	}
}

void CalcMetricPath::callbackPedsim(const pedsim_msgs::TrackedPersons::ConstPtr& msg)
{
	flag_.FIRST_PEDSIM = TRUE;
	pedsim_tracked_ = *msg;
}

//void CalcMetricPath::calculatePedMetric(const ped

float CalcMetricPath::calculatePathLength(const geometry_msgs::Point& current, geometry_msgs::Point& previous)
{
	float x_difference = current.x - previous.x;
	float y_difference = current.y - previous.y;
	float path_length = sqrt(x_difference*x_difference + y_difference*y_difference);
	previous = current;
	return path_length;
}

float CalcMetricPath::calculateHeadingChange(const nav_msgs::Odometry& current, float& yaw_degree_previous)
{
	//the counter is just use to filter initial NaN data
	if(counter_start_ > 2) {
		float yaw_angle_change = fabs(current.pose.pose.position.z - yaw_degree_previous);

		yaw_angle_change = (yaw_angle_change > 2.0)?0.0:yaw_angle_change;  //to filter suddenly drop to 0.0 bug
		yaw_degree_previous = current.pose.pose.position.z;
	
		return yaw_angle_change;
	} else {
		++counter_start_;
		return 0.0; 		
	}
}

void CalcMetricPath::calculatePathDeviation(double goal_y, const nav_msgs::Odometry& odom)
{
	float deviation = 0.0;
	deviation = fabs(odom.pose.pose.position.y - goal_y);
	path_deviation_ = (deviation>path_deviation_)?deviation:path_deviation_;
}

double CalcMetricPath::calculateSmoothness(double del_vel, bool& positive)
{
	if(positive && del_vel < -0.5 && del_vel > -1.0) {
		positive = false;
		return 1.0;
	} else if(!positive && del_vel > 0.5 && del_vel < 1.0) {
		positive = true;
		return 1.0;
	} else {
		return 0.0;
	}
}

int main(int argc, char **argv)
{
  	//Initiate ROS
	ros::init(argc, argv, "calculate_metric");

	ros::NodeHandle n;
	//Create an object of class CalcMetricPath that will take care of everything
	CalcMetricPath cmp;
	ros::Rate r(HZ);

	while(ros::ok()) {
		ros::spinOnce();
		cmp.loop();		
		r.sleep();
	}

	return 0;
}
