#include <ros/ros.h>
#include <potbot_lib/PotentialField.h>
#include <potbot_lib/PathPlanner.h>
#include <potbot_lib/Utility.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
#include <apf_pathplanner_tutorial/apf_pathplanner_tutorialConfig.h>

geometry_msgs::PoseWithCovarianceStamped g_robot;
geometry_msgs::PoseStamped g_goal;
std::vector<geometry_msgs::PointStamped> g_obstacles;

double g_potential_field_width					= 12;
double g_potential_field_height					= 12;
double g_potential_field_resolution				= 0.05;
double g_weight_attraction_field				= 0.1;
double g_weight_repulsion_field					= 0.1;
double g_distance_threshold_repulsion_field		= 0.3;
double g_max_path_length						= 6.0;
size_t g_path_search_range						= 1;

void inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    g_robot = msg;
}

void goal_callback(const geometry_msgs::PoseStamped& msg)
{
    g_goal = msg;
}

void point_callback(const geometry_msgs::PointStamped& msg)
{
    g_obstacles.push_back(msg);
}

void param_callback(const apf_pathplanner_tutorial::apf_pathplanner_tutorialConfig& param, uint32_t level)
{
	g_potential_field_width					= param.potential_field_width;
	g_potential_field_height				= param.potential_field_height;
	g_potential_field_resolution			= param.potential_field_resolution;
	g_weight_attraction_field				= param.weight_attraction_field;
	g_weight_repulsion_field				= param.weight_repulsion_field;
	g_distance_threshold_repulsion_field	= param.distance_threshold_repulsion_field;
	g_max_path_length						= param.max_path_length;
	g_path_search_range						= param.path_search_range;
}

int main(int argc,char **argv){
	ros::init(argc,argv,"apf_pathplanner_tutorial");

	ros::NodeHandle nh;

	ros::Publisher pub_attraction_field	= nh.advertise<sensor_msgs::PointCloud2>("field/attraction", 1);
	ros::Publisher pub_repulsion_field	= nh.advertise<sensor_msgs::PointCloud2>("field/repulsion", 1);
	ros::Publisher pub_potential_field	= nh.advertise<sensor_msgs::PointCloud2>("field/potential", 1);
	ros::Publisher pub_filtered_field	= nh.advertise<sensor_msgs::PointCloud2>("field/filtered", 1);
	ros::Publisher pub_path				= nh.advertise<nav_msgs::Path>("path", 1);

	ros::Subscriber sub_inipose			= nh.subscribe("initialpose",1,inipose_callback);
	ros::Subscriber sub_goal			= nh.subscribe("move_base_simple/goal",1,goal_callback);
	ros::Subscriber sub_point			= nh.subscribe("clicked_point",1,point_callback);

	dynamic_reconfigure::Server<apf_pathplanner_tutorial::apf_pathplanner_tutorialConfig> server;
	dynamic_reconfigure::Server<apf_pathplanner_tutorial::apf_pathplanner_tutorialConfig>::CallbackType f;
	f = boost::bind(param_callback, _1, _2);
	server.setCallback(f);

	while (ros::ok())
	{
        
        potbot_lib::PathPlanner::APFPathPlanner apf(
							g_potential_field_width,				//ポテンシャル場の幅(x軸方向) [m]
							g_potential_field_height,				//ポテンシャル場の高さ(y軸方向) [m]
							g_potential_field_resolution,			//ポテンシャル場グリッド1辺の長さ [m]
							g_weight_attraction_field,				//ポテンシャル場における引力場の重み
							g_weight_repulsion_field,				//ポテンシャル場における斥力場の重み
							g_distance_threshold_repulsion_field	//斥力場を場を作る距離の閾値 [m]
							);

        apf.set_goal(	g_goal.pose.position.x, 		g_goal.pose.position.y);
        apf.set_robot(	g_robot.pose.pose.position.x, 	g_robot.pose.pose.position.y);
		// for (auto obs : g_obstacles)
		// {
		// 	apf.set_obstacle(obs.point.x,				obs.point.y);
		// }
		
		for (double inc = -3; inc <=3; inc += 0.05)
		{
			double x = inc;
			double y = 3;
			apf.set_obstacle(x,y);
		}

		double time_start = ros::Time::now().toSec();
        apf.create_attraction_field();
		double time_create_attraction = ros::Time::now().toSec() - time_start;

		time_start = ros::Time::now().toSec();
        apf.create_repulsion_field();
		double time_create_repulsion = ros::Time::now().toSec() - time_start;

		time_start = ros::Time::now().toSec();
        apf.create_potential_field();
		double time_create_potential = ros::Time::now().toSec() - time_start;
		
		

		std::vector<std::vector<double>> path;
		double init_yaw = potbot_lib::utility::get_Yaw(g_robot.pose.pose.orientation);
		if (isnan(init_yaw)) init_yaw = 0;

		time_start = ros::Time::now().toSec();
		apf.create_path(path, init_yaw, g_max_path_length, g_path_search_range);
		double time_create_path = ros::Time::now().toSec() - time_start;

		ROS_INFO("attraction:%f  repulsion:%f  potential:%f  path:%f total:%f", 
		time_create_attraction, time_create_repulsion, time_create_potential, time_create_path,
		time_create_attraction+ time_create_repulsion+ time_create_potential+ time_create_path);

		potbot_lib::Potential::Field attraction_field, repulsion_field, potential_field, filtered_field;
		apf.get_attraction_field(attraction_field);
		apf.get_repulsion_field(repulsion_field);
		apf.get_potential_field(potential_field);
		potential_field.info_filter(filtered_field, {potbot_lib::Potential::GridInfo::IS_PLANNED_PATH, potbot_lib::Potential::GridInfo::IS_REPULSION_FIELD_EDGE},"and");

		nav_msgs::Path path_msg;
		for (auto point : path)
		{
			geometry_msgs::PoseStamped pose_msg;
			pose_msg.pose.position.x = point[0];
			pose_msg.pose.position.y = point[1];
			path_msg.poses.push_back(pose_msg);
		}

        sensor_msgs::PointCloud2 attraction_field_msg, repulsion_field_msg, potential_field_msg, filtered_field_msg;
		time_start = ros::Time::now().toSec();
        attraction_field.to_pcl2(attraction_field_msg);
        repulsion_field.to_pcl2(repulsion_field_msg);
        potential_field.to_pcl2(potential_field_msg);
		filtered_field.to_pcl2(filtered_field_msg);
		double time_to_pcl = ros::Time::now().toSec() - time_start;

        std_msgs::Header header_apf;
        header_apf.frame_id = "map";
        header_apf.stamp = ros::Time::now();
        attraction_field_msg.header = header_apf;
        repulsion_field_msg.header = header_apf;
        potential_field_msg.header = header_apf;
		filtered_field_msg.header = header_apf;
		path_msg.header = header_apf;

		time_start = ros::Time::now().toSec();
        pub_attraction_field.publish(attraction_field_msg);
        pub_repulsion_field.publish(repulsion_field_msg);
        pub_potential_field.publish(potential_field_msg);
		pub_filtered_field.publish(filtered_field_msg);
		pub_path.publish(path_msg);
		double time_publish = ros::Time::now().toSec() - time_start;

		time_start = ros::Time::now().toSec();
		ros::spinOnce();
		double time_spinOnece = ros::Time::now().toSec() - time_start;

		ROS_INFO("to_pcl:%f  publish:%f  spinOnece:%f  total:%f", 
		time_to_pcl, time_publish, time_spinOnece, time_to_pcl + time_publish + time_spinOnece);

		ROS_INFO("1 loop:%f", time_create_attraction+ time_create_repulsion+ time_create_potential+ time_create_path+time_to_pcl + time_publish + time_spinOnece);
	}

	return 0;
}