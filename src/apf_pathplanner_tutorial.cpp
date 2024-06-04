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
#include <apf_pathplanner_tutorial/APFPathPlannerTutorialConfig.h>

geometry_msgs::PoseWithCovarianceStamped g_robot;
geometry_msgs::PoseStamped g_goal;
std::vector<geometry_msgs::PointStamped> g_obstacles;

double g_potential_field_rows					= 240;
double g_potential_field_cols					= 240;
double g_potential_field_resolution				= 0.05;
double g_weight_attraction_field				= 0.1;
double g_weight_repulsion_field					= 0.1;
double g_distance_threshold_repulsion_field		= 0.3;
double g_max_path_length						= 6.0;
size_t g_path_search_range						= 1;
std::string g_potential_field_filter_mode		= "and";
std::vector<size_t> g_potential_field_filter_terms;

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

void param_callback(const apf_pathplanner_tutorial::APFPathPlannerTutorialConfig& param, uint32_t level)
{
	g_potential_field_rows					= param.potential_field_rows;
	g_potential_field_cols					= param.potential_field_cols;
	g_potential_field_resolution			= param.potential_field_resolution;
	g_weight_attraction_field				= param.weight_attraction_field;
	g_weight_repulsion_field				= param.weight_repulsion_field;
	g_distance_threshold_repulsion_field	= param.distance_threshold_repulsion_field;
	g_max_path_length						= param.max_path_length;
	g_path_search_range						= param.path_search_range;

	g_potential_field_filter_mode			= param.potential_field_grid_filter;
	g_potential_field_filter_terms.clear();
	if (param.goal)							g_potential_field_filter_terms.push_back(potbot_lib::Potential::GridInfo::IS_GOAL);
	if (param.robot)						g_potential_field_filter_terms.push_back(potbot_lib::Potential::GridInfo::IS_ROBOT);
	if (param.obstacle)						g_potential_field_filter_terms.push_back(potbot_lib::Potential::GridInfo::IS_OBSTACLE);
	if (param.repulsion_inside)				g_potential_field_filter_terms.push_back(potbot_lib::Potential::GridInfo::IS_REPULSION_FIELD_INSIDE);
	if (param.repulsion_edge)				g_potential_field_filter_terms.push_back(potbot_lib::Potential::GridInfo::IS_REPULSION_FIELD_EDGE);
	if (param.path)							g_potential_field_filter_terms.push_back(potbot_lib::Potential::GridInfo::IS_PLANNED_PATH);
	if (param.around_goal)					g_potential_field_filter_terms.push_back(potbot_lib::Potential::GridInfo::IS_AROUND_GOAL);
	if (param.local_minimum)				g_potential_field_filter_terms.push_back(potbot_lib::Potential::GridInfo::IS_LOCAL_MINIMUM);
}

int main(int argc,char **argv){
	ros::init(argc,argv,"apf_pathplanner_tutorial");

	ros::NodeHandle nh;

	ros::Publisher pub_attraction_field		= nh.advertise<sensor_msgs::PointCloud2>("field/attraction", 1);
	ros::Publisher pub_repulsion_field		= nh.advertise<sensor_msgs::PointCloud2>("field/repulsion", 1);
	ros::Publisher pub_potential_field		= nh.advertise<sensor_msgs::PointCloud2>("field/potential", 1);
	ros::Publisher pub_filtered_field		= nh.advertise<sensor_msgs::PointCloud2>("field/filtered", 1);
	ros::Publisher pub_path_raw				= nh.advertise<nav_msgs::Path>("path/raw", 1);
	ros::Publisher pub_path_interpolated	= nh.advertise<nav_msgs::Path>("path/interpolated", 1);

	ros::Subscriber sub_inipose				= nh.subscribe("initialpose",1,inipose_callback);
	ros::Subscriber sub_goal				= nh.subscribe("move_base_simple/goal",1,goal_callback);
	ros::Subscriber sub_point				= nh.subscribe("clicked_point",1,point_callback);

	dynamic_reconfigure::Server<apf_pathplanner_tutorial::APFPathPlannerTutorialConfig> server;
	dynamic_reconfigure::Server<apf_pathplanner_tutorial::APFPathPlannerTutorialConfig>::CallbackType f;
	f = boost::bind(param_callback, _1, _2);
	server.setCallback(f);

	while (ros::ok())
	{
		potbot_lib::utility::Timer timer;
        timer.start("1 loop");

        potbot_lib::PathPlanner::APFPathPlanner apf(
							g_potential_field_rows,					//ポテンシャル場の幅(x軸方向) [m]
							g_potential_field_cols,					//ポテンシャル場の高さ(y軸方向) [m]
							g_potential_field_resolution,			//ポテンシャル場グリッド1辺の長さ [m]
							g_weight_attraction_field,				//ポテンシャル場における引力場の重み
							g_weight_repulsion_field,				//ポテンシャル場における斥力場の重み
							g_distance_threshold_repulsion_field	//斥力場を場を作る距離の閾値 [m]
							);
        apf.set_goal(	g_goal.pose.position.x, 		g_goal.pose.position.y);
        apf.set_robot(	g_robot.pose.pose.position.x, 	g_robot.pose.pose.position.y);
		
		for (auto obs : g_obstacles)
		{
			apf.set_obstacle(obs.point.x,				obs.point.y);
		}

		timer.start("potential");
        apf.create_potential_field();
		timer.stop("potential");

		std::vector<std::vector<double>> path_raw, path_interpolated;
		double init_yaw = potbot_lib::utility::get_Yaw(g_robot.pose.pose.orientation);
		if (isnan(init_yaw)) init_yaw = 0;

		timer.start("path");
		apf.create_path(path_raw, init_yaw, g_max_path_length, g_path_search_range);
		apf.bezier(path_raw, path_interpolated);
		timer.stop("path");

		timer.print_time();

		potbot_lib::Potential::Field attraction_field, repulsion_field, potential_field, filtered_field;
		apf.get_attraction_field(attraction_field);
		apf.get_repulsion_field(repulsion_field);
		apf.get_potential_field(potential_field);
		// potential_field.info_filter(filtered_field, {potbot_lib::Potential::GridInfo::IS_PLANNED_PATH, potbot_lib::Potential::GridInfo::IS_REPULSION_FIELD_EDGE},"and");
		potential_field.info_filter(filtered_field, g_potential_field_filter_terms, g_potential_field_filter_mode);

		nav_msgs::Path path_msg_raw, path_msg_interpolated;
		for (auto point : path_raw)
		{
			geometry_msgs::PoseStamped pose_msg;
			pose_msg.pose.position.x = point[0];
			pose_msg.pose.position.y = point[1];
			path_msg_raw.poses.push_back(pose_msg);
		}
		for (auto point : path_interpolated)
		{
			geometry_msgs::PoseStamped pose_msg;
			pose_msg.pose.position.x = point[0];
			pose_msg.pose.position.y = point[1];
			path_msg_interpolated.poses.push_back(pose_msg);
		}

        sensor_msgs::PointCloud2 attraction_field_msg, repulsion_field_msg, potential_field_msg, filtered_field_msg;
        attraction_field.to_pcl2(attraction_field_msg);
        repulsion_field.to_pcl2(repulsion_field_msg);
        potential_field.to_pcl2(potential_field_msg);
		filtered_field.to_pcl2(filtered_field_msg);

        std_msgs::Header header_apf;
        header_apf.frame_id				= "map";
        header_apf.stamp				= ros::Time::now();
        attraction_field_msg.header		= header_apf;
        repulsion_field_msg.header		= header_apf;
        potential_field_msg.header		= header_apf;
		filtered_field_msg.header		= header_apf;
		path_msg_raw.header				= header_apf;
		path_msg_interpolated.header	= header_apf;

        pub_attraction_field.publish(attraction_field_msg);
        pub_repulsion_field.publish(repulsion_field_msg);
        pub_potential_field.publish(potential_field_msg);
		pub_filtered_field.publish(filtered_field_msg);
		pub_path_raw.publish(path_msg_raw);
		pub_path_interpolated.publish(path_msg_interpolated);

		ros::spinOnce();

		timer.stop("1 loop");
		timer.print_time("1 loop");
		
	}

	return 0;
}