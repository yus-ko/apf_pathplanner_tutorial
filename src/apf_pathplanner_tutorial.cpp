#include <ros/ros.h>
#include <potbot_lib/artificial_potential_field.h>
#include <potbot_lib/apf_path_planner.h>
#include <potbot_lib/utility_ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <dynamic_reconfigure/server.h>
#include <apf_pathplanner_tutorial/APFPathPlannerTutorialConfig.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

class APFPathPlanner
{
private:
	geometry_msgs::PoseWithCovarianceStamped robot_;
	geometry_msgs::PoseStamped goal_;
	std::vector<geometry_msgs::PointStamped> obstacles_;

	potbot_lib::path_planner::APFPathPlanner* pp_;
	potbot_lib::ArtificialPotentialField* apf_;

	double potential_field_rows_					= 240;
	double potential_field_cols_					= 240;
	double potential_field_resolution_				= 0.05;
	double weight_attraction_field_					= 0.1;
	double weight_repulsion_field_					= 0.1;
	double distance_threshold_repulsion_field_		= 0.3;
	double max_path_length_							= 6.0;
	double path_weight_potential_					= 0.0;
	double path_weight_pose_						= 1.0;
	size_t path_search_range_						= 1;
	std::string potential_field_filter_mode_		= "and";
	std::vector<size_t> potential_field_filter_terms_;

	size_t interactive_marker_num_ = 3;
	std::vector<visualization_msgs::Marker> interactive_markers_;

	dynamic_reconfigure::Server<apf_pathplanner_tutorial::APFPathPlannerTutorialConfig> *dsrv_;
	interactive_markers::InteractiveMarkerServer *imsrv_;
	interactive_markers::MenuHandler *menu_handler_;

	void inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
	void goal_callback(const geometry_msgs::PoseStamped& msg);
	void point_callback(const geometry_msgs::PointStamped& msg);
	void param_callback(const apf_pathplanner_tutorial::APFPathPlannerTutorialConfig& param, uint32_t level);

	void __marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

	void __init_interactive_markers();
	void __init_interactive_marker_server();

public:
	APFPathPlanner();
	~APFPathPlanner(){};
};

APFPathPlanner::APFPathPlanner()
{
	ros::NodeHandle nh;

	ros::Publisher pub_attraction_field		= nh.advertise<sensor_msgs::PointCloud2>("field/attraction", 1);
	ros::Publisher pub_repulsion_field		= nh.advertise<sensor_msgs::PointCloud2>("field/repulsion", 1);
	ros::Publisher pub_potential_field		= nh.advertise<sensor_msgs::PointCloud2>("field/potential", 1);
	ros::Publisher pub_filtered_field		= nh.advertise<sensor_msgs::PointCloud2>("field/filtered", 1);
	ros::Publisher pub_path_raw				= nh.advertise<nav_msgs::Path>("path/raw", 1);
	ros::Publisher pub_path_interpolated	= nh.advertise<nav_msgs::Path>("path/interpolated", 1);
	ros::Publisher pub_loop_edges			= nh.advertise<visualization_msgs::MarkerArray>("debug/loop_edges", 1);

	ros::Subscriber sub_inipose				= nh.subscribe("initialpose",1,&APFPathPlanner::inipose_callback,this);
	ros::Subscriber sub_goal				= nh.subscribe("move_base_simple/goal",1,&APFPathPlanner::goal_callback,this);
	ros::Subscriber sub_point				= nh.subscribe("clicked_point",1,&APFPathPlanner::point_callback,this);

	dsrv_ = new dynamic_reconfigure::Server<apf_pathplanner_tutorial::APFPathPlannerTutorialConfig>(ros::NodeHandle("~"));
	dynamic_reconfigure::Server<apf_pathplanner_tutorial::APFPathPlannerTutorialConfig>::CallbackType cb = boost::bind(&APFPathPlanner::param_callback, this, _1, _2);
	dsrv_->setCallback(cb);

	__init_interactive_marker_server();

	while (ros::ok())
	{
		potbot_lib::utility::Timer timer;
        timer.start("1 loop");

		
		apf_ = new potbot_lib::ArtificialPotentialField(
							potential_field_rows_,					//ポテンシャル場の幅(x軸方向) [m]
							potential_field_cols_,					//ポテンシャル場の高さ(y軸方向) [m]
							potential_field_resolution_,			//ポテンシャル場グリッド1辺の長さ [m]
							weight_attraction_field_,				//ポテンシャル場における引力場の重み
							weight_repulsion_field_,				//ポテンシャル場における斥力場の重み
							distance_threshold_repulsion_field_		//斥力場を場を作る距離の閾値 [m]
							);
        pp_ = new potbot_lib::path_planner::APFPathPlanner( apf_);
        apf_->setGoal(	goal_.pose.position.x, 		goal_.pose.position.y);
        apf_->setRobot(	robot_.pose.pose.position.x, 	robot_.pose.pose.position.y);
		
		for (const auto&  o:obstacles_)
		{
			Eigen::Vector2d vec;
			vec << o.point.x, o.point.y;
			apf_->setObstacle(vec);
		}
		// apf_->setObstacle(obstacles_);

		for (const auto&  o:interactive_markers_)
		{
			Eigen::Vector2d vec;
			vec << o.pose.position.x; o.pose.position.y;
			apf_->setObstacle(vec);
		}
		// apf_->setObstacle(interactive_markers_);

		timer.start("potential");
        apf_->createPotentialField();
		timer.stop("potential");

		std::vector<std::vector<double>> path_raw, path_interpolated;
		double init_yaw = potbot_lib::utility::get_Yaw(robot_.pose.pose.orientation);
		if (isnan(init_yaw)) init_yaw = 0;

		timer.start("path");
		if (path_weight_potential_ == 0.0 && path_weight_pose_ == 0.0)
		{
			pp_->createPath(path_raw, init_yaw, max_path_length_, path_search_range_);
		}
		else
		{
			pp_->createPathWithWeight(path_raw, init_yaw, max_path_length_, path_search_range_, path_weight_potential_, path_weight_pose_);
		}
		
		pp_->bezier(path_raw, path_interpolated);
		timer.stop("path");

		timer.print_time();

		potbot_lib::potential::Field attraction_field, repulsion_field, potential_field, filtered_field;
		apf_->getAttractionField(attraction_field);
		apf_->getAepulsionField(repulsion_field);
		apf_->getPotentialField(potential_field);
		// potential_field.info_filter(filtered_field, {potbot_lib::Potential::GridInfo::IS_PLANNED_PATH, potbot_lib::Potential::GridInfo::IS_REPULSION_FIELD_EDGE},"and");
		potential_field.infoFilter(filtered_field, potential_field_filter_terms_, potential_field_filter_mode_);

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
		potbot_lib::utility::field_to_pcl2(attraction_field, attraction_field_msg);
		potbot_lib::utility::field_to_pcl2(repulsion_field, repulsion_field_msg);
		potbot_lib::utility::field_to_pcl2(potential_field, potential_field_msg);
		potbot_lib::utility::field_to_pcl2(filtered_field, filtered_field_msg);

		visualization_msgs::MarkerArray loop_edges_msg;
		apf_->getLoopEdges(loop_edges_msg);

        std_msgs::Header header_apf;
        header_apf.frame_id				= "map";
        header_apf.stamp				= ros::Time::now();
        attraction_field_msg.header		= header_apf;
        repulsion_field_msg.header		= header_apf;
        potential_field_msg.header		= header_apf;
		filtered_field_msg.header		= header_apf;
		path_msg_raw.header				= header_apf;
		path_msg_interpolated.header	= header_apf;
		for (auto& m:loop_edges_msg.markers) m.header = header_apf;
		
        pub_attraction_field.publish(attraction_field_msg);
        pub_repulsion_field.publish(repulsion_field_msg);
        pub_potential_field.publish(potential_field_msg);
		pub_filtered_field.publish(filtered_field_msg);
		pub_path_raw.publish(path_msg_raw);
		pub_path_interpolated.publish(path_msg_interpolated);
		pub_loop_edges.publish(loop_edges_msg);

		ros::spinOnce();

		timer.stop("1 loop");
		timer.print_time("1 loop");
		
	}
}

void APFPathPlanner::__init_interactive_marker_server()
{
	__init_interactive_markers();

	imsrv_ = new interactive_markers::InteractiveMarkerServer("simple_marker");
	menu_handler_ = new interactive_markers::MenuHandler;

	interactive_markers::MenuHandler::EntryHandle x_entry = menu_handler_->insert("scale x");
	interactive_markers::MenuHandler::EntryHandle y_entry = menu_handler_->insert("scale y");
	interactive_markers::MenuHandler::EntryHandle xy_entry = menu_handler_->insert("scale xy");

	menu_handler_->insert( x_entry, "x2" , boost::bind(&APFPathPlanner::__marker_feedback, this, _1));
	menu_handler_->insert( x_entry, "x0.5" , boost::bind(&APFPathPlanner::__marker_feedback, this, _1));

	menu_handler_->insert( y_entry, "x2" , boost::bind(&APFPathPlanner::__marker_feedback, this, _1));
	menu_handler_->insert( y_entry, "x0.5" , boost::bind(&APFPathPlanner::__marker_feedback, this, _1));

	menu_handler_->insert( xy_entry, "x2" , boost::bind(&APFPathPlanner::__marker_feedback, this, _1));
	menu_handler_->insert( xy_entry, "x0.5" , boost::bind(&APFPathPlanner::__marker_feedback, this, _1));

	interactive_markers::MenuHandler::EntryHandle type_entry = menu_handler_->insert("marker type");
	menu_handler_->insert( type_entry, "cube" , boost::bind(&APFPathPlanner::__marker_feedback, this, _1));
	menu_handler_->insert( type_entry, "sphere" , boost::bind(&APFPathPlanner::__marker_feedback, this, _1));

	visualization_msgs::Marker move_marker;
	move_marker.type = visualization_msgs::Marker::SPHERE;
	move_marker.scale.x = 0.2;
	move_marker.scale.y = 0.2;
	move_marker.scale.z = 0.2;
	move_marker.color.r = 0.0;
	move_marker.color.g = 0.0;
	move_marker.color.b = 0.7;
	move_marker.color.a = 1.0;
	move_marker.pose = potbot_lib::utility::get_Pose(0,0.5,1,0,0,0);

	visualization_msgs::InteractiveMarkerControl move_control;
	move_control.name = "move_plane";
	move_control.orientation = potbot_lib::utility::get_Quat(0,-M_PI_2,0);
	move_control.always_visible = true;
	move_control.markers.push_back(move_marker);
	move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

	visualization_msgs::InteractiveMarkerControl rotate_control;
	rotate_control.name = "rotate_yaw";
	rotate_control.orientation = potbot_lib::utility::get_Quat(0,-M_PI_2,0);
	rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

	for (size_t i = 0; i < interactive_marker_num_; i++)
	{
		visualization_msgs::InteractiveMarker int_marker;
		int_marker.header.frame_id = "map";
		int_marker.header.stamp = ros::Time::now();
		int_marker.name = "obstacle_" + std::to_string(i);
		int_marker.description = int_marker.name;
		int_marker.pose = potbot_lib::utility::get_Pose(6,0,1,0,0,0);

		move_control.markers[0] = interactive_markers_[i];

		int_marker.controls.push_back(move_control);
		int_marker.controls.push_back(rotate_control);

		imsrv_->insert(int_marker, boost::bind(&APFPathPlanner::__marker_feedback, this, _1));

		interactive_markers_[i].pose = int_marker.pose;
		menu_handler_->apply(*imsrv_, int_marker.name);
	}

	imsrv_->applyChanges();
}

void APFPathPlanner::__init_interactive_markers()
{
	interactive_markers_.resize(interactive_marker_num_);
	visualization_msgs::Marker init_marker;
	init_marker.type = visualization_msgs::Marker::CUBE;
	init_marker.scale.x = 0.5;
	init_marker.scale.y = 0.5;
	init_marker.scale.z = 0.2;
	init_marker.color.r = 0.5;
	init_marker.color.g = 0.5;
	init_marker.color.b = 0.5;
	init_marker.color.a = 1.0;
	init_marker.pose = potbot_lib::utility::get_Pose();
	std::fill(interactive_markers_.begin(), interactive_markers_.end(), init_marker);
}

void APFPathPlanner::inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    robot_ = msg;
}

void APFPathPlanner::goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
}

void APFPathPlanner::point_callback(const geometry_msgs::PointStamped& msg)
{
    obstacles_.push_back(msg);
}

void APFPathPlanner::param_callback(const apf_pathplanner_tutorial::APFPathPlannerTutorialConfig& param, uint32_t level)
{
	potential_field_rows_					= param.potential_field_rows;
	potential_field_cols_					= param.potential_field_cols;
	potential_field_resolution_				= param.potential_field_resolution;
	weight_attraction_field_				= param.weight_attraction_field;
	weight_attraction_field_				= param.weight_repulsion_field;
	distance_threshold_repulsion_field_		= param.distance_threshold_repulsion_field;
	max_path_length_						= param.max_path_length;
	path_search_range_						= param.path_search_range;
	path_weight_potential_					= param.path_weight_potential;
	path_weight_pose_ 						= param.path_weight_pose;

	potential_field_filter_mode_			= param.potential_field_grid_filter;
	potential_field_filter_terms_.clear();
	if (param.goal)							potential_field_filter_terms_.push_back(potbot_lib::Potential::GridInfo::IS_GOAL);
	if (param.robot)						potential_field_filter_terms_.push_back(potbot_lib::Potential::GridInfo::IS_ROBOT);
	if (param.obstacle)						potential_field_filter_terms_.push_back(potbot_lib::Potential::GridInfo::IS_OBSTACLE);
	if (param.repulsion_inside)				potential_field_filter_terms_.push_back(potbot_lib::Potential::GridInfo::IS_REPULSION_FIELD_INSIDE);
	if (param.repulsion_edge)				potential_field_filter_terms_.push_back(potbot_lib::Potential::GridInfo::IS_REPULSION_FIELD_EDGE);
	if (param.path)							potential_field_filter_terms_.push_back(potbot_lib::Potential::GridInfo::IS_PLANNED_PATH);
	if (param.around_goal)					potential_field_filter_terms_.push_back(potbot_lib::Potential::GridInfo::IS_AROUND_GOAL);
	if (param.local_minimum)				potential_field_filter_terms_.push_back(potbot_lib::Potential::GridInfo::IS_LOCAL_MINIMUM);
}

void APFPathPlanner::__marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	std::stringstream ss(feedback->marker_name);
    std::string segment;
    std::vector<std::string> segments;

	// ROS_INFO_STREAM("Menu item " << feedback->menu_entry_id << " clicked in marker " << feedback->marker_name);

    // "_"で文字列を分割
    while (std::getline(ss, segment, '_'))
	{
        segments.push_back(segment);
    }

    if (segments.size() > 1)
	{
		if (segments[0] == "obstacle")
		{
			int id = std::stoi(segments[1]);
			interactive_markers_[id].pose = feedback->pose;

			visualization_msgs::InteractiveMarker int_marker;
			if (imsrv_->get(feedback->marker_name, int_marker)) {
				
				size_t eid = feedback->menu_entry_id;
				if (eid == 4)
				{
					int_marker.controls[0].markers[0].scale.x *= 2;
				}
				else if (eid == 5)
				{
					int_marker.controls[0].markers[0].scale.x *= 0.5;
				}
				else if (eid == 6)
				{
					int_marker.controls[0].markers[0].scale.y *= 2;
				}
				else if (eid == 7)
				{
					int_marker.controls[0].markers[0].scale.y *= 0.5;
				}
				else if (eid == 8)
				{
					int_marker.controls[0].markers[0].scale.x *= 2;
					int_marker.controls[0].markers[0].scale.y *= 2;
				}
				else if (eid == 9)
				{
					int_marker.controls[0].markers[0].scale.x *= 0.5;
					int_marker.controls[0].markers[0].scale.y *= 0.5;
				}
				else if (eid == 11)
				{
					int_marker.controls[0].markers[0].type = visualization_msgs::Marker::CUBE;
				}
				else if (eid == 12)
				{
					int_marker.controls[0].markers[0].type = visualization_msgs::Marker::SPHERE;
				}

				interactive_markers_[id].scale = int_marker.controls[0].markers[0].scale;
				interactive_markers_[id].type = int_marker.controls[0].markers[0].type;

				// 変更をサーバーに反映
				imsrv_->insert(int_marker, boost::bind(&APFPathPlanner::__marker_feedback, this, _1));
				imsrv_->applyChanges();
			}
		}
    }
}

int main(int argc,char **argv){
	ros::init(argc,argv,"apf_pathplanner_tutorial");
	APFPathPlanner apf;
	return 0;
}