#include <ros/ros.h>
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

#include <potbot_base/base_path_planner.h>
#include <pluginlib/class_loader.h>

class MarkerPathPlanner
{
	private:
		geometry_msgs::PoseWithCovarianceStamped robot_;
		geometry_msgs::PoseStamped goal_;
		std::vector<geometry_msgs::Point> obstacles_;

		// potbot_lib::path_planner::APFPathPlanner* pp_;
		// potbot_lib::ArtificialPotentialField* apf_;

		std::string path_planning_method_		= "edge";

		size_t interactive_marker_num_ = 3;
		std::vector<visualization_msgs::Marker> interactive_markers_;

		dynamic_reconfigure::Server<apf_pathplanner_tutorial::APFPathPlannerTutorialConfig> *dsrv_;
		interactive_markers::InteractiveMarkerServer *imsrv_;
		interactive_markers::MenuHandler *menu_handler_;

		boost::shared_ptr<potbot_base::PathPlanner> planner_;
		pluginlib::ClassLoader<potbot_base::PathPlanner> loader_;

		void inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
		void goal_callback(const geometry_msgs::PoseStamped& msg);
		void point_callback(const geometry_msgs::PointStamped& msg);
		void param_callback(const apf_pathplanner_tutorial::APFPathPlannerTutorialConfig& param, uint32_t level);

		void __marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

		void __init_interactive_markers();
		void __init_interactive_marker_server();

	public:
		MarkerPathPlanner(tf2_ros::Buffer* tf);
		~MarkerPathPlanner(){};
};

void toPointVec(const visualization_msgs::Marker& obs, std::vector<geometry_msgs::Point>& points)
{
	double origin_x = obs.pose.position.x;
	double origin_y = obs.pose.position.y;
	double origin_th = tf2::getYaw(obs.pose.orientation);
	double res = 0.05;

	Eigen::MatrixXd vertexes;
	if (obs.type == visualization_msgs::Marker::CUBE)
	{
		double width = obs.scale.x;
		double height = obs.scale.y;

		Eigen::Matrix2d rotation = potbot_lib::utility::get_rotate_matrix(origin_th);
		Eigen::MatrixXd translation(4,2);
		translation <<  origin_x, origin_y,
						origin_x, origin_y,
						origin_x, origin_y,
						origin_x, origin_y;
		Eigen::MatrixXd origin_vertexes(4,2);
		origin_vertexes <<  -width/2,   -height/2,
							-width/2,   height/2,
							width/2,    height/2,
							width/2,    -height/2;

		vertexes = rotation*origin_vertexes.transpose() + translation.transpose();
		
	}
	else if (obs.type == visualization_msgs::Marker::SPHERE)
	{
		double width = obs.scale.x;
		double height = obs.scale.y;

		Eigen::Matrix2d rotation = potbot_lib::utility::get_rotate_matrix(origin_th);
		Eigen::Vector2d translation;
		translation <<  origin_x, origin_y;
		Eigen::MatrixXd origin_vertexes(4,2);
		
		size_t vertex_num = 2*M_PI/res;
		vertexes.resize(2,vertex_num);
		for (size_t i = 0; i < vertex_num; i++)
		{
			double t = 2 * M_PI * i / vertex_num;
			double x = width/2 * cos(t);
			double y = height/2 * sin(t);
			Eigen::Vector2d p;
			p<< x,y;
			vertexes.col(i) = rotation*p + translation;
		}
	}

	for (size_t i = 0; i < vertexes.cols(); i++)
	{
		// std::cout<<vertexes.row(i)<<std::endl;
		points.push_back(potbot_lib::utility::get_point(vertexes(0,i), vertexes(1,i)));
	}
}

void toPointVec(const std::vector<visualization_msgs::Marker>& obs, std::vector<geometry_msgs::Point>& points)
{
	for (const auto& o:obs)
	{
		toPointVec(o, points);
	}
}

MarkerPathPlanner::MarkerPathPlanner(tf2_ros::Buffer* tf) : loader_("potbot_base", "potbot_base::PathPlanner")
{
	ros::NodeHandle nh;

	ros::Subscriber sub_inipose				= nh.subscribe("initialpose",1,&MarkerPathPlanner::inipose_callback,this);
	ros::Subscriber sub_goal				= nh.subscribe("move_base_simple/goal",1,&MarkerPathPlanner::goal_callback,this);
	ros::Subscriber sub_point				= nh.subscribe("clicked_point",1,&MarkerPathPlanner::point_callback,this);

	std::string plugin_name = "potbot_nav/APF";
	// private_nh.getParam("controller_name", plugin_name);
	try
	{
		planner_ = loader_.createInstance(plugin_name);
		planner_->initialize(plugin_name, tf);
		ROS_INFO("\t%s initialized", plugin_name.c_str());
	}
	catch(pluginlib::PluginlibException& ex)
	{
		ROS_ERROR("failed to load plugin. Error: %s", ex.what());
	}

	dsrv_ = new dynamic_reconfigure::Server<apf_pathplanner_tutorial::APFPathPlannerTutorialConfig>(ros::NodeHandle("~"));
	dynamic_reconfigure::Server<apf_pathplanner_tutorial::APFPathPlannerTutorialConfig>::CallbackType cb = boost::bind(&MarkerPathPlanner::param_callback, this, _1, _2);
	dsrv_->setCallback(cb);

	__init_interactive_marker_server();

	while (ros::ok())
	{
		potbot_lib::utility::Timer timer;
        timer.start("1 loop");

		nav_msgs::Odometry nav_robot;
		nav_robot.pose = robot_.pose;
		planner_->setRobot(nav_robot);
		planner_->setTargetPose(goal_);
		std::vector<geometry_msgs::Point> marker_obs;
		toPointVec(interactive_markers_, marker_obs);
		planner_->clearObstacles();
		planner_->setObstacles(obstacles_);
		planner_->setObstacles(marker_obs);
		planner_->planPath();
		
		timer.print_time();

		ros::spinOnce();

		timer.stop("1 loop");
		timer.print_time("1 loop");
	}
}

void MarkerPathPlanner::__init_interactive_marker_server()
{
	__init_interactive_markers();

	imsrv_ = new interactive_markers::InteractiveMarkerServer("simple_marker");
	menu_handler_ = new interactive_markers::MenuHandler;

	interactive_markers::MenuHandler::EntryHandle x_entry = menu_handler_->insert("scale x");
	interactive_markers::MenuHandler::EntryHandle y_entry = menu_handler_->insert("scale y");
	interactive_markers::MenuHandler::EntryHandle xy_entry = menu_handler_->insert("scale xy");

	menu_handler_->insert( x_entry, "x2" , boost::bind(&MarkerPathPlanner::__marker_feedback, this, _1));
	menu_handler_->insert( x_entry, "x0.5" , boost::bind(&MarkerPathPlanner::__marker_feedback, this, _1));

	menu_handler_->insert( y_entry, "x2" , boost::bind(&MarkerPathPlanner::__marker_feedback, this, _1));
	menu_handler_->insert( y_entry, "x0.5" , boost::bind(&MarkerPathPlanner::__marker_feedback, this, _1));

	menu_handler_->insert( xy_entry, "x2" , boost::bind(&MarkerPathPlanner::__marker_feedback, this, _1));
	menu_handler_->insert( xy_entry, "x0.5" , boost::bind(&MarkerPathPlanner::__marker_feedback, this, _1));

	interactive_markers::MenuHandler::EntryHandle type_entry = menu_handler_->insert("marker type");
	menu_handler_->insert( type_entry, "cube" , boost::bind(&MarkerPathPlanner::__marker_feedback, this, _1));
	menu_handler_->insert( type_entry, "sphere" , boost::bind(&MarkerPathPlanner::__marker_feedback, this, _1));

	visualization_msgs::Marker move_marker;
	move_marker.type = visualization_msgs::Marker::SPHERE;
	move_marker.scale.x = 0.2;
	move_marker.scale.y = 0.2;
	move_marker.scale.z = 0.2;
	move_marker.color.r = 0.0;
	move_marker.color.g = 0.0;
	move_marker.color.b = 0.7;
	move_marker.color.a = 1.0;
	move_marker.pose = potbot_lib::utility::get_pose(0,0.5,1,0,0,0);

	visualization_msgs::InteractiveMarkerControl move_control;
	move_control.name = "move_plane";
	move_control.orientation = potbot_lib::utility::get_quat(0,-M_PI_2,0);
	move_control.always_visible = true;
	move_control.markers.push_back(move_marker);
	move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

	visualization_msgs::InteractiveMarkerControl rotate_control;
	rotate_control.name = "rotate_yaw";
	rotate_control.orientation = potbot_lib::utility::get_quat(0,-M_PI_2,0);
	rotate_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;

	for (size_t i = 0; i < interactive_marker_num_; i++)
	{
		visualization_msgs::InteractiveMarker int_marker;
		int_marker.header.frame_id = "map";
		int_marker.header.stamp = ros::Time::now();
		int_marker.name = "obstacle_" + std::to_string(i);
		int_marker.description = int_marker.name;
		int_marker.pose = potbot_lib::utility::get_pose(6,0,1,0,0,0);

		move_control.markers[0] = interactive_markers_[i];

		int_marker.controls.push_back(move_control);
		int_marker.controls.push_back(rotate_control);

		imsrv_->insert(int_marker, boost::bind(&MarkerPathPlanner::__marker_feedback, this, _1));

		interactive_markers_[i].pose = int_marker.pose;
		menu_handler_->apply(*imsrv_, int_marker.name);
	}

	imsrv_->applyChanges();
}

void MarkerPathPlanner::__init_interactive_markers()
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
	init_marker.pose = potbot_lib::utility::get_pose();
	std::fill(interactive_markers_.begin(), interactive_markers_.end(), init_marker);
}

void MarkerPathPlanner::inipose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    robot_ = msg;
}

void MarkerPathPlanner::goal_callback(const geometry_msgs::PoseStamped& msg)
{
    goal_ = msg;
}

void MarkerPathPlanner::point_callback(const geometry_msgs::PointStamped& msg)
{
    obstacles_.push_back(msg.point);
}

void MarkerPathPlanner::param_callback(const apf_pathplanner_tutorial::APFPathPlannerTutorialConfig& param, uint32_t level)
{
	path_planning_method_ = param.path_planning_method;
}

void MarkerPathPlanner::__marker_feedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
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
				imsrv_->insert(int_marker, boost::bind(&MarkerPathPlanner::__marker_feedback, this, _1));
				imsrv_->applyChanges();
			}
		}
    }
}

int main(int argc,char **argv){
	ros::init(argc,argv,"pathplanner_tutorial");
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tfListener(tf_buffer_);
	MarkerPathPlanner mpp(&tf_buffer_);
	return 0;
}