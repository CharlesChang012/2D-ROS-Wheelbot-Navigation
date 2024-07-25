#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <pluginlib/class_list_macros.h>

using std::string;
using std::vector;
#ifndef ASTAR_PLANNER_H_
#define ASTAR_PLANNER_H_


namespace astar_planner{

	class posInfo{
	   	public:
	   		int pos_i;
	   		int pos_j;
	   		double pos_yaw;
	   		double fcost;
	};


	struct compare{
	 	bool operator()(posInfo &first, posInfo &second){
			return first.fcost > second.fcost;
		}
	};

	class AstarPlanner : public nav_core::BaseGlobalPlanner {
		public:
			AstarPlanner();
			AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

			/** overridden classes from interface nav_core::BaseGlobalPlanner **/
			void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

			bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

		private:
			costmap_2d::Costmap2DROS* costmap_ros_;
			costmap_2d::Costmap2D* costmap_;
			base_local_planner::WorldModel* world_model_;
			bool initialized_;
			
			double footprintCost(double x_i, double y_i, double theta_i);
			
			void ijToWorld(int pos_i, int pos_j, double &posWorld_x, double &posWorld_y, int mapWidth_x, int mapHeight_y);
			void worldToij(double posWorld_x, double posWorld_y, int &pos_i, int &pos_j, int mapWidth_x, int mapHeight_y);
	};
};
#endif
