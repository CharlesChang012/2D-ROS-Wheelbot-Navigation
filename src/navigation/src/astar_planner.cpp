 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include <pluginlib/class_list_macros.h>
 #include "astar_planner/astar_planner.h"


// Register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(astar_planner::AstarPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace astar_planner{
   		
	AstarPlanner::AstarPlanner() : costmap_ros_(NULL), initialized_(false){}
	
	AstarPlanner::AstarPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros) 
	: costmap_ros_(NULL), initialized_(false){
		initialize(name, costmap_ros);
	}
	
	void AstarPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
		if(!initialized_){
			costmap_ros_ = costmap_ros; //initialize the costmap_ros_ attribute to the parameter.
			costmap_ = costmap_ros_->getCostmap(); //get the costmap_ from costmap_ros_

			// initialize other planner parameters
			ros::NodeHandle private_nh("~/" + name);
			world_model_ = new base_local_planner::CostmapModel(*costmap_);

			initialized_ = true;
			ROS_INFO("Finish Initialization!\n");
		}
		else
			ROS_WARN("This planner has already been initialized... do nothing");
	}
	
	//we need to take the footprint of the robot into account when we calculate cost to obstacles
	double AstarPlanner::footprintCost(double x_i, double y_i, double theta_i){  // x, y in meters
		if(!initialized_){
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
			return -1.0;
		}

		std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
		//if we have no footprint... do nothing
		if(footprint.size() < 3)
			return -1.0;

		//check if the footprint is legal
		double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
		
		return footprint_cost;
   }
   	
   	// Helper function to convert 2D matrix i, j into world coord
   	void AstarPlanner::ijToWorld(int pos_i, int pos_j, double &posWorld_x, double &posWorld_y, int mapWidth_x, int mapHeight_y){

		int newPosMap_x = pos_j+1;
		int newPosMap_y = mapHeight_y-pos_i+1;

		costmap_ -> mapToWorld(newPosMap_x, newPosMap_y, posWorld_x, posWorld_y);
   	}
   	
   	void AstarPlanner::worldToij(double posWorld_x, double posWorld_y, int &pos_i, int &pos_j, int mapWidth_x, int mapHeight_y){
		unsigned int posMap_x, posMap_y;
	 	costmap_ -> worldToMap(posWorld_x, posWorld_y, posMap_x, posMap_y);

	 	// Convert map frame coord to c++ 2d matrix coor
	 	pos_i = mapHeight_y-posMap_y+1;
	 	pos_j = posMap_x-1;
   	}

   	
	//TODO Astar search algorithm
	bool AstarPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
 
		if(!initialized_){
			ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
			return false;
		}
	 
		ROS_DEBUG("Got a start: (%.2f, %.2f), and a goal: (%.2f, %.2f)", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);
	 
		plan.clear();
	 	costmap_ = costmap_ros_->getCostmap();
	 
		if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
			ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
			   costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
			return false;
		}
		ROS_INFO("Start making plan...\n");
		
		tf::Stamped<tf::Pose> goal_tf;
		tf::Stamped<tf::Pose> start_tf;

		poseStampedMsgToTF(goal,goal_tf);
		poseStampedMsgToTF(start,start_tf);
	 
		double useless_pitch, useless_roll, goal_yaw, start_yaw;
		start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
		goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

		// Map configuration data
		unsigned int mapWidth_x = costmap_ -> getSizeInCellsX();
		unsigned int mapHeight_y = costmap_ -> getSizeInCellsY();
		double mapResolution = costmap_ -> getResolution();
		
		// Set start and goal 
		double goal_x = goal.pose.position.x;
		double goal_y = goal.pose.position.y;
		double start_x = start.pose.position.x;
		double start_y = start.pose.position.y;
	 	
		double diff_x = goal_x - start_x;
		double diff_y = goal_y - start_y;
		double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);

	 	// Convert world to matrix ij coord (world frame: origin at left-bottom, map frame: origin at center, matrix frame: origin at (i = 0, j = 0))
	 	int start_i, start_j;
	 	worldToij(start_x, start_y, start_i, start_j, mapWidth_x, mapHeight_y);
	 	
	 	int goal_i, goal_j;
	 	worldToij(goal_x, goal_y, goal_i, goal_j, mapWidth_x, mapHeight_y);

		// Parent map to store the path
		vector<vector<int>> parent_i(mapHeight_y, vector<int> (mapWidth_x, -1));
		vector<vector<int>> parent_j (mapHeight_y, vector<int> (mapWidth_x));
		vector<vector<double>> parent_yaw(mapHeight_y, vector<double> (mapWidth_x));
	 	
	 	// Initialize cost arrays
	 	std::vector<std::vector<double>> g(mapHeight_y, std::vector<double> (mapWidth_x, 1000000000));
	 	g[start_i][start_j] = 0;
	 	
	 	std::vector<std::vector<double>> f(mapHeight_y, std::vector<double> (mapWidth_x, 1000000000));
	 	f[start_i][start_j] = sqrt(pow((start_i-goal_i),2)) + sqrt(pow((start_j-goal_j),2));
	 	
	 	// Initialize yawInfo
	 	std::vector<std::vector<double>> yawInfo(mapHeight_y, std::vector<double> (mapWidth_x));
	 	yawInfo[start_i][start_j] = start_yaw;
	 	
	 	// Record visited cells
	 	std::vector<std::vector<bool>> visited(mapHeight_y, std::vector<bool> (mapWidth_x, false));
	 	// Record expanded cells
	 	vector<vector<bool>> expanded(mapHeight_y, vector<bool> (mapWidth_x, false));

	 	// Main algorithm
	 	ROS_INFO("In main algorithm!\n");
	 	
	 	//Store next to explore coords
	 	posInfo startPos;
	 	startPos.fcost = f[start_i][start_j];
		startPos.pos_i = start_i;
		startPos.pos_j = start_j;
		startPos.pos_yaw = start_yaw;
		
	 	priority_queue<posInfo, vector<posInfo>, compare> pq;
	 	pq.push(startPos);

		while(!pq.empty())
		{	
		    posInfo curPos;
			curPos = pq.top();
			pq.pop();
			
			double minfCost = curPos.fcost;
			int cur_i = curPos.pos_i;
			int cur_j = curPos.pos_j;
			double cur_yaw = curPos.pos_yaw;
			
			if((cur_i == goal_i && cur_j == goal_j) || minfCost == 1000000000) break;
			
			visited[cur_i][cur_j] = true;
			
			// Update cost function in four direction
			// In c++ 2D matrix coord
			int movements[8][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, 1}, {-1, 1}, {1, -1}}; //
			for(int k = 0; k < 8; k++){
				int newPos_i = cur_i + movements[k][0]; // y-direction
				int newPos_j = cur_j + movements[k][1]; // x-direction
				double diff_yaw = atan2(movements[k][0], movements[k][1]);
				double newPos_yaw = cur_yaw + diff_yaw;  // in radian
				
				// Tranform ij frame to world frame
				double newPos_x;
				double newPos_y;
				ijToWorld(newPos_i, newPos_j, newPos_x, newPos_y, mapWidth_x, mapHeight_y);

	 			// footprint_cost >= 0: probability occur obstacle, -1: lethal_obstable(254), -2: no info (255), -3: out of bound; 
	 			double footprint_cost = footprintCost(newPos_x, newPos_y, newPos_yaw);
	 			 
				// Ckeck if new pos is out of bound or visited
				if(newPos_i < 0 || newPos_i >= mapHeight_y || newPos_j < 0 || newPos_j >= mapWidth_x || footprint_cost < 0 || footprint_cost >= 128 || visited[newPos_i][newPos_j]) continue;
				
	 			// Calculate cost
	 			int g_new = (k <= 3) ? g[cur_i][cur_j] + 1 : g[cur_i][cur_j] + 1.4;
	 			int f_new = g_new + sqrt(pow((newPos_i-goal_i),2)) + sqrt(pow((newPos_j-goal_j),2));
	 			
			 	posInfo newPos;
			 	newPos.fcost = f_new;
				newPos.pos_i = newPos_i;
				newPos.pos_j = newPos_j;
				newPos.pos_yaw = newPos_yaw;
				
				if(!expanded[newPos_i][newPos_j] || f_new < f[newPos_i][newPos_j]){
					expanded[newPos_i][newPos_j] = true;
					
					// Update cost
					g[newPos_i][newPos_j] =  g_new;
					f[newPos_i][newPos_j] = f_new;
					
					// Update parent map
					parent_i[newPos_i][newPos_j] = cur_i;
					parent_j[newPos_i][newPos_j] = cur_j;
					parent_yaw[newPos_i][newPos_j] = cur_yaw;
					
					pq.push(newPos);
				
				}
				
			}
		}

		// Construct plan path from backtracking from parent map
		ROS_INFO("Constructing Map\n");
		
		int newPathNode_i = parent_i[goal_i][goal_j];
		int newPathNode_j = parent_j[goal_i][goal_j];
		plan.push_back(goal);
		
		while(parent_i[newPathNode_i][newPathNode_j] != -1){
		
			geometry_msgs::PoseStamped newPathNode;
			double newPosWorld_x, newPosWorld_y;
			ijToWorld(newPathNode_i, newPathNode_j, newPosWorld_x, newPosWorld_y, mapWidth_x, mapHeight_y);
		
			tf::Quaternion newPathNode_quat = tf::createQuaternionFromYaw(parent_yaw[newPathNode_i][newPathNode_j]);

			newPathNode.pose.position.x = newPosWorld_x;
			newPathNode.pose.position.y = newPosWorld_y;
			
			newPathNode.pose.orientation.x = newPathNode_quat.x();
			newPathNode.pose.orientation.y = newPathNode_quat.y();
			newPathNode.pose.orientation.z = newPathNode_quat.z();
			newPathNode.pose.orientation.w = newPathNode_quat.w();
			
			plan.push_back(newPathNode);
			
			newPathNode_i = parent_i[newPathNode_i][newPathNode_j];
			newPathNode_j = parent_j[newPathNode_i][newPathNode_j];
			
		}
		
		plan.push_back(start);
	
		reverse(plan.begin(), plan.end());
		
		return true;
	}
       
};
