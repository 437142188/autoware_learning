# motion note

在quick_start demo中，使用runtime manager 加载相应的motion launch file,其中，涉及到的包有
rostopic
runtime_manager
astar_planner
	obstacle_avoid
	velocity_set
waypoint_follower
	pure_pursuit
	twist_filter
	
	
## obstacle_avoid
障碍物绕行模块
  	// ROS subscribers
  ros::Subscriber map_sub = n.subscribe("grid_map_visualization/distance_transform", 1,
                                        &astar_planner::SearchInfo::mapCallback, &search_info);
  ros::Subscriber start_sub =
      n.subscribe("current_pose", 1, &astar_planner::SearchInfo::currentPoseCallback, &search_info);
  ros::Subscriber waypoints_sub =
      n.subscribe("base_waypoints", 1, &astar_planner::SearchInfo::waypointsCallback, &search_info);
  ros::Subscriber obstacle_waypoint_sub =
      n.subscribe("obstacle_waypoint", 1, &astar_planner::SearchInfo::obstacleWaypointCallback, &search_info);
  ros::Subscriber closest_waypoint_sub =
      n.subscribe("closest_waypoint", 1, &astar_planner::SearchInfo::closestWaypointCallback, &search_info);
  ros::Subscriber current_velocity_sub =
      n.subscribe("current_velocity", 1, &astar_planner::SearchInfo::currentVelocityCallback, &search_info);
  ros::Subscriber state_sub = n.subscribe("state", 1, &astar_planner::SearchInfo::stateCallback, &search_info);
  
    // ROS publishers
  ros::Publisher path_pub = n.advertise<nav_msgs::Path>("astar_path", 1, true);
  ros::Publisher waypoints_pub = n.advertise<autoware_msgs::lane>("safety_waypoints", 1, true);
  
其中，当接收到obs的信息后，会初始化地图、初始点和目标点信息，调用Astar完成路径规划。obs会传递过来一个waypoint的index信息，表示该段waypoint需要进行避障，然后
根据obs的index会加上相应的distance作为goal index，减去distance作为start index。最后判断如果地图信息、初始点和目标点已经初始化好了，则执行Astar算法。最终
输出规划出来的路径和路点。

## velocity_set

  // velocity set subscriber
  ros::Subscriber waypoints_sub = nh.subscribe("safety_waypoints", 1, &VelocitySetPath::waypointsCallback, &vs_path);
  ros::Subscriber current_vel_sub =
      nh.subscribe("current_velocity", 1, &VelocitySetPath::currentVelocityCallback, &vs_path);

  // velocity set info subscriber
  ros::Subscriber config_sub = nh.subscribe("config/velocity_set", 1, &VelocitySetInfo::configCallback, &vs_info);
  ros::Subscriber points_sub = nh.subscribe(points_topic, 1, &VelocitySetInfo::pointsCallback, &vs_info);
  ros::Subscriber localizer_sub = nh.subscribe("localizer_pose", 1, &VelocitySetInfo::localizerPoseCallback, &vs_info);
  ros::Subscriber control_pose_sub = nh.subscribe("current_pose", 1, &VelocitySetInfo::controlPoseCallback, &vs_info);
  ros::Subscriber obstacle_sim_points_sub = nh.subscribe("obstacle_sim_pointcloud", 1, &VelocitySetInfo::obstacleSimCallback, &vs_info);
  ros::Subscriber detectionresult_sub = nh.subscribe("/state/stopline_wpidx", 1, &VelocitySetInfo::detectionCallback, &vs_info);

  // vector map subscriber
  ros::Subscriber sub_dtlane = nh.subscribe("vector_map_info/cross_walk", 1, &CrossWalk::crossWalkCallback, &crosswalk);
  ros::Subscriber sub_area = nh.subscribe("vector_map_info/area", 1, &CrossWalk::areaCallback, &crosswalk);
  ros::Subscriber sub_line = nh.subscribe("vector_map_info/line", 1, &CrossWalk::lineCallback, &crosswalk);
  ros::Subscriber sub_point = nh.subscribe("vector_map_info/point", 1, &CrossWalk::pointCallback, &crosswalk);

  // publisher
  ros::Publisher detection_range_pub = nh.advertise<visualization_msgs::MarkerArray>("detection_range", 1);
  ros::Publisher obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 1);
  ros::Publisher obstacle_waypoint_pub = nh.advertise<std_msgs::Int32>("obstacle_waypoint", 1, true);

  ros::Publisher final_waypoints_pub;
  if(enablePlannerDynamicSwitch){
	  final_waypoints_pub = nh.advertise<autoware_msgs::lane>("astar/final_waypoints", 1, true);
  }else{
	  final_waypoints_pub = nh.advertise<autoware_msgs::lane>("final_waypoints", 1, true);
  }
  
  该模块接收来自obstacle_avoid模块的safety_waypoints数据，最终输出final_waypoints，给pure_pursuit模块使用。模块中会根据状态判断是否需要stop、decelerate
  或者keep，从而修正waypoint。
  
  
## pure_pursuit

纯追踪算法，用于跟踪规划出的路径，输出控制量，参考https://blog.csdn.net/yzdhit/article/details/72382426

## pure_pursuit

对pure_pursuit节点输出的控制量进行低通滤波，参考https://blog.csdn.net/yzdhit/article/details/72520444