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
	
## astar_planner	
	
### astar_planner-obstacle_avoid
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

### astar_planner-velocity_set

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
  或者keep，从而修正waypoint。算法根据道路上的点云信息，判断障碍物相对应的路点位置，再根据stop_range和deceleration_range判断应该减速还是停止，最后修正之前的
  路点速度信息，发布出去。
  
## waypoint_follower
  
### waypoint_follower-pure_pursuit

纯追踪算法，用于跟踪规划出的路径，输出控制量，参考https://blog.csdn.net/yzdhit/article/details/72382426

### waypoint_follower-twist_filter

对pure_pursuit节点输出的控制量进行低通滤波，参考https://blog.csdn.net/yzdhit/article/details/72520444

## lattice_planner
子node调用顺序：
lattice_velocity_set->path_select->lattice_trajectory_gen->lattice_twist_convert

lattice_velocity_set：对全局路径进行速度设置，其中会处理障碍物信息
path_select：将temporal_waypoints话题传递给final_waypoints
lattice_trajectory_gen：生成lattice轨迹
lattice_twist_convert:将生成后的轨迹转换为相应的速度指令


### lattice_planner-lattice_trajectory_gen

  // Subscribe to the following topics: 
  ros::Subscriber waypoint_subcscriber = nh.subscribe("final_waypoints", 1, WayPointCallback);
  ros::Subscriber current_pose_subscriber = nh.subscribe("current_pose", 1, currentPoseCallback);
  ros::Subscriber current_vel_subscriber = nh.subscribe("current_velocity", 1, currentVelCallback);
  ros::Subscriber config_subscriber = nh.subscribe("config/waypoint_follower", 1, ConfigCallback);
  
    // Publish the following topics: 
  g_vis_pub = nh.advertise<visualization_msgs::Marker>("next_waypoint_mark", 1);
  g_stat_pub = nh.advertise<std_msgs::Bool>("wf_stat", 0);
  // Publish the curvature information:
  ros::Publisher spline_parameters_pub = nh.advertise<std_msgs::Float64MultiArray>("spline", 10);
  ros::Publisher state_parameters_pub = nh.advertise<std_msgs::Float64MultiArray>("state", 10);
  // Publish the trajectory visualization
  g_marker_pub = nh.advertise<visualization_msgs::Marker>("cubic_splines_viz", 10);
  
  
  输入：final_waypoint、current_pose
  输出：spline、state
  
  算法使用三次样条算法生成平滑轨迹，参考https://blog.csdn.net/adamshan/article/details/80696881,最终实现了对输入的路点的平滑，保证了其速度和加速度的连续性。不过似乎每次调用算法，只优化最近路点与下一路点之间
  的轨迹。

生成latiice算法使用的轨迹

### lattice_planner-lattice_twist_convert

  // Subscribe to the following topics:
  // Curvature parameters and state parameters
  ros::Subscriber spline_parameters = nh.subscribe("spline", 1, splineCallback);
  ros::Subscriber state_parameters = nh.subscribe("state", 1, stateCallback);
  
  // Publish the following topics:
  // Commands
  ros::Publisher cmd_velocity_publisher = nh.advertise<geometry_msgs::TwistStamped>("twist_raw", 10);
  
  该模块用于根据lattice_trajectory_gen生成的优化轨迹发布相对应的速度控制命令。包括线速度和角速度。
  
### lattice_planner-lattice_velocity_set

  ros::Subscriber localizer_sub = nh.subscribe("localizer_pose", 1, localizerCallback);
  ros::Subscriber control_pose_sub = nh.subscribe("current_pose", 1, controlCallback);
  ros::Subscriber vscan_sub = nh.subscribe("vscan_points", 1, vscanCallback);
  ros::Subscriber base_waypoint_sub = nh.subscribe("base_waypoints", 1, baseWaypointCallback);
  ros::Subscriber obj_pose_sub = nh.subscribe("obj_pose", 1, objPoseCallback);
  ros::Subscriber current_vel_sub = nh.subscribe("current_velocity", 1, currentVelCallback);
  ros::Subscriber config_sub = nh.subscribe("config/lattice_velocity_set", 10, configCallback);

  //------------------ Vector Map ----------------------//
  ros::Subscriber sub_dtlane = nh.subscribe("vector_map_info/cross_walk", 1, &CrossWalk::crossWalkCallback, &vmap);
  ros::Subscriber sub_area = nh.subscribe("vector_map_info/area", 1, &CrossWalk::areaCallback, &vmap);
  ros::Subscriber sub_line = nh.subscribe("vector_map_info/line", 1, &CrossWalk::lineCallback, &vmap);
  ros::Subscriber sub_point = nh.subscribe("vector_map_info/point", 1, &CrossWalk::pointCallback, &vmap);
  
  g_range_pub = nh.advertise<visualization_msgs::MarkerArray>("detection_range", 0);
  g_sound_pub = nh.advertise<std_msgs::String>("sound_player", 10);
  g_temporal_waypoints_pub = nh.advertise<autoware_msgs::lane>("temporal_waypoints", 1000, true);
  ros::Publisher closest_waypoint_pub;
  closest_waypoint_pub = nh.advertise<std_msgs::Int32>("closest_waypoint", 1000);
  g_obstacle_pub = nh.advertise<visualization_msgs::Marker>("obstacle", 0);
  
  与astar_planner中的velocity_set功能类似，用于对订阅的base_waypoint得到的waypoint进行速度修正，涉及到遇到障碍物是否减速、停止等问题。
  
### lattice_planner-path_select

    ros::Subscriber twist_sub = nh.subscribe("temporal_waypoints", 1, callback);
    
    _pub = nh.advertise<autoware_msgs::lane>("final_waypoints", 1000,true);
    
该节点把收到的temporal_waypoints发布到final_waypoints上
