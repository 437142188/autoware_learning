# mission_note

在quick_start demo中，使用runtime manager 加载相应的mission launch file,其中，涉及到的包有
rostopic
runtime_manager
autoware_connector
waypoint_maker
lane_planner
	lane_navi
	lane_rule
	lane_stop
	lane_select
 

## waypoint_maker

	加载路径文件



## lane_navi
 
 	//ros subcriber
 	ros::Subscriber route_sub = n.subscribe("/route_cmd", sub_route_queue_size, create_waypoint);
	ros::Subscriber point_sub = n.subscribe("/vector_map_info/point", sub_vmap_queue_size, cache_point);
	ros::Subscriber lane_sub = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
	ros::Subscriber node_sub = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);

	//ros publish
	waypoint_pub = n.advertise<autoware_msgs::LaneArray>("/lane_waypoints_array", pub_waypoint_queue_size,
								 pub_waypoint_latch);

								 
## lane_rule

	//ros subcriber
	ros::Subscriber waypoint_sub = n.subscribe("/lane_waypoints_array", sub_waypoint_queue_size, create_waypoint);
	ros::Subscriber point_sub = n.subscribe("/vector_map_info/point", sub_vmap_queue_size, cache_point);
	ros::Subscriber lane_sub = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
	ros::Subscriber node_sub = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);
	ros::Subscriber stopline_sub = n.subscribe("/vector_map_info/stop_line", sub_vmap_queue_size, cache_stopline);
	ros::Subscriber dtlane_sub = n.subscribe("/vector_map_info/dtlane", sub_vmap_queue_size, cache_dtlane);
	ros::Subscriber config_sub = n.subscribe("/config/lane_rule", sub_config_queue_size, config_parameter);
	
	//ros publish
	traffic_pub = n.advertise<autoware_msgs::LaneArray>("/traffic_waypoints_array", pub_waypoint_queue_size,
								pub_waypoint_latch);
	red_pub = n.advertise<autoware_msgs::LaneArray>("/red_waypoints_array", pub_waypoint_queue_size,
							    pub_waypoint_latch);
	green_pub = n.advertise<autoware_msgs::LaneArray>("/green_waypoints_array", pub_waypoint_queue_size,
							      pub_waypoint_latch);

							      
							      
## lane_select

  // setup subscriber
  sub1_ = nh_.subscribe("traffic_waypoints_array", 1, &LaneSelectNode::callbackFromLaneArray, this);
  sub2_ = nh_.subscribe("current_pose", 1, &LaneSelectNode::callbackFromPoseStamped, this);
  sub3_ = nh_.subscribe("current_velocity", 1, &LaneSelectNode::callbackFromTwistStamped, this);
  sub4_ = nh_.subscribe("state", 1, &LaneSelectNode::callbackFromState, this);
  sub5_ = nh_.subscribe("/config/lane_select", 1, &LaneSelectNode::callbackFromConfig, this);
  sub6_ = nh_.subscribe("/decisionmaker/states", 1, &LaneSelectNode::callbackFromStates, this);
  
    // setup publisher
  pub1_ = nh_.advertise<autoware_msgs::lane>("base_waypoints", 1);

  if (enablePlannerDynamicSwitch)
  {
    pub2_ = nh_.advertise<std_msgs::Int32>("/astar/closest_waypoint", 1);
  }
  else
  {
    pub2_ = nh_.advertise<std_msgs::Int32>("closest_waypoint", 1);
  }

  pub3_ = nh_.advertise<std_msgs::Int32>("change_flag", 1);
  pub4_ = nh_.advertise<std_msgs::Int32>("/current_lane_id",1);

  vis_pub1_ = nh_.advertise<visualization_msgs::MarkerArray>("lane_select_marker", 1);
  
  该节点对lane进行选择，根据决策模块输出的决策量/decisionmaker/states，决定是否需要更换lane，输入lane_rule模块的traffic_waypoints_array，输出closest_waypoint
  
  
##lane_stop

  // setup subscriber
	ros::Subscriber light_sub = n.subscribe("/light_color", sub_light_queue_size, receive_auto_detection);
	ros::Subscriber light_managed_sub = n.subscribe("/light_color_managed", sub_light_queue_size,
							receive_manual_detection);
	ros::Subscriber red_sub = n.subscribe("/red_waypoints_array", sub_waypoint_queue_size, cache_red_lane);
	ros::Subscriber green_sub = n.subscribe("/green_waypoints_array", sub_waypoint_queue_size, cache_green_lane);
	ros::Subscriber config_sub = n.subscribe("/config/lane_stop", sub_config_queue_size, config_parameter);
	
   // setup publisher
	traffic_pub = n.advertise<autoware_msgs::LaneArray>("/traffic_waypoints_array", pub_waypoint_queue_size,
								pub_waypoint_latch);
	该节点根据交通灯，对lane进行选择。输入为light_color以及red_waypoints_array、green_waypoints_array，输出选择的traffic_waypoints_array
  