#include "fncs_header.hpp"

void UAV_control_class::spinOnce(){
	if(!mission_flag){
	    if(!pause_toggle){
		pause_target.position = pix_poseLocal.pose.pose.position;
		pause_target.yaw = UAV_control_class::Quat2Angle(current_orientation).z;
	    }

	    if(UAV_control_mode == emergency_mode){
		target_position.type_mask = velocity_control_type;
		target_position.yaw_rate = 0;
		target_position.velocity.x = 0;
		target_position.velocity.y = 0;
		target_position.velocity.z = -landing_vel;
		setpoint_raw_pub.publish(target_position);
	    }
	    else {
		target_position.velocity.x = 0;
		target_position.velocity.y = 0;
		target_position.velocity.z = 0;
		target_position.yaw_rate = 0;
		target_position.position.x = 0;
		target_position.position.y = 0;
		target_position.position.z = 0;
		target_position.yaw = 0;
		switch (UAV_control_mode) {
		case take_off_mode:
		{
		    target_position.type_mask = position_control_type;
		    target_position.position.x = pix_poseLocal.pose.pose.position.x;
		    target_position.position.y = pix_poseLocal.pose.pose.position.y;
		    target_position.position.z = take_off_height;
		    target_position.yaw = Quat2Angle(current_orientation).z;

		    mavros_msgs::CommandBool cmd;
		    cmd.request.value = true;

		    setpoint_raw_pub.publish(target_position);

		    if(armed_flag == false){

		        client_cmdArming.call(cmd);

		        // log
		        loglog = "Not armed";
		        log_pub_ros_info(LOG_ERROR,loglog);
		    }

		    if(offboard_flag){
		        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0))){

		            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
		                // log
		                loglog ="Offboard gained";
		                log_pub_ros_info(LOG_ERROR,loglog);
		            }
		            last_request = ros::Time::now();
		        }
		        if(abs(pix_poseLocal.pose.pose.position.z - take_off_height) < threshold_distance){
		            offboard_flag = true;
		            wp_idx = -1;
		            UAV_control_mode = hover_wp_mode;
		        }
		    }
		}
		break;
		case landing_mode:
		{
		    target_position.type_mask = velocity_control_type;
		    target_position.velocity.z = (-1)*landing_vel;
		    if(ros::Time::now() - landing_time > ros::Duration(4.0)){
		        landing_old = landing_cur; // Change old height value
		        landing_cur = pix_poseLocal.pose.pose.position.z; // Change current height value
		        landing_time = ros::Time::now();
		        printf("old : %lf // cur : %lf // delta : %lf \n",landing_old, landing_cur, landing_cur - landing_old);

		        // landing clear condition
		        // old height value > cur height value and the difference is less than 50cm
		        if((landing_old > landing_cur) && abs(landing_old - landing_cur) < 0.5){
		            target_position.type_mask = position_control_type;
		            target_position.velocity.z = 0;
		            target_position.position.x = pix_poseLocal.pose.pose.position.x;
		            target_position.position.y = pix_poseLocal.pose.pose.position.y;
		            target_position.position.z = 0;

		            mavros_msgs::CommandBool cmd;
		            cmd.request.value = false;
		            client_cmdArming.call(cmmission_flagd);

		            if(!armed_flag){
		                // log
		                loglog = "Landing clear";
		                log_pub_ros_info(LOG_ERROR,loglog);

		                UAV_control_mode = not_ready_mode;
		                wp_idx = -1;
		                wp_size = 0;
		                landing_old = 0;
		            }
		            else {
		                // log
		                loglog = "Landing on going";
		                log_pub_ros_info(LOG_ERROR,loglog);
		            }
		        }
		    }
		    setpoint_raw_pub.publish(target_position);
		}
		break;
		case move_wp_mode:
		{
		  loglog = "waypoint mode!";
		  log_pub_ros_info(LOG_NORMAL, loglog);
		  switch (test_mode) {
		  case 0:
		  {
		    LOS_move();
		  }break;
		  case 1:
		  {
	//            move_wo_LoS();
		  }break;
		  case 2:
		  {
	//            move_vel_wo_cond();
		  }break;mission_flag
		  case 3:
		  {
	//            move_pos_wo_cond();
		  }break;}
		  setpoint_raw_pub.publish(target_position);
		}
		break;
		case hover_wp_mode:
		{
		  integrated_distance_calculator();

		  target_position.type_mask = position_control_type;
		  if(wp_idx >= 0 && wp_idx < wp_size) {
		    if ((xy_dist < arrive_distance) && (abs(leftover.z) < arrive_distance) && (abs(yaw_dist) < threshold_yaw))
		    {
		      target_position.position = uav_wp_local[wp_idx];
		      target_position.yaw = uav_heading[wp_idx];

		      if (auto_flag) {
		        if (!auto_arrive_cnt) {
		          arrive_time = ros::Time::now();
		          auto_arrive_cnt = true;
		        }

		        if ((ros::Time::now() - arrive_time > ros::Duration(auto_arrive_delay))) {
		          wp_idx++;
		          if(wp_idx == wp_size -1){
		            auto_flag = false;
		            loglog = "Automatic flight end!";
		            log_pub_ros_info(LOG_NORMAL,loglog);
		          }
		          else{
		            UAV_control_mode = move_wp_mode;
		            arrive_time = ros::Time::now();

		            // log
		            loglog = "Automatically going waypoint "+std::to_string(wp_idx)+"!";
		            log_pub_ros_info(LOG_NORMAL,loglog);
		            auto_arrive_cnt = false;
		          }
		        }
		        else{
		          loglog = "hovering mode!";
		          log_pub_ros_info(LOG_NORMAL, loglog);
		        }
		      }
		      else {
		        // log
		        loglog = "Waypoint arrive "+std::to_string(wp_idx)+"!";
		        log_pub_ros_info(LOG_IMPORTANT,loglog);
		      }
		    }
		    else{
		      target_position.type_mask = yaw_align_control_type;
		      target_position.position = uav_wp_local[wp_idx];
		      target_position.yaw_rate = yaw_dist > 0 ? yaw_vel : (-1)*yaw_vel;
		      ROS_INFO("yaw_rate : %lf",target_position.yaw_rate);
		      ROS_INFO("yaw_dist : %lf",yaw_dist);
		      ROS_INFO("curr_yaw : %lf",Quat2Angle(current_orientation).z);
		      ROS_INFO("target_yaw : %lf",uav_heading[wp_idx]);

		      auto_arrive_cnt = false;

		      //log
		      loglog = "Yaw aligning!";
		      log_pub_ros_info(LOG_NORMAL,loglog);
		    }
		  }
		  else {
		    target_position.type_mask = position_control_type;
		    target_position.position.x = 0;
		    target_position.position.y = 0;
		    target_position.position.z = take_off_height;
		    target_position.yaw = UAV_control_class::Quat2Angle(current_orientation).z;

		    loglog = "Take off clear!";
		    log_pub_ros_info(LOG_NORMAL,loglog);
		  }
		  setpoint_raw_pub.publish(target_position);
		}
		break;

		case vel_hover_wo_cond_mode:
		{
		  integrated_distance_calculator();

		  target_position.type_mask = velocity_control_type;
		  if(wp_idx >= 0 && wp_idx < wp_size) {
		    if ((xy_dist < arrive_distance) && (abs(leftover.z) < arrive_distance) && (abs(yaw_dist) < threshold_yaw))
		    {
	//              target_position.velocity.x = 0;
	//              target_position.velocity.y = 0;
	//              target_position.velocity.z = 0;
	//              target_position.yaw_rate = 0;
		      target_position.velocity.x = leftover.x*wp_vel/xyz_dist;
		      target_position.velocity.y = leftover.y*wp_vel/xyz_dist;
		      target_position.velocity.z = leftover.z*wp_vel/xyz_dist;
		      target_position.yaw_rate = yaw_dist > 0 ? yaw_vel : (-1)*yaw_vel;

		      if (auto_flag) {
		        if (!auto_arrive_cnt) {
		          arrive_time = ros::Time::now();
		          auto_arrive_cnt = true;
		        }

		        if ((ros::Time::now() - arrive_time > ros::Duration(auto_arrive_delay))) {
		          wp_idx++;
		          if(wp_idx == wp_size -1){
		            auto_flag = false;
		            loglog = "Automatic flight end!";
		            log_pub_ros_info(LOG_NORMAL,loglog);
		          }


		          UAV_control_mode = move_wp_mode;
		          arrive_time = ros::Time::now();

		          // log
		          loglog = "Automatically going waypoint "+std::to_string(wp_idx)+"!";
		          log_pub_ros_info(LOG_NORMAL,loglog);
		          auto_arrive_cnt = false;
		        }
		        else{
		          loglog = "hovering mode!";
		          log_pub_ros_info(LOG_NORMAL, loglog);
		        }
		      }
		      else {
		        if ((ros::Time::now() - arrive_time > ros::Duration(auto_arrive_delay))) {
		          if(wp_idx == wp_size -1){
		            // log
		            loglog = "Waypoint arrive "+std::to_string(wp_idx)+"!";
		            log_pub_ros_info(LOG_IMPORTANT,loglog);
		            loglog = "Flight end!";
		            log_pub_ros_info(LOG_NORMAL,loglog);
		          }
		          else {
		            // log
		            loglog = "Waypoint arrive "+std::to_string(wp_idx)+"!";
		            log_pub_ros_info(LOG_IMPORTANT,loglog);
		          }
		        }
		        else{
		          loglog = "hovering mode!";
		          log_pub_ros_info(LOG_NORMAL, loglog);
		        }
		      }
		    }
		    else{
		      target_position.type_mask = velocity_control_type;
		      target_position.velocity.x = leftover.x*wp_vel/xyz_dist;
		      target_position.velocity.y = leftover.y*wp_vel/xyz_dist;
		      target_position.velocity.z = leftovemission_flagr.z*wp_vel/xyz_dist;
		      target_position.yaw_rate = yaw_dist > 0 ? yaw_vel : (-1)*yaw_vel;
		      ROS_INFO("yaw_rate : %lf",target_position.yaw_rate);
		      ROS_INFO("yaw_dist : %lf",yaw_dist);
		      ROS_INFO("curr_yaw : %lf",Quat2Angle(current_orientation).z);
		      ROS_INFO("target_yaw : %lf",uav_heading[wp_idx]);

		      auto_arrive_cnt = false;

		      //log
		      loglog = "Yaw aligning!";
		      log_pub_ros_info(LOG_NORMAL,loglog);
		    }
		  }
		  else {
		    target_position.type_mask = position_control_type;
		    target_position.position.x = 0;
		    target_position.position.y = 0;
		    target_position.position.z = take_off_height;
		    target_position.yaw = UAV_control_class::Quat2Angle(current_orientation).z;

		    loglog = "Take off clear!";
		    log_pub_ros_info(LOG_NORMAL,loglog);
		  }
		  setpoint_raw_pub.publish(target_position);
		}
		break;

		case pos_hover_wo_cond_mode:
		{
		  integrated_distance_calculator();

		  target_position.type_mask = position_control_type;
		  if(wp_idx >= 0 && wp_idx < wp_size) {
		    if ((xy_dist < arrive_distance) && (abs(leftover.z) < arrive_distance) && (abs(yaw_dist) < threshold_yaw))
		    {
		      target_position.position = uav_wp_local[wp_idx];
		      target_position.yaw = uav_heading[wp_idx];

		      if (auto_flag) {
		        if (!auto_arrive_cnt) {
		          arrive_time = ros::Time::now();
		          auto_arrive_cnt = true;
		        }

		        if ((ros::Time::now() - arrive_time > ros::Duration(auto_arrive_delay))) {
		          wp_idx++;
		          if(wp_idx == wp_size -1){
		            auto_flag = false;
		            loglog = "Automatic flight end!";
		            log_pub_ros_info(LOG_NORMAL,loglog);
		          }
		          else{
		            UAV_control_mode = move_wp_mode;

		            // log
		            loglog = "Automatically going waypoint "+std::to_string(wp_idx)+"!";
		            log_pub_ros_info(LOG_NORMAL,loglog);
		            auto_arrive_cnt = false;
		          }
		        }
		        else{
		          loglog = "hovering mode!";
		          log_pub_ros_info(LOG_NORMAL, loglog);
		        }
		      }
		      else {
		        if (!auto_arrive_cnt) {
		          arrive_time = ros::Time::now();
		          auto_arrive_cnt = true;
		        }
		        if ((ros::Time::now() - arrive_time > ros::Duration(auto_arrive_delay))) {
		          if(wp_idx == wp_size -1){
		            loglog = "Waypoint flight end!";
		            log_pub_ros_info(LOG_NORMAL,loglog);
		          }
		          else{
		            // log
		            loglog = "Waypoint arrive "+std::to_string(wp_idx)+"!";
		            log_pub_ros_info(LOG_NORMAL,loglog);
		            auto_arrive_cnt = false;
		          }
		        }
		        else{
		          loglog = "hovering mode!";
		          log_pub_ros_info(LOG_NORMAL, loglog);}
		      }
		    }
		    else{
		      target_position.type_mask = position_control_type;
		      target_position.position = uav_wp_local[wp_idx];
		      target_position.yaw = uav_heading[wp_idx];
		      ROS_INFO("yaw_rate : %lf",target_position.yaw_rate);
		      ROS_INFO("yaw_dist : %lf",yaw_dist);
		      ROS_INFO("curr_yaw : %lf",Quat2Angle(current_orientation).z);
		      ROS_INFO("target_yaw : %lf",uav_heading[wp_idx]);

		      auto_arrive_cnt = false;

		      //log
		      loglog = "Yaw aligning!";
		      log_pub_ros_info(LOG_NORMAL,loglog);
		    }
		  }
		  else {
		    target_position.type_mask = position_control_type;
		    target_position.position.x = 0;
		    target_position.position.y = 0;
		    target_position.position.z = take_off_height;
		    target_position.yaw = UAV_control_class::Quat2Angle(current_orientation).z;

		    loglog = "Take off clear!";
		    log_pub_ros_info(LOG_NORMAL,loglog);
		  }
		  setpoint_raw_pub.publish(target_position);
		}
		break;

		case pause_mode:
		{
		    if(pause_toggle){
		        target_position.type_mask = position_control_type;
		        target_position.position = pause_target.position;
		        target_position.yaw = pause_target.yaw;
		        setpoint_raw_pub.publish(target_position);

		        loglog = "UAV paused!";
		        log_pub_ros_info(LOG_NORMAL,loglog);
		    }
		    else{
		        UAV_control_mode = move_wp_mode;
		        loglog = "UAV pause disabled!";
		        log_pub_ros_info(LOG_NORMAL,loglog);
		    }
		    setpoint_raw_pub.publish(target_position);
		}
		break;
		case return_home_mode:
		{
		    //target_position.type_mask = position_control_type;
		    //target_position.position.z = take_off_height;
		    //setpoint_raw_pub.publish(target_position);
		    
		    loglog = "return home mode!";
		    log_pub_ros_info(LOG_NORMAL, loglog);
		    LOS_move();
		    setpoint_raw_pub.publish(target_position);
		}
		break;
		}
	    }
	else{

	}
}
//} DUCK
//easy function
geometry_msgs::Vector3 UAV_control_class::Quat2Angle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 res;
    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(res.x, res.y, res.z);
    return res;
}

geometry_msgs::Vector3 UAV_control_class::euclidean_xyz_distance(geometry_msgs::Point start, geometry_msgs::Point end){
    geometry_msgs::Vector3 res;
    res.x = end.x - start.x;
    res.y = end.y - start.y;
    res.z = end.z - start.z;
    return res;
}

float UAV_control_class::yaw_distance(float start, float end){
    float yawyaw = end - start;
    while (yawyaw > 3.141592){
        yawyaw -= 3.141592*2;
    }
    while (yawyaw < -3.141592){
        yawyaw += 3.141592*2;
    }
    return yawyaw;
}
void UAV_control_class::integrated_distance_calculator(){
    leftover = euclidean_xyz_distance(pix_poseLocal.pose.pose.position,uav_wp_local[wp_idx]);
    xy_dist = sqrt(pow(leftover.x,2)+pow(leftover.y,2));
    xyz_dist = sqrt(pow(leftover.x,2)+pow(leftover.y,2)+pow(leftover.z,2));
    yaw_dist = yaw_distance(Quat2Angle(current_orientation).z,uav_heading[wp_idx]);
}

void UAV_control_class::set_start_end_wp(int idx){
    if(idx == 0){
        start_wp.x = 0;
        start_wp.y = 0;
        start_wp.z = take_off_height;
    }
    else{
        start_wp = uav_wp_local[idx-1];
    }
    end_wp = uav_wp_local[idx];

    total_distance = euclidean_xyz_distance(start_wp,end_wp);
    xyz_total_dist = sqrt(pow(total_distance.x,2)+pow(total_distance.y,2)+pow(total_distance.z,2));
}

void UAV_control_class::log_pub_ros_info(int color, std::string log){
    //// log
    /*
     * LOG_ERROR = 0
     * LOG_NORMAL = 1
     * LOG_EMERGENCY = 2
     */
    drone_contest::log_data pix2uav_log_data;
    pix2uav_log_data.string_color = color;
    pix2uav_log_data.log_string = log;

    if(log != loglog_old){
      loglog_old = log;
      pub_log_data.publish(pix2uav_log_data);
    }
}

void UAV_control_class::LOS_move(){
  target_position.type_mask = waypoint_control_type;
  if(UAV_control_mode == return_home_mode){
    total_distance = euclidean_xyz_distance(start_wp,end_wp);
    xyz_total_dist = sqrt(pow(total_distance.x,2)+pow(total_distance.y,2)+pow(total_distance.z,2));
    leftover = euclidean_xyz_distance(pix_poseLocal.pose.pose.position,end_wp);
    xy_dist = sqrt(pow(leftover.x,2)+pow(leftover.y,2));
    xyz_dist = sqrt(pow(leftover.x,2)+pow(leftover.y,2)+pow(leftover.z,2));
    yaw_dist = yaw_distance(Quat2Angle(current_orientation).z,uav_heading[0]);
  }
  else{
    set_start_end_wp(wp_idx);
    integrated_distance_calculator();
  }

  ROS_INFO("xy_dist : %lf",xy_dist);
  ROS_INFO("z axis : %lf",leftover.z);
  ROS_INFO("yaw_dist : %lf",yaw_dist);
  if(xy_dist < threshold_distance && abs(leftover.z) < threshold_distance){
    if(UAV_control_mode == return_home_mode){
      loglog = "landing to home mode!";
      log_pub_ros_info(LOG_NORMAL, loglog);
      UAV_control_mode = landing_mode;
    }
    else{
      loglog = "alignment mode!";
      log_pub_ros_info(LOG_NORMAL, loglog);
      target_position.type_mask = velocity_control_type;
      target_position.velocity.x = 0;
      target_position.velocity.y = 0;
      target_position.velocity.z = 0;
      target_position.yaw_rate = 0;
      UAV_control_mode = hover_wp_mode;
    }
  }
  else{
    float perpendicular_ratio;
    geometry_msgs::Point perpendicular_point;
    perpendicular_ratio= (total_distance.x * (pix_poseLocal.pose.pose.position.x - start_wp.x) + total_distance.y * (pix_poseLocal.pose.pose.position.y - start_wp.y) + total_distance.z * (pix_poseLocal.pose.pose.position.z - start_wp.z))/pow(xyz_total_dist,2);
    perpendicular_point.x = start_wp.x + total_distance.x * perpendicular_ratio;
    perpendicular_point.y = start_wp.y + total_distance.y * perpendicular_ratio;
    perpendicular_point.z = start_wp.z + total_distance.z * perpendicular_ratio;

    geometry_msgs::Vector3 perpendicular_dist;
    float perpendicular_total_dist;
    perpendicular_dist = euclidean_xyz_distance(pix_poseLocal.pose.pose.position,perpendicular_point);
    perpendicular_total_dist = sqrt(pow(perpendicular_dist.x,2)+pow(perpendicular_dist.y,2)+pow(perpendicular_dist.z,2));

    float final_ratio;
    if(perpendicular_total_dist<1 && perpendicular_total_dist>0){
      if(LOS_radius>perpendicular_total_dist) {
        final_ratio = perpendicular_ratio + sqrt(pow(LOS_radius,2)-pow(perpendicular_total_dist,2))/xyz_total_dist;
      }
      else {
        final_ratio = perpendicular_ratio;
      }
    }
    else{
      final_ratio = 1;
    }
    final_ratio = 1 + (final_ratio - 1)*((final_ratio - 1) < 0)-final_ratio*(final_ratio<0);

    geometry_msgs::Point local_destination;

    local_destination.x = start_wp.x + final_ratio*total_distance.x;
    local_destination.y = start_wp.y + final_ratio*total_distance.y;
    local_destination.z = start_wp.z + final_ratio*total_distance.z;

    geometry_msgs::Vector3 local_diff;
    local_diff = euclidean_xyz_distance(pix_poseLocal.pose.pose.position,local_destination);
    float local_dist = sqrt(pow(local_diff.x,2)+pow(local_diff.y,2)+pow(local_diff.z,2));

    target_position.velocity.x = local_diff.x*wp_vel/local_dist;
    target_position.velocity.y = local_diff.y*wp_vel/local_dist;
    target_position.velocity.z = local_diff.z*wp_vel/local_dist;
    if(wp_idx>0) target_position.yaw = uav_heading[wp_idx-1];
    else UAV_control_class::Quat2Angle(current_orientation).z;
    ROS_INFO("local diff x : %lf",local_diff.x);
    ROS_INFO("local diff y : %lf",local_diff.y);
    ROS_INFO("local diff z : %lf",local_diff.z);
  }
}

void UAV_control_class::move_wo_LoS(){
  //DUCK
  target_position.type_mask = waypoint_control_type;
  set_start_end_wp(wp_idx);
  integrated_distance_calculator();
  ROS_INFO("xy_dist : %lf",xy_dist);
  ROS_INFO("z axis : %lf",leftover.z);
  ROS_INFO("yaw_dist : %lf",yaw_dist);
  if(xy_dist < threshold_distance && abs(leftover.z) < threshold_distance){
    loglog = "alignment mode!";
    log_pub_ros_info(LOG_NORMAL, loglog);
    target_position.type_mask = velocity_control_type;
    target_position.velocity.x = 0;
    target_position.velocity.y = 0;
    target_position.velocity.z = 0;
    target_position.yaw_rate = 0;
    UAV_control_mode = hover_wp_mode;
  }
  else{
    geometry_msgs::Point local_destination;

    local_destination.x = start_wp.x + total_distance.x;
    local_destination.y = start_wp.y + total_distance.y;
    local_destination.z = start_wp.z + total_distance.z;

    geometry_msgs::Vector3 local_diff;
    local_diff = euclidean_xyz_distance(pix_poseLocal.pose.pose.position,local_destination);
    float local_dist = sqrt(pow(local_diff.x,2)+pow(local_diff.y,2)+pow(local_diff.z,2));

    target_position.velocity.x = local_diff.x*wp_vel/local_dist;
    target_position.velocity.y = local_diff.y*wp_vel/local_dist;
    target_position.velocity.z = local_diff.z*wp_vel/local_dist;
    if(wp_idx>0) target_position.yaw = uav_heading[wp_idx-1];
    else UAV_control_class::Quat2Angle(current_orientation).z;
    ROS_INFO("local diff x : %lf",local_diff.x);
    ROS_INFO("local diff y : %lf",local_diff.y);
    ROS_INFO("local diff z : %lf",local_diff.z);
  }
}


void UAV_control_class::move_vel_wo_cond(){
  target_position.type_mask = waypoint_control_type;
  set_start_end_wp(wp_idx);
  integrated_distance_calculator();
  ROS_INFO("xy_dist : %lf",xy_dist);
  ROS_INFO("z axis : %lf",leftover.z);
  ROS_INFO("yaw_dist : %lf",yaw_dist);
  target_position.type_mask = velocity_control_type;
  target_position.velocity.x = leftover.x*wp_vel/xyz_dist;
  target_position.velocity.y = leftover.y*wp_vel/xyz_dist;
  target_position.velocity.z = leftover.z*wp_vel/xyz_dist;
  target_position.yaw_rate = yaw_dist > 0 ? yaw_vel : (-1)*yaw_vel;
  if(xy_dist < threshold_distance && abs(leftover.z) < threshold_distance){
    loglog = "alignment mode!";
    log_pub_ros_info(LOG_NORMAL, loglog);
    UAV_control_mode = vel_hover_wo_cond_mode;
  }
}

void UAV_control_class::move_pos_wo_cond(){
  target_position.type_mask = waypoint_control_type;
  set_start_end_wp(wp_idx);
  integrated_distance_calculator();
  ROS_INFO("xy_dist : %lf",xy_dist);
  ROS_INFO("z axis : %lf",leftover.z);
  ROS_INFO("yaw_dist : %lf",yaw_dist);
  target_position.type_mask = position_control_type;
  target_position.position = uav_wp_local[wp_idx];
  target_position.yaw = uav_heading[wp_idx];
  if(xy_dist < threshold_distance && abs(leftover.z) < threshold_distance){
    loglog = "alignment mode!";
    log_pub_ros_info(LOG_NORMAL, loglog);
    UAV_control_mode = pos_hover_wo_cond_mode;
  }
}
