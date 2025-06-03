int main(int argc, char **argv)
{
    ros::init(argc, argv, "visual_throw");

    ros::NodeHandle nh;
	 
	//订阅状态话题
	ros::Subscriber state_sub     = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
		
	//订阅实时位置信息
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    
	//发布位置控制话题
	ros::Publisher  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
		
	//发布多维控制话题
    ros::Publisher  mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);   
    ros::Publisher  mavros_setpoint_att_pub = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
		               
	//请求解锁服务        
	ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
		
	//请求设置飞行模式，本代码请求进入GUIDED
	ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

	//请求控制舵机客户端
    ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    //循环频率
    ros::Rate rate(20.0); 
   
    //等待连接到rover
    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

	mavros_setpoint_att_pub.publish(setpoint_att_raw);
 
    for(int i = 100; ros::ok() && i > 0; --i)
    {
		mavros_setpoint_att_pub.publish(setpoint_att_raw);
        ros::spinOnce();
        rate.sleep();
    }

    // 请求设置模式为GUIDED
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";
 
    // 请求解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
   
    // 记录上一次请求的时间
    ros::Time last_request = ros::Time::now();
    int flag=0;
    // 请求进入guided模式并且解锁无人船     
    while(ros::ok())
    {
    	// 如果当前模式不是GUIDED且距离上次请求已超过1秒，则请求进入GUIDED模式
        if( current_state.mode != "GUIDED" && (ros::Time::now() - last_request > ros::Duration(1.0)))
        {
            // 调用set_mode服务切换模式
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("GUIDED enabled"); // 模式切换成功
            }
            else
            {
               ROS_INFO("GUIDED failed"); // 模式切换失败
            }
            
            last_request = ros::Time::now(); // 更新时间戳
       	}
        else 
		{
			// 如果未解锁且距离上次请求已超过1秒，则请求解锁
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(1.0)))
			{
		        // 调用arming服务解锁
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed"); // 解锁成功
				//break; // 可选：解锁后跳出循环
		        }
                else
                {
                    ROS_INFO("arm failed"); // 解锁失败
                }
		        	last_request = ros::Time::now(); // 更新时间戳
			}
		}
                  if(ros::Time::now() - last_request > ros::Duration(10.0))
        {
            flag++;
            last_request = ros::Time::now();
        }
        //位置控制
        if(flag==0)
        {
	    setpoint_pos_raw.type_mask = /*1 + 2 + 4 + */8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
		setpoint_pos_raw.coordinate_frame = 1;
		setpoint_pos_raw.position.x = 10;
		setpoint_pos_raw.position.y =10;
        mavros_setpoint_pos_pub.publish(setpoint_pos_raw);
        ROS_INFO("pos localned");
        }
        //偏航角度控制
        if(flag==1)
        {
	    setpoint_pos_raw.type_mask =1 + 2 + 4 + 8 + 16 + 32 + 64 + 128 + 256 + /*512 + 1024 + */2048;
		setpoint_pos_raw.coordinate_frame = 1;
		setpoint_pos_raw.yaw=3.14;
        mavros_setpoint_pos_pub.publish(setpoint_pos_raw);
        ROS_INFO("yaw localned");
        }
        //速度控制（本地坐标系）
        if(flag==2)
        {
	    setpoint_pos_raw.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
		setpoint_pos_raw.coordinate_frame = 1;
		setpoint_pos_raw.velocity.x = 1;
		setpoint_pos_raw.velocity.y = 1;
        mavros_setpoint_pos_pub.publish(setpoint_pos_raw);
        ROS_INFO("velocity localned");
        }
        //速度控制（机体坐标系）
        if(flag==3)
        {
	    setpoint_pos_raw.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024 + 2048;
		setpoint_pos_raw.coordinate_frame = 8;
		setpoint_pos_raw.velocity.x = 1;
		setpoint_pos_raw.velocity.y = 0;
        mavros_setpoint_pos_pub.publish(setpoint_pos_raw);
        ROS_INFO("velocity bodyned");
        }
        //机体速度和偏航角速率
        if(flag==4)
        {
	    setpoint_pos_raw.type_mask = 1 + 2 + 4 + /*8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024/* + 2048*/;
		setpoint_pos_raw.coordinate_frame = 8;
		setpoint_pos_raw.velocity.x = 1;
		setpoint_pos_raw.velocity.y = 0;
        setpoint_pos_raw.yaw_rate=1;
        mavros_setpoint_pos_pub.publish(setpoint_pos_raw);
        ROS_INFO("velocity bodyned and yawrate");
        }
        //推力和偏航角速度
        if(flag==5)
        {
            setpoint_att_raw.type_mask =163;
	        setpoint_att_raw.body_rate.z = 0.5;
	        setpoint_att_raw.thrust      = 1.0;
		    mavros_setpoint_att_pub.publish(setpoint_att_raw);
            ROS_INFO("thr and yawrate");
        }
        //推力和姿态
        if(flag==6)
        {
            setpoint_att_raw.type_mask =39;
	        setpoint_att_raw.orientation.x = 1.0;
            setpoint_att_raw.orientation.y = 0;
            setpoint_att_raw.orientation.z = 0;
            setpoint_att_raw.orientation.w = 0;
	        setpoint_att_raw.thrust      = 1.0;
		    mavros_setpoint_att_pub.publish(setpoint_att_raw);
            ROS_INFO("thr and att");
        }

        ros::spinOnce();
        rate.sleep();
    }   
    return 0;
}
