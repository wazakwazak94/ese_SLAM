#include <ese_slam/IMU_DR.h>

void imuCallback (const msgs_Imu::ConstPtr& imuMsgs)
{
    //ROS_INFO("CallBack!");
    original_accel[0] = imuMsgs->linear_acceleration.x;
    original_accel[1] = imuMsgs->linear_acceleration.y;
    original_accel[2] = imuMsgs->linear_acceleration.z;

    original_ang_vel[0] = imuMsgs->angular_velocity.x;
    original_ang_vel[1] = imuMsgs->angular_velocity.y;
    original_ang_vel[2] = imuMsgs->angular_velocity.z;

    IMUrotation();

    return ;
}

void publishOdom (ros::Publisher imu_odom)
{
    msgs_Point imuOdom;
    pcl::toROSMsg (*IMU_odom, imuOdom);
    imuOdom.header.frame_id = "base_link";
    imuOdom.header.stamp = ros::Time::now();

    imu_odom.publish(imuOdom);
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "imu_dr");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    std::string imu_msg;
    float imu_roll, imu_pitch, imu_yaw;

    ph.param<std::string>("imu_msg",imu_msg,"os1_node/imu");
    ph.param("imu_roll",imu_roll, 0.0f);
    ph.param("imu_pitch",imu_pitch, 0.0f);
    ph.param("imu_yaw",imu_yaw, 0.0f);
    
    ros::Publisher imu_odom = nh.advertise<msgs_Point>("odom",50);
    ros::Subscriber imu_sub = nh.subscribe(imu_msg,1000,imuCallback);

    //initial time 
    imu_current_time = ros::Time::now();
    imu_last_time = ros::Time::now();

    initRotationMatrix(imu_roll, imu_pitch, imu_yaw);
    ROS_INFO ("Matrix Initilize!!");

    ros::Rate rate(10);
    while(ros::ok())
    {
        //ros::spinOnce();

        TimeUpdate();
        estimateOdomFromImu();

        publishOdom(imu_odom);

        ros::spinOnce();
        rate.sleep();
    }
    
}