#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include "gyro_lib/allan.h"
#include <code_utils/ros_utils.h>
#include <iostream>
#include <mutex>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

std::mutex m_buf;

std::queue< sensor_msgs::ImuConstPtr > imu_buf;

imu::Allan* gyro_x;
imu::Allan* gyro_y;
imu::Allan* gyro_z;

void
imu_callback( const sensor_msgs::ImuConstPtr& imu_msg )
{
    m_buf.lock( );
    imu_buf.push( imu_msg );
    m_buf.unlock( );
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "gyro_test" );
    ros::NodeHandle n( "~" );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug );

    std::string IMU_TOPIC   = ros_utils::readParam< std::string >( n, "imu_topic" );
    std::string ALLAN_TOPIC = ros_utils::readParam< std::string >( n, "allan_topic" );
    ros::Subscriber sub_imu = n.subscribe( IMU_TOPIC, //
                                           2000,
                                           imu_callback,
                                           ros::TransportHints( ).tcpNoDelay( ) );

    gyro_x = new imu::Allan( );
    gyro_y = new imu::Allan( );
    gyro_z = new imu::Allan( );

    ros::spin( );

    return 0;
}
