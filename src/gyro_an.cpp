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
    //    m_buf.lock( );
    //    imu_buf.push( imu_msg );
    //    m_buf.unlock( );
    double time = imu_msg->header.stamp.toSec( );
    gyro_x->push( imu_msg->angular_velocity.x, time );
    gyro_y->push( imu_msg->angular_velocity.y, time );
    gyro_z->push( imu_msg->angular_velocity.z, time );
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "gyro_test" );
    ros::NodeHandle n( "~" );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug );

    std::string IMU_TOPIC = "/djiros/imu"; //= ros_utils::readParam< std::string >( n, "imu_topic" );
    std::string ALLAN_TOPIC;               //= ros_utils::readParam< std::string >( n, "allan_topic" );

    //    IMU_TOPIC   = ros_utils::readParam< std::string >( n, "imu_topic" );
    //    ALLAN_TOPIC = ros_utils::readParam< std::string >( n, "allan_topic" );

    ros::Subscriber sub_imu = n.subscribe( IMU_TOPIC, //
                                           20000000,
                                           imu_callback,
                                           ros::TransportHints( ).tcpNoDelay( ) );

    gyro_x = new imu::Allan( );
    gyro_y = new imu::Allan( );
    gyro_z = new imu::Allan( );

    std::cout << "wait for imu data." << std::endl;

    ros::spin( );

    std::vector< double > v_x = gyro_x->calc( );
    std::vector< double > v_y = gyro_y->calc( );
    std::vector< double > v_z = gyro_z->calc( );
    for ( int index = 0; index < v_x.size( ); ++index )
    {
        std::cout << "x " << index << " " << v_x[index] << std::endl;
        std::cout << "y " << index << " " << v_y[index] << std::endl;
        std::cout << "z " << index << " " << v_z[index] << std::endl;
    }

    return 0;
}
