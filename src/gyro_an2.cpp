#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward
{
backward::SignalHandling sh;
}

#include "gyro_lib/allan.h"
#include "gyro_lib/allan_acc.h"
#include "gyro_lib/fitallan.h"
#include <code_utils/ros_utils.h>
#include <geometry_msgs/Vector3Stamped.h>
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
imu::AllanAcc* acc_x;
imu::AllanAcc* acc_y;
imu::AllanAcc* acc_z;
double start_t;
bool start = true;
bool end   = false;

void
imu_callback( const sensor_msgs::ImuConstPtr& imu_msg )
{
    //    m_buf.lock( );
    //    imu_buf.push( imu_msg );
    //    m_buf.unlock( );
    double time = imu_msg->header.stamp.toSec( );
    gyro_x->pushRadPerSec( imu_msg->angular_velocity.x, time );
    gyro_y->pushRadPerSec( imu_msg->angular_velocity.y, time );
    gyro_z->pushRadPerSec( imu_msg->angular_velocity.z, time );
    acc_x->push( imu_msg->linear_acceleration.x, time );
    acc_y->push( imu_msg->linear_acceleration.y, time );
    acc_z->push( imu_msg->linear_acceleration.z, time );

    if ( start )
    {
        start_t = time;
        start   = false;
    }
    else
    {
        double time_min = ( time - start_t ) / 60;
        if ( time_min > 30 )
            end = true;
    }
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "gyro_test" );
    ros::NodeHandle n( "~" );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug );

    std::string IMU_TOPIC;   //= ros_utils::readParam< std::string >( n, "imu_topic" );
    std::string ALLAN_TOPIC; //= ros_utils::readParam< std::string >( n, "allan_topic" );

    //    IMU_TOPIC   = ros_utils::readParam< std::string >( n, "imu_topic" );
    //    ALLAN_TOPIC = ros_utils::readParam< std::string >( n, "allan_topic" );
    IMU_TOPIC   = "/djiros/imu";
    ALLAN_TOPIC = "gyro_x";

    ros::Subscriber sub_imu = n.subscribe( IMU_TOPIC, //
                                           20000000,
                                           imu_callback,
                                           ros::TransportHints( ).tcpNoDelay( ) );
    ros::Publisher pub = n.advertise< geometry_msgs::Vector3Stamped >( ALLAN_TOPIC, 2000 );

    gyro_x = new imu::Allan( "gyro x", 100 );
    gyro_y = new imu::Allan( "gyro y", 1000 );
    gyro_z = new imu::Allan( "gyro z", 1000 );
    acc_x  = new imu::AllanAcc( "acc x", 1000 );
    acc_y  = new imu::AllanAcc( "acc y", 1000 );
    acc_z  = new imu::AllanAcc( "acc z", 1000 );
    std::cout << "wait for imu data." << std::endl;
    ros::Rate loop( 100 );

    //    ros::spin( );
    while ( !end )
    {
        loop.sleep( );
        ros::spinOnce( );
    }

    gyro_x->calc( );
    std::vector< double > gyro_v_x  = gyro_x->getVariance( );
    std::vector< double > gyro_d_x  = gyro_x->getDeviation( );
    std::vector< double > gyro_ts_x = gyro_x->getTimes( );

    gyro_y->calc( );
    std::vector< double > gyro_v_y  = gyro_y->getVariance( );
    std::vector< double > gyro_d_y  = gyro_y->getDeviation( );
    std::vector< double > gyro_ts_y = gyro_y->getTimes( );

    gyro_z->calc( );
    std::vector< double > gyro_v_z  = gyro_z->getVariance( );
    std::vector< double > gyro_d_z  = gyro_z->getDeviation( );
    std::vector< double > gyro_ts_z = gyro_z->getTimes( );

    for ( int index = 0; index < gyro_v_x.size( ); ++index )
    {
        //        loop.sleep( );

        std::cout << "x " << gyro_ts_x[index] << " " << gyro_d_x[index] << " " << gyro_d_y[index]
                  << " " << gyro_d_z[index] << std::endl;

        geometry_msgs::Vector3Stamped v_t;
        v_t.header.frame_id = "body";
        v_t.header.stamp    = ros::Time::now( );
        v_t.vector.x        = gyro_d_x[index];
        pub.publish( v_t );

        //        std::cout << "y " << index << " " << v_y[index] << std::endl;
        //        std::cout << "z " << index << " " << v_z[index] << std::endl;
    }

    FitAllan fit_x( gyro_v_x, gyro_ts_x );
    std::cout << "-------------------" << std::endl;
    std::cout << "Gyro X " << std::endl;
    std::cout << "     Q " << fit_x.getQ( ) << std::endl;
    std::cout << "     N " << fit_x.getN( ) << std::endl;
    std::cout << "     B " << fit_x.getB( ) << std::endl;
    std::cout << "     K " << fit_x.getK( ) << std::endl;
    std::cout << "     R " << fit_x.getR( ) << std::endl;

    //    FitAllan fit_y( gyro_v_y, gyro_ts_y );
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "Gyro y " << std::endl;
    //    std::cout << "     Q " << fit_y.getQ( ) << std::endl;
    //    std::cout << "     N " << fit_y.getN( ) << std::endl;
    //    std::cout << "     B " << fit_y.getB( ) << std::endl;
    //    std::cout << "     K " << fit_y.getK( ) << std::endl;
    //    std::cout << "     R " << fit_y.getR( ) << std::endl;

    //    FitAllan fit_z( gyro_v_z, gyro_ts_z );
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "Gyro z " << std::endl;
    //    std::cout << "     Q " << fit_z.getQ( ) << std::endl;
    //    std::cout << "     N " << fit_z.getN( ) << std::endl;
    //    std::cout << "     B " << fit_z.getB( ) << std::endl;
    //    std::cout << "     K " << fit_z.getK( ) << std::endl;
    //    std::cout << "     R " << fit_z.getR( ) << std::endl;

    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "-------------------" << std::endl;

    //    acc_x->calc( );
    //    std::vector< double > acc_v_x  = acc_x->getVariance( );
    //    std::vector< double > acc_d_x  = acc_x->getDeviation( );
    //    std::vector< double > acc_ts_x = acc_x->getTimes( );

    //    acc_y->calc( );
    //    std::vector< double > acc_v_y  = acc_y->getVariance( );
    //    std::vector< double > acc_d_y  = acc_y->getDeviation( );
    //    std::vector< double > acc_ts_y = acc_y->getTimes( );

    //    acc_z->calc( );
    //    std::vector< double > acc_v_z  = acc_z->getVariance( );
    //    std::vector< double > acc_d_z  = acc_z->getDeviation( );
    //    std::vector< double > acc_ts_z = acc_z->getTimes( );

    //    for ( int index = 0; index < acc_v_x.size( ); ++index )
    //    {
    //        loop.sleep( );

    //        std::cout << "x " << acc_ts_x[index] << " " << acc_d_x[index] << " " <<
    //        acc_d_y[index]
    //                  << " " << acc_d_z[index] << std::endl;

    //        geometry_msgs::Vector3Stamped v_t;
    //        v_t.header.frame_id = "body";
    //        v_t.header.stamp    = ros::Time::now( );
    //        v_t.vector.x        = d_x[index];
    //        pub.publish( v_t );

    //        std::cout << "y " << index << " " << v_y[index] << std::endl;
    //        std::cout << "z " << index << " " << v_z[index] << std::endl;
    //    }

    //    FitAllan fit_x_acc( acc_v_x, acc_ts_x );
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "acc X " << std::endl;
    //    std::cout << "     Q " << fit_x_acc.getQ( ) << std::endl;
    //    std::cout << "     N " << fit_x_acc.getN( ) << std::endl;
    //    std::cout << "     B " << fit_x_acc.getB( ) << std::endl;
    //    std::cout << "     K " << fit_x_acc.getK( ) << std::endl;
    //    std::cout << "     R " << fit_x_acc.getR( ) << std::endl;

    //    FitAllan fit_y_acc( acc_v_y, acc_ts_y );
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "acc y " << std::endl;
    //    std::cout << "     Q " << fit_y_acc.getQ( ) << std::endl;
    //    std::cout << "     N " << fit_y_acc.getN( ) << std::endl;
    //    std::cout << "     B " << fit_y_acc.getB( ) << std::endl;
    //    std::cout << "     K " << fit_y_acc.getK( ) << std::endl;
    //    std::cout << "     R " << fit_y_acc.getR( ) << std::endl;

    //    FitAllan fit_z_acc( acc_v_z, acc_ts_z );
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "acc z " << std::endl;
    //    std::cout << "     Q " << fit_z_acc.getQ( ) << std::endl;
    //    std::cout << "     N " << fit_z_acc.getN( ) << std::endl;
    //    std::cout << "     B " << fit_z_acc.getB( ) << std::endl;
    //    std::cout << "     K " << fit_z_acc.getK( ) << std::endl;
    //    std::cout << "     R " << fit_z_acc.getR( ) << std::endl;

    return 0;
}
