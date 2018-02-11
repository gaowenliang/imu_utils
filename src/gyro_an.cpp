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
bool start       = true;
bool end         = false;
int max_time_min = 10;
std::string data_save_path;

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
        if ( time_min > max_time_min )
            end = true;
    }
}

void
writeData1( const std::string sensor_name, //
            const std::vector< double >& gyro_ts_x,
            const std::vector< double >& gyro_d )
{
    std::ofstream out_t;
    std::ofstream out_x;
    out_t.open( data_save_path + "data_" + sensor_name + "_t.txt", std::ios::trunc );
    out_x.open( data_save_path + "data_" + sensor_name + "_x.txt", std::ios::trunc );
    out_t << std::setprecision( 10 );
    out_x << std::setprecision( 10 );
    for ( int index = 0; index < gyro_ts_x.size( ); ++index )
    {
        out_t << gyro_ts_x[index] << '\n';
        out_x << gyro_d[index] << '\n';
    }
    out_t.close( );
    out_x.close( );
}

void
writeData3( const std::string sensor_name,
            const std::vector< double >& gyro_ts_x,
            const std::vector< double >& gyro_d_x,
            const std::vector< double >& gyro_d_y,
            const std::vector< double >& gyro_d_z )
{
    std::ofstream out_t;
    std::ofstream out_x;
    std::ofstream out_y;
    std::ofstream out_z;
    out_t.open( data_save_path + "data_" + sensor_name + "_t.txt", std::ios::trunc );
    out_x.open( data_save_path + "data_" + sensor_name + "_x.txt", std::ios::trunc );
    out_y.open( data_save_path + "data_" + sensor_name + "_y.txt", std::ios::trunc );
    out_z.open( data_save_path + "data_" + sensor_name + "_z.txt", std::ios::trunc );
    out_t << std::setprecision( 10 );
    out_x << std::setprecision( 10 );
    out_y << std::setprecision( 10 );
    out_z << std::setprecision( 10 );
    for ( int index = 0; index < gyro_ts_x.size( ); ++index )
    {
        out_t << gyro_ts_x[index] << '\n';
        out_x << gyro_d_x[index] << '\n';
        out_y << gyro_d_y[index] << '\n';
        out_z << gyro_d_z[index] << '\n';
    }
    out_t.close( );
    out_x.close( );
    out_y.close( );
    out_z.close( );
}

int
main( int argc, char** argv )
{
    ros::init( argc, argv, "gyro_test" );
    ros::NodeHandle n( "~" );
    ros::console::set_logger_level( ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug );

    std::string IMU_TOPIC;
    std::string IMU_NAME;
    int max_cluster;

    IMU_TOPIC      = ros_utils::readParam< std::string >( n, "imu_topic" );
    IMU_NAME       = ros_utils::readParam< std::string >( n, "imu_name" );
    data_save_path = ros_utils::readParam< std::string >( n, "data_save_path" );
    max_time_min   = ros_utils::readParam< int >( n, "max_time_min" );
    max_cluster    = ros_utils::readParam< int >( n, "max_cluster" );

    ros::Subscriber sub_imu = n.subscribe( IMU_TOPIC, //
                                           20000000,
                                           imu_callback,
                                           ros::TransportHints( ).tcpNoDelay( ) );
    //    ros::Publisher pub = n.advertise< geometry_msgs::Vector3Stamped >( ALLAN_TOPIC, 2000 );

    gyro_x = new imu::Allan( "gyro x", max_cluster );
    gyro_y = new imu::Allan( "gyro y", max_cluster );
    gyro_z = new imu::Allan( "gyro z", max_cluster );
    acc_x  = new imu::AllanAcc( "acc x", max_cluster );
    acc_y  = new imu::AllanAcc( "acc y", max_cluster );
    acc_z  = new imu::AllanAcc( "acc z", max_cluster );
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

    FitAllan fit_x( gyro_v_x, gyro_ts_x );
    std::cout << "-------------------" << std::endl;
    std::cout << "Gyro X " << std::endl;
    std::cout << "     Q " << fit_x.getQ( ) << std::endl;
    std::cout << "     N " << fit_x.getN( ) << std::endl;
    std::cout << "     B " << fit_x.getB( ) << std::endl;
    std::cout << "     K " << fit_x.getK( ) << std::endl;
    std::cout << "     R " << fit_x.getR( ) << std::endl;
    std::cout << "  bias " << gyro_x->getAvgValue( ) / 3600 << " degree/s" << std::endl;

    FitAllan fit_y( gyro_v_y, gyro_ts_y );
    std::cout << "-------------------" << std::endl;
    std::cout << "Gyro y " << std::endl;
    std::cout << "     Q " << fit_y.getQ( ) << std::endl;
    std::cout << "     N " << fit_y.getN( ) << std::endl;
    std::cout << "     B " << fit_y.getB( ) << std::endl;
    std::cout << "     K " << fit_y.getK( ) << std::endl;
    std::cout << "     R " << fit_y.getR( ) << std::endl;
    std::cout << "  bias " << gyro_y->getAvgValue( ) / 3600 << " degree/s" << std::endl;

    FitAllan fit_z( gyro_v_z, gyro_ts_z );
    std::cout << "-------------------" << std::endl;
    std::cout << "Gyro z " << std::endl;
    std::cout << "     Q " << fit_z.getQ( ) << std::endl;
    std::cout << "     N " << fit_z.getN( ) << std::endl;
    std::cout << "     B " << fit_z.getB( ) << std::endl;
    std::cout << "     K " << fit_z.getK( ) << std::endl;
    std::cout << "     R " << fit_z.getR( ) << std::endl;
    std::cout << "  bias " << gyro_z->getAvgValue( ) / 3600 << " degree/s" << std::endl;

    std::vector< double > gyro_sim_d_x = fit_x.calcSimDeviation( gyro_ts_x );
    std::vector< double > gyro_sim_d_y = fit_y.calcSimDeviation( gyro_ts_y );
    std::vector< double > gyro_sim_d_z = fit_z.calcSimDeviation( gyro_ts_z );

    writeData3( IMU_NAME + "_sim", gyro_ts_x, gyro_sim_d_x, gyro_sim_d_y, gyro_sim_d_z );
    writeData3( IMU_NAME, gyro_ts_x, gyro_d_x, gyro_d_y, gyro_d_z );
    //    for ( int index = 0; index < gyro_v_x.size( ); ++index )
    //    {
    //        std::cout << gyro_ts_x[index] << " " << gyro_d_y[index] << " " <<
    //        gyro_sim_d_y[index] << std::endl;

    //        std::cout << gyro_ts_x[index] << " " << gyro_d_x[index] << " " <<
    //        gyro_d_y[index] << " "
    //                  << gyro_d_z[index] << std::endl;

    //        std::cout << "y " << index << " " << v_y[index] << std::endl;
    //        std::cout << "z " << index << " " << v_z[index] << std::endl;
    //    }

    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "-------------------" << std::endl;
    //    std::cout << "-------------------" << std::endl;

    acc_x->calc( );
    std::vector< double > acc_v_x  = acc_x->getVariance( );
    std::vector< double > acc_d_x  = acc_x->getDeviation( );
    std::vector< double > acc_ts_x = acc_x->getTimes( );

    acc_y->calc( );
    std::vector< double > acc_v_y  = acc_y->getVariance( );
    std::vector< double > acc_d_y  = acc_y->getDeviation( );
    std::vector< double > acc_ts_y = acc_y->getTimes( );

    acc_z->calc( );
    std::vector< double > acc_v_z  = acc_z->getVariance( );
    std::vector< double > acc_d_z  = acc_z->getDeviation( );
    std::vector< double > acc_ts_z = acc_z->getTimes( );

    writeData3( IMU_NAME + "_acc", acc_ts_x, acc_d_x, acc_d_y, acc_d_z );
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
