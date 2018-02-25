#include "fitallan.h"

FitAllan::FitAllan( std::vector< double > sigma2s, std::vector< double > taus )
: Q( 0.0 )
, N( 0.0 )
, B( 0.0 )
, K( 0.0 )
, R( 0.0 )
{
    if ( sigma2s.size( ) != taus.size( ) )
        std::cerr << "Error of point size" << std::endl;

    std::vector< double > init = initValue( sigma2s, taus );

    int num_samples = sigma2s.size( );
    //    double param[]  = { Q, N, B, K, R };
    double param[] = { init[0], init[1], init[2], init[3], init[4] };

    ceres::Problem problem;

    for ( int i = 0; i < num_samples; ++i )
    {

        //        std::cout << "sigma " << i << " " << taus[i] << " " << sigma2s[i] << std::endl;

        ceres::CostFunction* f = new ceres::AutoDiffCostFunction< AllanSigmaError, 1, 5 >(
        new AllanSigmaError( sigma2s[i], taus[i] ) );

        problem.AddResidualBlock( f, NULL /* squared loss */, param );
    }
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.logging_type                 = ceres::SILENT;
    options.trust_region_strategy_type   = ceres::DOGLEG;
    //    options.max_num_iterations         = 5;

    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );
    //        std::cout << summary.FullReport( ) << "\n";
    //    std::cout << "num_parameters " << summary.num_parameters << std::endl;

    Q = param[0];
    N = param[1];
    B = param[2];
    K = param[3];
    R = param[4];

    // std::cout << "Q " << Q //
    //           << " " << N  //
    //           << " " << B  //
    //           << " " << K  //
    //           << " " << R << std::endl;

    std::cout << " Bias Instability " << findMinNum( calcSimDeviation( taus ) ) / ( 57.3 * 3600 )
              << " rad/s, at " << taus[findMinIndex( calcSimDeviation( taus ) )] << " s" << std::endl;
    std::cout << " White Noise " << sqrt( calcSigma2( Q, N, B, K, R, 1 ) ) / ( 57.3 * 3600 )
              << " rad/s" << std::endl;

    std::cout << " Bias Instability " << findMinNum( calcSimDeviation( taus ) ) << " rad/s" << std::endl;
    std::cout << " White Noise " << sqrt( calcSigma2( Q, N, B, K, R, 1 ) ) << " rad/s" << std::endl;
}

std::vector< double >
FitAllan::initValue( std::vector< double > sigma2s, std::vector< double > taus )
{
    if ( sigma2s.size( ) != taus.size( ) )
        std::cout << " Error with data size!!!" << std::endl;

    Eigen::MatrixXd Y( sigma2s.size( ), 1 );

    for ( unsigned int index = 0; index < sigma2s.size( ); ++index )
    {
        Y( index, 0 ) = sqrt( sigma2s[index] );
    }
    //        std::cout << "Y " << Y << std::endl;

    int m_order = 2;

    Eigen::MatrixXd B( 2 * m_order + 1, 1 );
    B.setZero( );

    Eigen::MatrixXd F( taus.size( ), 2 * m_order + 1 );
    F.setZero( );

    for ( unsigned int index = 0; index < taus.size( ); ++index )
        for ( int order_index = 0; order_index < 2 * m_order + 1; ++order_index )
        {
            int kk = order_index - m_order;
            F( index, order_index ) = pow( sqrt( taus[index] ), kk );
        }
    //        std::cout << "F " << F << std::endl;

    Eigen::MatrixXd A = F.transpose( ) * F;
    B                 = F.transpose( ) * Y;
    //        std::cout << "B " << B << std::endl;
    //        std::cout << "A " << A << std::endl;

    Eigen::MatrixXd C = A.inverse( ) * B;
    std::cout << "C " << C.transpose( ) << std::endl;

    std::vector< double > init;
    for ( int index = 0; index < 2 * m_order + 1; ++index )
        init.push_back( std::abs( C( index, 0 ) ) );

    return init;
}

double
FitAllan::getN( ) const
{
    return sqrt( N * N ) / 60.0;
}

double
FitAllan::getB( ) const
{
    return sqrt( B * B ) / 0.6642824703;
}

double
FitAllan::getK( ) const
{
    return 60.0 * sqrt( 3.0 * K * K );
}

double
FitAllan::getR( ) const
{
    return 3600.0 * sqrt( 2.0 * R * R );
}

double
FitAllan::getQ( ) const
{
    return sqrt( Q * Q ) / ( 3600.0 * sqrt( 3.0 ) );
}

// double
// FitAllan::getN( ) const
//{
//    return sqrt( N ) / 60.0;
//}

// double
// FitAllan::getB( ) const
//{
//    return sqrt( B ) / 0.6642824703;
//}

// double
// FitAllan::getK( ) const
//{
//    return 60.0 * sqrt( 3.0 * K );
//}

// double
// FitAllan::getR( ) const
//{
//    return 3600.0 * sqrt( 2.0 * R );
//}

// double
// FitAllan::getQ( ) const
//{
//    return sqrt( Q ) / ( 3600.0 * sqrt( 3.0 ) );
//}
