#include "fitallan_gyr.h"

using namespace imu;

FitAllanGyr::FitAllanGyr( std::vector< double > sigma2s, std::vector< double > taus, double _freq )
: Q( 0.0 )
, N( 0.0 )
, B( 0.0 )
, K( 0.0 )
, R( 0.0 )
, freq( _freq )
{
    if ( sigma2s.size( ) != taus.size( ) )
        std::cerr << "Error of point size" << std::endl;

    m_taus = taus;

    std::vector< double > init = initValue( sigma2s, taus );

    int num_samples = sigma2s.size( );
    //    double param[]  = { Q, N, B, K, R };
    double param[] = { init[0], init[1], init[2], init[3], init[4] };

    ceres::Problem problem;

    for ( int i = 0; i < num_samples; ++i )
    {
        // std::cout << "sigma " << i << " " << taus[i] << " " << sigma2s[i] << std::endl;

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

    std::cout << " Bias Instability " << getB( ) / ( 57.3 * 3600 ) << " rad/s" << std::endl;
    std::cout << " Bias Instability " << getBiasInstability( ) << " rad/s, at "
              << taus[findMinIndex( calcSimDeviation( taus ) )] << " s" << std::endl;

    std::cout << " White Noise " << sqrt( freq ) * getN( ) * 60 / 57.3 << " rad/s" << std::endl;
    std::cout << " White Noise " << getWhiteNoise( ) << " rad/s" << std::endl;
}

std::vector< double >
FitAllanGyr::initValue( std::vector< double > sigma2s, std::vector< double > taus )
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
            int kk                  = order_index - m_order;
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

std::vector< double >
FitAllanGyr::calcSimDeviation( const std::vector< double > taus ) const
{
    std::vector< double > des;
    for ( auto& tau : taus )
        des.push_back( sqrt( calcSigma2( Q, N, B, K, R, tau ) ) );
    return des;
}

double
FitAllanGyr::getBiasInstability( ) const
{
    return findMinNum( calcSimDeviation( m_taus ) ) / ( 57.3 * 3600 );
}

double
FitAllanGyr::getWhiteNoise( ) const
{
    return sqrt( freq ) * sqrt( calcSigma2( Q, N, B, K, R, 1 ) ) / ( 57.3 * 3600 );
}

double
FitAllanGyr::findMinNum( const std::vector< double > num ) const
{
    double min = 1000.0;
    for ( unsigned int index = 0; index < num.size( ); ++index )
        min = min < num[index] ? min : num[index];
    return min;
}

int
FitAllanGyr::findMinIndex( std::vector< double > num )
{
    double min    = 1000.0;
    int min_index = 0;
    for ( unsigned int index = 0; index < num.size( ); ++index )
    {
        min_index = min < num[index] ? min_index : index;
        min       = min < num[index] ? min : num[index];
    }
    return min_index;
}

double
FitAllanGyr::calcSigma2( double _Q, double _N, double _B, double _K, double _R, double _tau ) const
{
    // clang-format off
  return  _Q * _Q / ( _tau * _tau )
      + _N * _N / _tau
      + _B * _B
      + _K * _K * _tau
      + _R * _R * _tau * _tau;
    // clang-format on
}

double
FitAllanGyr::getN( ) const
{
    return sqrt( N * N ) / 60.0;
}

double
FitAllanGyr::getB( ) const
{
    return sqrt( B * B ) / 0.6642824703;
}

double
FitAllanGyr::getK( ) const
{
    return 60.0 * sqrt( 3.0 * K * K );
}

double
FitAllanGyr::getR( ) const
{
    return 3600.0 * sqrt( 2.0 * R * R );
}

double
FitAllanGyr::getQ( ) const
{
    return sqrt( Q * Q ) / ( 3600.0 * sqrt( 3.0 ) );
}
