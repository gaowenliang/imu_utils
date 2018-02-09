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

    initValue( sigma2s, taus );

    int num_samples = sigma2s.size( );
    double param[]  = { Q, N, B, K, R };

    ceres::Problem problem;

    for ( int i = 0; i < num_samples; ++i )
    {

        std::cout << "sigma " << i << " " << taus[i] << " " << sigma2s[i] << std::endl;

        ceres::CostFunction* f = new ceres::AutoDiffCostFunction< AllanSigmaError, 1, 5 >(
        new AllanSigmaError( sigma2s[i], taus[i] ) );

        problem.AddResidualBlock( f, NULL /* squared loss */, param );
    }
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    //        options.logging_type                 = ceres::SILENT;
    options.trust_region_strategy_type = ceres::DOGLEG;
    //    options.max_num_iterations         = 5;

    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );
    //        std::cout << summary.FullReport( ) << "\n";
    std::cout << "num_parameters " << summary.num_parameters << std::endl;

    Q = param[0];
    N = param[1];
    B = param[2];
    K = param[3];
    R = param[4];

    std::cout << "Q " << Q << std::endl;
    std::cout << "N " << N << std::endl;
    std::cout << "B " << B << std::endl;
    std::cout << "K " << K << std::endl;
    std::cout << "R " << R << std::endl;
}

double
FitAllan::getN( ) const
{
    return sqrt( N ) / 60.0;
}

double
FitAllan::getB( ) const
{
    return sqrt( B ) / 0.6642824703;
}

double
FitAllan::getK( ) const
{
    return 60.0 * sqrt( 3.0 * K );
}

double
FitAllan::getR( ) const
{
    return 3600.0 * sqrt( 2.0 * R );
}

double
FitAllan::getQ( ) const
{
    return sqrt( Q ) / ( 3600.0 * sqrt( 3.0 ) );
}
