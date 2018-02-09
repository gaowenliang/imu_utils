#ifndef FITALLAN_H
#define FITALLAN_H

#include <ceres/ceres.h>
#include <cmath>
#include <eigen3/Eigen/Eigen>

#define M_PI 3.141592653589

class FitAllan
{
    class AllanSigmaError
    {
        public:
        AllanSigmaError( const double& _sigma2, const double& _tau )
        : sigma2( _sigma2 )
        , tau( _tau )
        {
        }

        template< typename T >
        T calcSigma2( T _Q, T _N, T _B, T _K, T _R, T _tau ) const
        {
            // clang-format off
            return  _Q / ( _tau * _tau )
                  + _N / _tau
                  + _B
                  + _K * _tau
                  + _R * _tau * _tau;
            // clang-format on
        }

        template< typename T >
        bool operator( )( const T* const _paramt, T* residuals ) const
        {
            T _Q   = T( _paramt[0] );
            T _N   = T( _paramt[1] );
            T _B   = T( _paramt[2] );
            T _K   = T( _paramt[3] );
            T _R   = T( _paramt[4] );
            T _tau = T( tau );

            //            T _sigma2 = calcSigma2( _Q, _N, _B, _K, _R, _tau );
            //            T _sigma2 = _Q / ( _tau * _tau ) + _N / _tau + _B + _K * _tau + _R *
            //            _tau * _tau;
            T _sigma2 = _B * _B + _K * _K * _tau + _R * _R * _tau * _tau
                        + _Q * _Q / ( _tau * _tau ) + _N * _N / _tau;

            T _dsigma2   = _sigma2 - T( sigma2 );
            residuals[0] = _dsigma2;
            //            std::cout << "_err " << T( sigma2 ) << " " << _sigma2 << std::endl;

            return true;
        }

        double sigma2;
        double tau;
    };

    public:
    FitAllan( std::vector< double > sigma2s, std::vector< double > taus );
    std::vector< double > initValue( std::vector< double > sigma2s, std::vector< double > taus )
    {
        if ( sigma2s.size( ) != taus.size( ) )
            std::cout << " Error with data size!!!" << std::endl;

        Eigen::MatrixXd Y( sigma2s.size( ), 1 );

        for ( int index = 0; index < sigma2s.size( ); ++index )
        {
            Y( index, 0 ) = sigma2s[index];
        }

        int m = 2;

        Eigen::MatrixXd B( 2 * m + 1, 1 );
        B.setZero( );

        Eigen::MatrixXd F( taus.size( ), 2 * m + 1 );
        F.setZero( );

        for ( int index = 0; index < taus.size( ); ++index )
            for ( int order_index = 0; order_index < 2 * m + 1; ++order_index )
            {
                int kk = order_index - m;
                F( index, order_index ) = pow( taus[index], kk );
            }
        //        std::cout << "F " << F << std::endl;

        Eigen::MatrixXd A = F.transpose( ) * F;
        B                 = F.transpose( ) * Y;
        //        std::cout << "B " << B << std::endl;
        //        std::cout << "A " << A << std::endl;

        Eigen::MatrixXd C = A.inverse( ) * B;

        std::vector< double > init;
        for ( int index = 0; index < 2 * m + 1; ++index )
            init.push_back( C( index, 0 ) );

        std::cout << "C " << C << std::endl;
        return init;
    }
    /**
     * @brief getQ
     *          Quantization Noise
     * @unit: degree
     * @return
     */
    double getQ( ) const;
    /**
     * @brief getN
     *          Angle Random Walk
     * @unit: degree / sqrt( hour )
     * @return
     */
    double getN( ) const;
    /**
     * @brief getB
     *        Bias Instability
     * @unit: degree / hour
     * @return
     */
    double getB( ) const;
    /**
     * @brief getK
     *      Rate Random Walk
     * @unit: degree / (hour*sqrt(hour))
     * @return
     */
    double getK( ) const;
    /**
     * @brief getR
     *        Angle Rate Ramp
     * @unit: degree / (hour * hour)
     * @return
     */
    double getR( ) const;

    double Q;
    double N;
    double B;
    double K;
    double R;
};

#endif // FITALLAN_H
