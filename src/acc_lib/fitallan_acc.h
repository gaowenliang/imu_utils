#ifndef FitAllanAcc_H
#define FitAllanAcc_H

#include <ceres/ceres.h>
#include <cmath>
#include <eigen3/Eigen/Eigen>

namespace imu
{

class FitAllanAcc
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
        T calcLog10( T src ) const
        {
            return ( log( src ) ) / ( log( 10 ) );
        }

        template< typename T >
        T calcSigma2( T _Q, T _N, T _B, T _K, T _R, T _tau ) const
        {
            // clang-format off
            return  _Q * _Q / ( _tau * _tau )
                  + _N * _N / _tau
                  + _B * _B
                  + _K * _K * _tau
                  + _R * _R * _tau * _tau;
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

            T _sigma2    = calcSigma2( _Q, _N, _B, _K, _R, _tau );
            T _dsigma2   = T( calcLog10( _sigma2 ) ) - T( calcLog10( sigma2 ) );
            residuals[0] = _dsigma2;
            //            std::cout << "_err " << T( sigma2 ) << " " << _sigma2
            //            << std::endl;

            return true;
        }

        double sigma2;
        double tau;
    };

    public:
    FitAllanAcc( std::vector< double > sigma2s, std::vector< double > taus, double _freq );
    std::vector< double > calcSimDeviation( const std::vector< double > taus ) const;
    double getBiasInstability( ) const;
    double getWhiteNoise( ) const;

    private:
    std::vector< double > checkData( std::vector< double > sigma2s, std::vector< double > taus );

    std::vector< double > initValue( std::vector< double > sigma2s, std::vector< double > taus );
    double findMinNum( const std::vector< double > num ) const;
    int findMinIndex( std::vector< double > num );
    double calcSigma2( double _Q, double _N, double _B, double _K, double _R, double _tau ) const;

    public:
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

    private:
    std::vector< double > m_taus;
    double freq;
};
}

#endif // FitAllanAcc_H
