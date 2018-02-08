#ifndef ALLAN_H
#define ALLAN_H

#include "type.h"
#include <iostream>
#include <math.h>
#include <vector>

namespace imu
{

class Allan
{
    public:
    Allan( int maxCluster = 10000 )
    : numData( 0 )
    , numCluster( maxCluster )
    {
        std::cout << " num of Cluster " << numCluster << std::endl;
    }
    ~Allan( )
    {
        m_rawData.clear( );
        m_thetas.clear( );
        mFactors.clear( );
    }

    void push( double data, double time )
    {
        m_rawData.push_back( GyrData( data, time ) );
        numData++;
    }
    std::vector< double > calc( )
    {
        std::cout << " numData " << numData << std::endl;
        if ( numData < 10000 )
            std::cout << " Too few number" << std::endl;

        double start_t = m_rawData.begin( )->t;
        double end_t   = m_rawData[numData - 1].t;
        std::cout << " start_t " << start_t << std::endl;
        std::cout << " end_t " << end_t << std::endl;
        std::cout << " dt " //
                  << ( end_t - start_t ) << " s" << std::endl
                  << ( end_t - start_t ) / 60 << " min" << std::endl
                  << ( end_t - start_t ) / 3600 << " h" << std::endl;

        if ( ( end_t - start_t ) / 60 < 10 )
            std::cout << " Too few time!!!!" << std::endl;

        double freq = getAvgFreq( );
        std::cout << " freq " << freq << std::endl;

        double period = getAvgPeriod( );
        std::cout << " period " << period << std::endl;

        m_thetas = calcThetas( freq );

        initStrides( );
        std::cout << " initStrides. " << std::endl;

        std::vector< double > variance = getVariance( period );
        return variance;
    }

    std::vector< double > getVariance( double period )
    {
        std::vector< double > sigma2( numFactors, 0.0 );

        for ( int i = 0; i < numFactors; i++ )
        {
            int factor           = ( int )mFactors[i];
            float clusterPeriod2 = ( period * factor ) * ( period * factor );
            float devided        = 2 * clusterPeriod2 * ( period - 2 * factor );
            int averageFactorX2  = 2 * factor;
            int max              = numData - averageFactorX2;

            for ( int k = 0; k < max; k++ )
            {
                float temp = ( m_thetas[k + averageFactorX2] - 2 * m_thetas[k + factor] + m_thetas[k] );
                sigma2[i] += ( temp * temp );
            }

            sigma2[i] = sigma2[i] / devided;
        }
        return sigma2;
    }

    std::vector< double > getDeviation( )
    {
        double period = getAvgPeriod( );

        std::vector< double > sigma2 = getVariance( period );
        std::vector< double > sigma;

        for ( auto& sig : sigma2 )
        {
            sigma.push_back( sqrt( sig ) );
        }
        return sigma;
    }

    private:
    std::vector< double > calcThetas( const double freq )
    {
        std::vector< double > thetas;

        double sum = 0;
        for ( auto& gyro : m_rawData )
        {
            sum += gyro.w;
            thetas.push_back( sum / freq );
        }
        return thetas;
    }
    void initStrides( )
    {

        int mode               = numData / 2;
        unsigned int maxStride = 1;
        int shft               = 0;
        while ( mode )
        {
            mode      = mode >> 1;
            maxStride = 1 << shft;
            shft++;
        }
        std::cout << " mode: " << mode << std::endl;
        std::cout << " shft: " << shft << std::endl;

        std::vector< double > avgFactors = getLogSpace( 0, log10( maxStride ) );

        for ( int i = 0; i < numCluster; i++ )
        {
            avgFactors[i] = ceil( avgFactors[i] );
        }

        std::vector< int > factors( numCluster, 0 );

        numFactors = 1;
        factors[0] = ( int )avgFactors[0];

        for ( int i = 1; i < numCluster; i++ )
        {
            std::cout << " avgFactors " << i << " " << avgFactors[i] << std::endl;
            if ( avgFactors[i] != avgFactors[i - 1] )
            {
                factors[numFactors] = ( int )avgFactors[i];
                numFactors++;
            }
        }

        mFactors = factors;

        std::cout << " numFactors " << numFactors << std::endl;
    }
    std::vector< double > getLogSpace( float a, float b )
    {
        std::vector< double > logSpace;

        double start       = pow( 10, a );
        double end         = pow( 10, b );
        double progression = pow( end / start, ( float )1 / ( numCluster - 1 ) );

        logSpace.push_back( start );
        for ( int i = 1; i < numCluster; i++ )
        {
            logSpace.push_back( logSpace[i - 1] * progression );
        }
        std::cout << "size logSpace " << logSpace.size( ) << std::endl;
    }
    double getAvgFreq( ) { return 1.0 / getAvgDt( ); }
    double getAvgDt( )
    {
        double sum_dt  = 0.0;
        double start_t = m_rawData[0].t;
        bool first     = true;
        for ( auto& gyro : m_rawData )
        {
            if ( !first )
                sum_dt += ( gyro.t - start_t );
            first = false;
        }
        return sum_dt / ( numData - 1 );
    }
    double getAvgPeriod( ) { return getAvgDt( ); }
    int getFactorsNum( ) { return numFactors; }

    int numData;
    std::vector< GyrData > m_rawData;
    std::vector< double > m_thetas;
    int numCluster;
    int numFactors;
    std::vector< int > mFactors;
};
}

#endif // ALLAN_H
