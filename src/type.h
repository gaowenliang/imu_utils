#ifndef TYPE_H
#define TYPE_H

class Data
{
    public:
    Data( )
    : v( 0.0 )
    , t( 0.0 )
    {
    }
    Data( double data, double time )
    : v( data )
    , t( time )
    {
    }

    double v;
    double t;
};

class AccData
{
    public:
    AccData( )
    : a( 0.0 )
    , t( 0.0 )
    {
    }
    AccData( double data, double time )
    : a( data )
    , t( time )
    {
    }

    double a;
    double t;
};

class GyrData
{
    public:
    GyrData( )
    : w( 0.0 )
    , t( 0.0 )
    {
    }
    GyrData( double data, double time )
    : w( data )
    , t( time )
    {
    }

    double w;
    double t;
};

#endif // TYPE_H
