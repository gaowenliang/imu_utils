#ifndef UTILS_H
#define UTILS_H

#include <vector>

#define M_PI_CON 3.141592653589

namespace utils
{
template< class T >
T
avg( const std::vector< T > datas )
{
    T sum_data = T( 0 );
    for ( auto data : datas )
        sum_data += data;
    return sum_data / datas.size( );
}
}

#endif // UTILS_H
