#ifndef ALLAN_H
#define ALLAN_H

#include <iostream>
#include <vector>

namespace imu
{

class Allan
{
    public:
    Allan( ) {}

    std::vector< double > rawData;
};
}
#endif // ALLAN_H
