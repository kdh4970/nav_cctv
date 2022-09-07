#ifndef POINT_SUBSCRIBER_H
#define POINT_SUBSCRIBER_H
#include "nav_cctv/Locations.h"

namespace PointSub{
    int Topic_X;
    int* x_Ptr;
    int msg_seq;
    int* seq_Ptr;
    void pointCallback(const nav_cctv::LocationsConstPtr&Topic_point);

}



#endif