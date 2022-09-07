#ifndef POINT_SUBSCRIBER_H
#define POINT_SUBSCRIBER_H
#include "nav_cctv/Locations.h"

int* Topic_X;
int* msg_seq;

void pointCallback(const nav_cctv::LocationsConstPtr&Topic_point);

#endif