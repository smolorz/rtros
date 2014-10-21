/*
 *  Copyright (c) 2014, Sebastian Smolorz <sesmo@gmx.net>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef RTROS_RATE_H
#define RTROS_RATE_H

#include <native/task.h>

namespace rtros
{

class Rate
{

public:
    Rate(double frequency)
    {
        period = 1000000000llu / frequency;
    }
    ~Rate() {}

    int setPeriodic()
    {
        return rt_task_set_periodic(NULL, TM_NOW, period);
    }

    int sleep()
    {
        return rt_task_wait_period(NULL);
    }

private:
    RTIME period;
};

}
#endif // RTROS_RT_TASK_H
