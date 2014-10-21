/*
 *  Copyright (c) 2014, Sebastian Smolorz <sesmo@gmx.net>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef RTROS_RT_TASK_H
#define RTROS_RT_TASK_H

#include <native/task.h>

namespace rtros
{

class RtShadowTask
{

public:
    RtShadowTask() {}
    ~RtShadowTask() {}

    int nrt2RT(const char* name, int prio = 1)
    {
        return rt_task_shadow(&task, name, prio, 0);
    }

private:
    RT_TASK task;
};

}
#endif // RTROS_RT_TASK_H
