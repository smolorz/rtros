/*
 *  Copyright (c) 2014, Sebastian Smolorz <sesmo@gmx.net>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include "rtros/init.h"

namespace rtros
{

std::string node_name;
RT_TASK main_shadow_task;
RT_EVENT recv_data_msg_event;
std::vector<RackModule*> rackModules;


void sigxcpu_handler(int sig) {}

int init(int& argc, char** argv, const std::string& name, uint32_t options)
{
    ros::init(argc, argv, name, options);

    node_name = name;

    srand(time(NULL));

    signal(SIGINT, rtros_signal_handler);
    signal(SIGXCPU, sigxcpu_handler);

    rt_print_auto_init(1);

    rt_event_create(&recv_data_msg_event, "recv_data_msg_event", 0, EV_PRIO);

    return rt_task_shadow(&main_shadow_task, name.c_str(), 0, 0);
}

int set_priority(int prio)
{
    return rt_task_set_priority(&main_shadow_task, prio);
}

void spin(void)
{
    rt_event_signal(&recv_data_msg_event, 1);
    ros::spin();
}

void shutdown()
{
    for (std::vector<RackModule*>::iterator mod = rackModules.begin();
         mod != rackModules.end(); ++mod)
    {
        (*mod)->moduleTerminate();
    }

    ros::shutdown();
}

void rtros_signal_handler(int sig)
{
    shutdown();
}

}
