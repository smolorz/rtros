/*
 *  Copyright (c) 2014, Sebastian Smolorz <sesmo@gmx.net>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef RTROS_INIT_H
#define RTROS_INIT_H

#include <signal.h>
#include <native/task.h>
#include <native/event.h>
#include <rtdk.h>
#include <ros/init.h>
#include <main/rack_module.h>

namespace rtros
{

extern std::string node_name;
extern RT_EVENT recv_data_msg_event;
extern std::vector<RackModule*> rackModules;

int init(int &argc, char **argv, const std::string& name, uint32_t options = 0);

int set_priority(int prio);

void spin(void);

void shutdown();

void rtros_signal_handler(int sig);

}
#endif // RTROS_INIT_H
