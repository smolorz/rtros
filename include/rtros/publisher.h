/*
 *  Copyright (c) 2014, Sebastian Smolorz <sesmo@gmx.net>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef RTROS_PUBLISHER_H
#define RTROS_PUBLISHER_H

#include <ros/ros.h>

#include "init.h"

#include <main/rack_data_module.h>

extern arg_table_t argTab[];

namespace rtros
{

class Publisher : public RackDataModule
{

public:
    Publisher(uint32_t slots, uint32_t classId);
    ~Publisher() {}

    template <typename M>
        int moduleInit(const std::string& topic)
    {
        int ret;
        ros::AdvertiseOptions ops;

        save_argTab(argTab, (node_name + "_" + topic).c_str());
        
        setDataBufferMaxDataSize(sizeof(M));

        ret = RackDataModule::moduleInit();
        if (ret)
            return ret;

        ret = startCmdTask();
        if (ret)
            return ret;

        ops.template init<M>(topic, 1);
        ops.tims_addr = getName();
        rosPublisher = rosNh.advertise(ops);

        return 0;
    }

    template <typename M>
        void publish(const M& message)
    {
        void*     dataBuffer;
        uint32_t  datalength;
        
        dataBuffer = getDataBufferWorkSpace();
        datalength = getDataBufferMaxDataSize();
        memcpy(dataBuffer, &message.data, datalength);
        putDataBufferWorkSpace(datalength);
    }

private:
    ros::Publisher rosPublisher;
    ros::NodeHandle rosNh;
};

}
#endif // RTROS_PUBLISHER_H
