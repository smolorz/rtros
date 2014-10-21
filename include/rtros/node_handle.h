/*
 *  Copyright (c) 2014, Sebastian Smolorz <sesmo@gmx.net>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef RTROS_NODE_HANDLE_H
#define RTROS_NODE_HANDLE_H

#include "init.h"
#include "publisher.h"
#include "subscriber.h"

namespace rtros
{

class NodeHandle
{
public:
    NodeHandle() {}
    ~NodeHandle() {}

    template <typename M>
        Publisher advertise(const std::string& topic, uint32_t slots)
    {
        uint32_t randomCmdMbxAdr = rand();
        module_argTab[0].val.i = RackName::systemId(randomCmdMbxAdr);
        module_argTab[1].val.i = RackName::instanceId(randomCmdMbxAdr);

        Publisher pub(slots, RackName::classId(randomCmdMbxAdr));

        if (pub.moduleInit<M>(topic))
            ROS_ERROR("moduleInit failed, Publisher not functional!");
        else
            rackModules.push_back(&pub);

        return pub;
    }

    template <typename M>
        Subscriber subscribe(const std::string &topic, uint32_t queue_size,
                             void(*fp)(const boost::shared_ptr< M const > &))
    {
        uint32_t randomCmdMbxAdr = rand();
        module_argTab[0].val.i = RackName::systemId(randomCmdMbxAdr);
        module_argTab[1].val.i = RackName::instanceId(randomCmdMbxAdr);

        Subscriber sub(sizeof(M), queue_size, RackName::classId(randomCmdMbxAdr));

        if (sub.moduleInit(topic, fp))
            ROS_ERROR("moduleInit failed, Subscriber not functional!");
        else
            rackModules.push_back(&sub);

        return sub;
    }
};

}
#endif // RTROS_NODE_HANDLE_H
