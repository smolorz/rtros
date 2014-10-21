/*
 *  Copyright (c) 2014, Sebastian Smolorz <sesmo@gmx.net>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef RTROS_PUBLISHER_PROXY_H
#define RTROS_PUBLISHER_PROXY_H

#include <main/rack_proxy.h>

namespace rtros
{

class PublisherProxy : public RackDataProxy
{

public:
    PublisherProxy(RackMailbox *workMbx, uint32_t sys_id, uint32_t class_id,
                   uint32_t instance)
        : RackDataProxy(workMbx, sys_id, class_id, instance)
    {}

    ~PublisherProxy() {}
};

}
#endif // RTROS_PUBLISHER_PROXY_H
