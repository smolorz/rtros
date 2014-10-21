/*
 *  Copyright (c) 2014, Sebastian Smolorz <sesmo@gmx.net>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include "rtros/subscriber.h"

namespace rtros
{

Subscriber::Subscriber(size_t size, uint32_t queue_size, uint32_t classId)
    : RackDataModule(classId,
                     5000000000llu,                  // 5s datatask error sleep time
                     16,                             // command mailbox slots
                     48,                             // command mailbox data size per slot
                     MBX_IN_KERNELSPACE | MBX_SLOT,  // command mailbox flags
                     1, 1)
{
    msg_size = size;
    slots = queue_size;
}

void Subscriber::getStopContData(PublisherProxy* proxy, int get_stop_cont)
{
    if (get_stop_cont == MSG_GET_CONT_DATA)
    {
        if (proxy->getContData(0, &(this->dataMbx), 0))
        {
            ROS_ERROR("Can't get continuous data from publisher over TiMS");
        }
    }
    else if (get_stop_cont == MSG_STOP_CONT_DATA)
    {
        proxy->stopContData(&(this->dataMbx));
    }
}

void get_stop_cont_data_proc(void *arg)
{
    SubProxyGetStopCont* subProxyGetStopCont = (SubProxyGetStopCont *)arg;
    subProxyGetStopCont->sub->getStopContData(subProxyGetStopCont->proxy,
                                              subProxyGetStopCont->getStopCont);
}

void recv_data_msg_proc(void *arg)
{
    unsigned long event_mask_r;
    Subscriber* sub = (Subscriber *)arg;

    rt_event_wait(&recv_data_msg_event, 1, &event_mask_r, EV_ANY, TM_INFINITE);

    sub->recvDataMsg();
}

void Subscriber::getContData(uint32_t cmd_mbx_addr)
{
    PublisherProxy* proxy;

    ROS_INFO("I'm gonna subscribe to %x", cmd_mbx_addr);

    proxy = new PublisherProxy(&workMbx, RackName::systemId(cmd_mbx_addr),
                               RackName::classId(cmd_mbx_addr),
                               RackName::instanceId(cmd_mbx_addr));
    publishers.push_back(proxy);
    SubProxyGetStopCont arg_for_rt_proc = {this, proxy, MSG_GET_CONT_DATA};
    rt_task_start(&get_stop_cont_data_task, &get_stop_cont_data_proc, &arg_for_rt_proc);
}

void Subscriber::stopContData(void)
{
    for (std::vector<PublisherProxy*>::iterator c = publishers.begin();
         c != publishers.end(); ++c)
    {
        SubProxyGetStopCont arg_for_rt_proc = {this, *c, MSG_STOP_CONT_DATA};
        rt_task_start(&get_stop_cont_data_task, &get_stop_cont_data_proc, &arg_for_rt_proc);
    }
}

void Subscriber::recvDataMsg(void)
{
    int ret;
    RackMessage info;

    do {
        ret = dataMbx.recvDataMsg(msg_data, msg_size, &info);
        if (!ret)
            msgHelper->call();
    } while (ret == 0);
}

}
