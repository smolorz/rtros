/*
 *  Copyright (c) 2014, Sebastian Smolorz <sesmo@gmx.net>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#ifndef RTROS_SUBSCRIBER_H
#define RTROS_SUBSCRIBER_H

#include <ros/ros.h>

#include "init.h"
#include "publisher_proxy.h"

#include <main/rack_data_module.h>

extern arg_table_t argTab[];

namespace rtros
{

// init_flags (for init and cleanup)
#define INIT_BIT_MODULE                 0
#define INIT_BIT_MBX_DATA               1
#define INIT_BIT_MBX_WORK               2

class Subscriber;

struct SubProxyGetStopCont
{
    Subscriber*     sub;
    PublisherProxy* proxy;
    int             getStopCont;
};

void recv_data_msg_proc(void *arg);

class MessageHelper
{
public:
    virtual ~MessageHelper() {}
    virtual const void* getMsgVoidPtr() = 0;
    virtual void call() = 0;
};

template <typename M>
class MessageHelperT : public MessageHelper
{
public:    
    typedef M msgType;
    typedef boost::shared_ptr<M const> msgConstPtr;
    typedef boost::function<void(msgConstPtr&)> Callback;

    MessageHelperT(void(*fp)(const boost::shared_ptr< M const > &))
    {
       msgConstPtr message__(new msgType);
       message_ = message__;
       callback_ = fp;
    }
    ~MessageHelperT() {}

    const void* getMsgVoidPtr()
    {
        return message_.get();
    }

    void call()
    {
        callback_(message_);
    }

private:
    msgConstPtr message_;
    Callback callback_;
};

class Subscriber : public RackDataModule
{
public:
    Subscriber(size_t size, uint32_t queue_size, uint32_t classId);
    ~Subscriber()
    {
        moduleCleanup();
    }

    void getStopContData(PublisherProxy* proxy, int get_stop_cont);
    void getContData(uint32_t cmd_mbx_addr);
    void stopContData(void);
    void recvDataMsg(void);

    template <typename M>
        int moduleInit(const std::string &topic,
                       void(*fp)(const boost::shared_ptr< M const > &))
    {
        int ret;
        ros::SubscribeOptions ops;

        msgHelper = new MessageHelperT<M>(fp);
        msg_data = (void *)(msgHelper->getMsgVoidPtr());

        save_argTab(argTab, (node_name + "_" + topic).c_str());

        ret = RackModule::moduleInit();
        if (ret) {
            ROS_ERROR("RackModule::moduleInit() failed with error %d", ret);
            return ret;
        }
        subInitBits.setBit(INIT_BIT_MODULE);

        // work mailbox
        ret = mailboxCreate(&workMbx, rand(), 1, 128,
                            MBX_IN_KERNELSPACE | MBX_SLOT, dataTaskPrio);
        if (ret) {
            ROS_ERROR("Creating work mailbox failed with error %d", ret);
            goto init_error;
        }
        subInitBits.setBit(INIT_BIT_MBX_WORK);

        // data mailbox
        ret = mailboxCreate(&dataMbx, rand(), slots, msg_size,
                            MBX_IN_KERNELSPACE | MBX_SLOT, dataTaskPrio);
        if (ret) {
            ROS_ERROR("Creating data mailbox failed with error %d", ret);
            goto init_error;
        }
        subInitBits.setBit(INIT_BIT_MBX_DATA);

        rt_task_create(&get_stop_cont_data_task,
                      (node_name + "_" + topic + "contDataTask").c_str(),
                      0, 1, 0);

        rt_task_spawn(&recv_data_msg_task,
                      (node_name + "_" + topic + "recvDataMsgTask").c_str(),
                      0, 51, 0, &recv_data_msg_proc, this);

        ops.template init<M>(topic, 1, fp);
        ops.transport_hints = ros::TransportHints().tims();
        ops.pendConnDoneCb = boost::bind(&Subscriber::getContData, this, _1);
        ops.stopContDataCb = boost::bind(&Subscriber::stopContData, this);
        rosSubscriber = rosNh.subscribe(ops);

        return 0;

    init_error:
        moduleCleanup();
        return ret;
    }

    void moduleCleanup(void)
    {
        stopContData();
        if (subInitBits.testAndClearBit(INIT_BIT_MODULE))
            RackModule::moduleCleanup();
        if (subInitBits.testAndClearBit(INIT_BIT_MBX_DATA))
            destroyMbx(&dataMbx);
        if (subInitBits.testAndClearBit(INIT_BIT_MBX_WORK))
            destroyMbx(&workMbx);
    }

private:
    ros::Subscriber rosSubscriber;
    ros::NodeHandle rosNh;
    size_t          msg_size;
    uint32_t        slots;
    void*           msg_data;

    RackBits        subInitBits;
    RackMailbox     workMbx;
    RackMailbox     dataMbx;

    RT_TASK         get_stop_cont_data_task;
    RT_TASK         recv_data_msg_task;

    std::vector<PublisherProxy*>  publishers;

    MessageHelper*  msgHelper;
};

}
#endif // RTROS_SUBSCRIBER_H
