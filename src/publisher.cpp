/*
 *  Copyright (c) 2014, Sebastian Smolorz <sesmo@gmx.net>
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#include <main/rack_module.h>
#include "rtros/publisher.h"

arg_table_t argTab[] = {{ 0, "", 0, 0, "", { 0 } }};

namespace rtros
{
    
Publisher::Publisher(uint32_t slots, uint32_t classId)
    : RackDataModule(classId,
                         5000000000llu,                  // 5s datatask error sleep time
                         16,                             // command mailbox slots
                         48,                             // command mailbox data size per slot
                         MBX_IN_KERNELSPACE | MBX_SLOT,  // command mailbox flags
                         slots,                          // max buffer entries
                         10)                             // data buffer listener
    {}

}
