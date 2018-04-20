/* 
 * File:   send_osc.h
 * Author: Karlo
 *
 * Created on November 23, 2015, 4:30 PM
 */

#ifndef SEND_OSC_H
#define	SEND_OSC_H

#include "config_parser.h"

#include "osc/OscOutboundPacketStream.h"
#include "ip/UdpSocket.h"

class MyOSCSender {
    UdpTransmitSocket *transmitSocket;
public:
    MyOSCSender(ConfigParser config);
    void sendPosition(int ball, double x, double y, double z);
};

#endif	/* SEND_OSC_H */

