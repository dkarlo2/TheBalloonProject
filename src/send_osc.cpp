#include "send_osc.h"

#include <iostream>
#include <cstdio>

#define OUTPUT_BUFFER_SIZE 1024

MyOSCSender::MyOSCSender(ConfigParser config) {
    const char* address = config.getString("oscAddress");
    int port = config.getInt("oscPort");
    transmitSocket = new UdpTransmitSocket(IpEndpointName(address, port));
}

void MyOSCSender::sendPosition(int ball, double x, double y, double z) {
    char buffer[OUTPUT_BUFFER_SIZE];
    osc::OutboundPacketStream p(buffer, OUTPUT_BUFFER_SIZE);

    char sx[10], sy[10], sz[10];
    sprintf(sx, "/ball%d/x", ball);
    sprintf(sy, "/ball%d/y", ball);
    sprintf(sz, "/ball%d/z", ball);

    p << osc::BeginBundleImmediate
            << osc::BeginMessage(sx) << x << osc::EndMessage
            << osc::BeginMessage(sy) << y << osc::EndMessage
            << osc::BeginMessage(sz) << z << osc::EndMessage
      << osc::EndBundle;

    transmitSocket->Send(p.Data(), p.Size());
}
