#include "datacontrol.h"

DataControl::DataControl()
{
//    memset(&ClientToServerInitParam, 0, sizeof(ClientToServerInitParam));
    memset(&ClientToServer, 0, sizeof(ClientToServer));
    memset(&ServerToClient, 0, sizeof(ServerToClient));
    logging_start = false;
}

DataControl::~DataControl()
{
}

void DataControl::DataReset()
{
//    memset(&ClientToServerInitParam, 0, sizeof(ClientToServerInitParam));
    memset(&ClientToServer, 0, sizeof(ClientToServer));
    memset(&ServerToClient, 0, sizeof(ServerToClient));
    logging_start = false;
}

