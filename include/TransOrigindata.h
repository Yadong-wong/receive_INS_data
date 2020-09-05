//
// Created by agv on 2020/4/8.
//

#ifndef SRC_TRANSFULLDATA_H
#define SRC_TRANSFULLDATA_H

#define SERVER_PORT 6060
#define BUFF_LEN 1024

#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/PointCloud2.h>
#include <frmudpserver/Initial_data.h>
#include <geometry_msgs/PoseStamped.h>
#include "receive_INS_data/gsof.h"

#include <time.h>
#include <tf/transform_datatypes.h>

#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>

#include <QtNetwork>
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>

using namespace std;

string stCurrentTime;
string filename;

#endif //SRC_TRANSFULLDATA_H
