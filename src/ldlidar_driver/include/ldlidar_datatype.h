/**
 * @file ldlidar_datatype.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  lidar data structure
 *         This code is only applicable to LDROBOT products
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-09
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef _POINT_DATA_H_
#define _POINT_DATA_H_

#include <stdint.h>

#include <iostream>
#include <string>
#include <vector>
#include <list>

#define ANGLE_TO_RADIAN(angle) ((angle)*3141.59 / 180000)
#define RADIAN_TO_ANGLED(angle) ((angle)*180000 / 3141.59)

namespace ldlidar {

enum class LDType {
  STP_23,
  STP_23L,
  NO_VERSION
};

// STP-23 ---------------------------------------------------
#define POINT_PER_PACK 12
struct LaserPointDataType {
  int numbers;
  uint16_t temperature;  // measure temperature orgin ADC value  
  uint16_t distance[POINT_PER_PACK];   // Distance is measured in millimeters
  uint8_t intensity[POINT_PER_PACK];  // Intensity is 0 to 255
};

typedef std::vector<LaserPointDataType> LaserMeasurePointsVec;
typedef std::list<LaserPointDataType> LaserMeasurePointsList;

typedef struct __attribute__((packed)) {
  uint16_t distance;
  uint8_t intensity;
} LidarPointStructDef;

typedef struct __attribute__((packed)) {
  uint8_t header;
  uint8_t ver_len;
  uint16_t temperature;
  uint16_t start_angle;
  LidarPointStructDef point[POINT_PER_PACK];
  uint16_t end_angle;
  uint16_t timestamp;
  uint8_t crc8;
} LiDARFrameTypeDef;

// STP-23L --------------------------------------------------
typedef enum {
  PACK_GET_DISTANCE =   0x02,  /*Frame ID of distance data*/
  PACK_RESET_SYSTEM =   0x0D,  /*Frame ID of Reset sensor system*/
  PACK_STOP =           0x0F,  /*Frame ID of stop distance data transmission*/
  PACK_ACK =            0x10,  /*Frame ID of cmd ack frame*/
  PACK_VERSION =        0x14   /*Frame ID of get sensor manufacture informations*/
} PackageIDTypeDef; // TRNet protocol

#define MEASUREMENT_POINT_NUM  12
typedef struct __attribute__((packed)) {
  int16_t distance;      /*距离数据*/
  uint16_t noise;        /*环境噪声*/
  uint32_t peak;         /*接收强度值*/
  uint8_t  confidence;   /*置信度*/
  uint32_t intg;         /*积分次数*/
  int16_t  reftof;       /*温度表征值*/
} MeasurePointDataTypeDef;

typedef struct __attribute__((packed)) {
  MeasurePointDataTypeDef point[MEASUREMENT_POINT_NUM];
  uint32_t timestamp;
} MeasureFrameDataTypeDef;

struct LaserFrameDataType {
  int numbers;
  MeasurePointDataTypeDef points[MEASUREMENT_POINT_NUM];
  uint32_t timestamp;
};

typedef struct __attribute__((packed)) {
  uint8_t ack_send_cmd;
  uint8_t achieve_result;
} ACKFrameDataTypeDef;

////  lidar product manufacture informations
typedef struct __attribute__((packed)) {
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
} DataTypeDef;

typedef struct __attribute__((packed)) {
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t backup;
} TimeTypeDef;

typedef struct __attribute__((packed)) {
  uint32_t     firmware_version;
  uint32_t     hardware_version;
  DataTypeDef  date_t;
  TimeTypeDef  time_t;
  uint32_t     id1;
  uint32_t     id2;
  uint32_t     id3;
  uint8_t      sn[8];
  uint16_t     pitch_angle[4];
  uint16_t     blind_area[2];
  uint32_t     frequence;  
} LiDARManufactureInfoTypeDef;

typedef struct LidarDeviceVerInfo {
  std::string firmware_ver;
  std::string hardware_ver;
  std::string manufacture_times;
  std::string mcu_id;
  std::string sn;
  uint16_t    pitch_angle[4];
  uint16_t    blind_area_most_near;
  uint16_t    blind_area_most_far;
  uint32_t    frequence;  
} LidarDeviceVerInfoTypeDef;


} // namespace ldlidar

#endif  // _POINT_DATA_H_

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/