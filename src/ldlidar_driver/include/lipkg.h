/**
 * @file lipkg.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  LiDAR data protocol processing App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-10-28
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

#ifndef __LIPKG_H
#define __LIPKG_H

#include <math.h>

#include <chrono>
#include <algorithm>

#include "ldlidar_datatype.h"
#include "cmd_serial_interface_linux.h"
#include "trnet.h"

namespace ldlidar {

class LiPkg {
 public:
  LiPkg();
  ~LiPkg();
  // set product type (belong to enum class LDType)
  void SetProductType(LDType type_number);
  // get sdk version number
  std::string GetSdkVersionNumber(void);

  // Get lidar data ready flag  
  bool IsFrameReady(void);  
  // Lidar data readiness flag reset
  void ResetFrameReady(void);
  
  /**
   * @brief 获取STP23测量数据
  */
  bool GetLaserMeasureData(LaserPointDataType& dst);

  /**
   * @brief 获取STP23L测量数据
  */
  bool GetLaserMeasureData(LaserFrameDataType& dst);

  bool SendCmd(CmdSerialInterfaceLinux* port, uint8_t address, uint8_t id);

  bool IsDeviceInfoReady(void);

  void ResetDeviceInfoReay(void);

  LidarDeviceVerInfoTypeDef GetLidarDeviceVersionInfo(void);

  bool IsWorkStopReady(void);
  void ResetWorkStopReady(void);

  void CommReadCallback(const char *byte, size_t len);
  
 private:
  LDType product_type_;
  std::string sdk_pack_version_;
  uint16_t timestamp_;
  bool is_frame_ready_;
  bool is_deviceinfo_ready_;
  bool is_work_stop_;

  TRNet t;
  const TRData *tr_data = nullptr;

  LiDARFrameTypeDef pkg_;
  LaserMeasurePointsList tmp_measure_buf_;
  LaserPointDataType laser_measure_data_;
  LaserFrameDataType stp23l_measure_data_;

  std::mutex  mutex_lock1_;
  std::mutex  mutex_lock2_;

  LidarDeviceVerInfoTypeDef  device_ver_info_;
  std::mutex  mutex_lock3_;
  std::mutex  mutex_lock4_;

   // parse single packet
  bool AnalysisOne(uint8_t byte);
  bool Parse(const uint8_t* data, long len);  
  // combine stantard data into data frames and calibrate
  bool AssemblePacket();  

  bool AnalysisTRNetByte(uint8_t byte);
  bool ParseTRNetData(const uint8_t* data, long len);
  
  void SetFrameReady(void);
  /**
   * @brief  设置STP23测量数据
  */
  void SetLaserMeasureData(LaserPointDataType& src);

  /**
   * @brief  设置STP23L测量数据
  */
  void SetLaserMeasureData(LaserFrameDataType& src);

  void SetLidarDeviceVersionInfo(LidarDeviceVerInfoTypeDef& info);
  void SetDeviceInfoReady(void);
};

} // namespace ldlidar

#endif  //__LIPKG_H

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/