/**
 * @file lipkg.cpp
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

#include "lipkg.h"
#include "log_module.h"

namespace ldlidar {

static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8};

uint8_t CalCRC8(const uint8_t *data, uint16_t data_len) {
  uint8_t crc = 0;
  while (data_len--) {
    crc = CrcTable[(crc ^ *data) & 0xff];
    data++;
  }
  return crc;
}

LiPkg::LiPkg() : product_type_(LDType::NO_VERSION),
  sdk_pack_version_("v2.0.1"),
  timestamp_(0),
  is_frame_ready_(false),
  is_deviceinfo_ready_(false),
  is_work_stop_(false){

}

LiPkg::~LiPkg() {

}

void LiPkg::SetProductType(LDType type_number) {
  product_type_ = type_number;
  switch (type_number) {
    case LDType::STP_23:
    case LDType::STP_23L:
      break;
    default : 
      break;
  }
}

std::string LiPkg::GetSdkVersionNumber(void) {
  return sdk_pack_version_;
}

bool LiPkg::AnalysisOne(uint8_t byte) {
  static enum {
    HEADER,
    VER_LEN,
    DATA,
  } state = HEADER;
  static uint16_t count = 0;
  static uint8_t tmp[128] = {0};
  static uint16_t pkg_count = sizeof(LiDARFrameTypeDef);

  switch (state) {
    case HEADER:
      if (byte == 0x54) {
        tmp[count++] = byte;
        state = VER_LEN;
      }
      break;
    case VER_LEN:
      if ((byte&0x1F) == POINT_PER_PACK) {
          tmp[count++] = byte;
          state = DATA;
      } else {
        state = HEADER;
        count = 0;
        return false;
      }
      break;
    case DATA:
      tmp[count++] = byte;
      if (count >= pkg_count) {
        memcpy((uint8_t *)&pkg_, tmp, pkg_count);
        uint8_t crc = CalCRC8((uint8_t *)&pkg_, pkg_count - 1);
        state = HEADER;
        count = 0;
        if (crc == pkg_.crc8) {
          return true;
        } else {
          return false;
        }
      }
      break;
    default:
      break;
  }

  return false; 
}

bool LiPkg::Parse(const uint8_t *data, long len) {
  for (int i = 0; i < len; i++) {
    if (AnalysisOne(data[i])) {
      // parse a package is success
      timestamp_ = pkg_.timestamp; // In milliseconds
      LaserPointDataType pkg_data;
      pkg_data.numbers = POINT_PER_PACK;
      pkg_data.temperature = pkg_.temperature; // Temperature ADC value
      for (int i = 0; i < POINT_PER_PACK; i++) {
        pkg_data.distance[i] = pkg_.point[i].distance; // unit is mm
        pkg_data.intensity[i] = pkg_.point[i].intensity;
      }
      tmp_measure_buf_.push_back(pkg_data);
    }
  }

  return true;
}

bool LiPkg::AssemblePacket() {
  
  if (!tmp_measure_buf_.empty()) {
    LaserPointDataType data = tmp_measure_buf_.front();
    SetLaserMeasureData(data);
    SetFrameReady();
    tmp_measure_buf_.pop_front();
    return true;
  }

  return false;
}

bool LiPkg::AnalysisTRNetByte(uint8_t byte) {
  static enum {
    HEADER1,
    HEADER2,
    HEADER3,
    HEADER4,
    LENS,
    DATA,
  } state = HEADER1;
  static uint16_t count = 0;
  static uint8_t tmp[500] = {0};
  static uint16_t pkg_count = 0;

  switch (state) {
    case HEADER1:
      if (byte == 0xAA) {
        tmp[count++] = byte;
        state = HEADER2;
      }
      break;
    case HEADER2:
      if (byte == 0xAA) {
        tmp[count++] = byte;
        state = HEADER3;
      } else {
        state = HEADER1;
        count = 0;
      }
      break;
    case HEADER3:
      if (byte == 0xAA) {
        tmp[count++] = byte;
        state = HEADER4;
      } else {
        state = HEADER1;
        count = 0;
      }
      break;
    case HEADER4:
      if (byte == 0xAA) {
        tmp[count++] = byte;
        state = LENS;
      } else {
        state = HEADER1;
        count = 0;
      }
      break;
    case LENS:
      tmp[count++] = byte;
      if (count == 10) {
        uint16_t data_lens_val = ((tmp[9] << 8) | tmp[8]);
        if (data_lens_val > 320) {
          state = HEADER1;
          count = 0;
        } else {
          pkg_count = data_lens_val + 11;
          state = DATA;
        }
      }
      break;
    case DATA:
      tmp[count++] = byte;
      if (count >= pkg_count) {
        state = HEADER1;
        count = 0;
        tr_data = t.Unpack(tmp, pkg_count);
        if ((tr_data != nullptr) && tr_data->data_len) {
          return true;
        } else {
          return false;
        }
      }
      break;
    default:
      break;
  }

  return false; 
}

bool LiPkg::ParseTRNetData(const uint8_t* data, long len) {
  for (int i = 0; i < len; i++) {
    if (AnalysisTRNetByte(data[i])) {
      switch (tr_data->pack_id) {
        case PACK_GET_DISTANCE: {
          MeasureFrameDataTypeDef sensor_measure_data = *(MeasureFrameDataTypeDef *)(tr_data->data.data());
          LaserFrameDataType framedata;
          framedata.numbers = MEASUREMENT_POINT_NUM;
          for (int i = 0; i < MEASUREMENT_POINT_NUM; i++) {
            framedata.points[i] = sensor_measure_data.point[i];
          }
          framedata.timestamp = sensor_measure_data.timestamp;
          SetLaserMeasureData(framedata);
          SetFrameReady();
          break;
        }
        case PACK_VERSION: {
          LiDARManufactureInfoTypeDef device_info = *((LiDARManufactureInfoTypeDef *)(tr_data->data.data()));
          LidarDeviceVerInfoTypeDef device_info_str;
          //  frimware ver
          char val[100] = {0};
          snprintf(val, 100, "v%d.%d.%d", (device_info.firmware_version>>24), 
              (device_info.firmware_version>>16&0xff), (device_info.firmware_version&0xff));
          device_info_str.firmware_ver = val;
          memset(val, 0, sizeof(val)/sizeof(val[0]));
          //  harware ver
          snprintf(val, 100, "v%d.%d.%d", (device_info.hardware_version>>24), 
              (device_info.hardware_version>>16&0xff), (device_info.hardware_version&0xff));
          device_info_str.hardware_ver = val;
          memset(val, 0, sizeof(val)/sizeof(val[0]));
          // manufacture times
          snprintf(val, 100, "%d-%d-%d,%d:%d:%d", device_info.date_t.year,
              device_info.date_t.month,
              device_info.date_t.day,
              device_info.time_t.hour,
              device_info.time_t.minute,
              device_info.time_t.second);
          device_info_str.manufacture_times = val;
          memset(val, 0, sizeof(val)/sizeof(val[0]));
          // mcu id
          snprintf(val, 100, "%x%x%x", device_info.id3, device_info.id2, device_info.id1);
          device_info_str.mcu_id = val;
          memset(val, 0, sizeof(val)/sizeof(val[0]));
          // sn
          device_info_str.sn = (char *)device_info.sn;
          // pitch_angle
          device_info_str.pitch_angle[0] = device_info.pitch_angle[0];
          device_info_str.pitch_angle[1] = device_info.pitch_angle[1];
          device_info_str.pitch_angle[2] = device_info.pitch_angle[2];
          device_info_str.pitch_angle[3] = device_info.pitch_angle[3];
          // blind_area
          device_info_str.blind_area_most_near = device_info.blind_area[0];
          device_info_str.blind_area_most_far = device_info.blind_area[1];
          // frequence
          device_info_str.frequence = device_info.frequence;
          SetLidarDeviceVersionInfo(device_info_str);
          SetDeviceInfoReady();
          break;
        }
        case PACK_ACK: {
          ACKFrameDataTypeDef ackresult = *(ACKFrameDataTypeDef *)(tr_data->data.data());
          LDS_LOG_INFO("ACK,id:0x%x,result:%d", ackresult.ack_send_cmd, ackresult.achieve_result);
          if (PACK_STOP == ackresult.ack_send_cmd) {
            if (1 == ackresult.achieve_result) {
              is_work_stop_ = true;
            } else {
              is_work_stop_ = false;
            }
          }
          break;
        }
        default:
          break;
      }
    }
  }

  return true;
}

bool LiPkg::SendCmd(CmdSerialInterfaceLinux* port, uint8_t address, uint8_t id) {
  TRNet pkg;
  std::vector<uint8_t> out;
  TRData out_data;
  uint32_t len = 0;
  out_data.device_address = address;
  out_data.pack_id = id;
  out_data.chunk_offset = 0;
  pkg.Pack(out_data, out);
  // for (auto val : out) {
  //   printf("0x%x ", val);
  // }
  // printf("\r\n");
  bool ret = port->WriteToIo((const uint8_t *)out.data(), out.size(), &len);
  if (ret) {
    return true;
  } else {
    return false;
  }
}

bool LiPkg::IsFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  return is_frame_ready_; 
}  

void LiPkg::ResetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  is_frame_ready_ = false;
}

void LiPkg::SetFrameReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock1_);
  is_frame_ready_ = true;
}

bool LiPkg::GetLaserMeasureData(LaserPointDataType& dst) {
  if (this->product_type_ != LDType::STP_23) {
    return false;
  }
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  dst = laser_measure_data_;
  return true;
}

bool LiPkg::GetLaserMeasureData(LaserFrameDataType& dst){
  if (this->product_type_ != LDType::STP_23L) {
    return false;
  }
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  dst = stp23l_measure_data_;
  return true;
}

void LiPkg::SetLaserMeasureData(LaserPointDataType& src) {
  std::lock_guard<std::mutex> lg(mutex_lock2_);
  laser_measure_data_ = src;
}

void LiPkg::SetLaserMeasureData(LaserFrameDataType& src) {
  std::lock_guard<std::mutex> lg(mutex_lock2_); 
  stp23l_measure_data_ = src;
}

void LiPkg::CommReadCallback(const char *byte, size_t len) {
  if (LDType::STP_23 == this->product_type_) {
    if (this->Parse((uint8_t *)byte, len)) {
      this->AssemblePacket();
    }
  } else if (LDType::STP_23L == this->product_type_) {
    // this->UnpackTRNetData((uint8_t *)byte, len);
    this->ParseTRNetData((uint8_t *)byte, len);
  }
}

void LiPkg::SetLidarDeviceVersionInfo(LidarDeviceVerInfoTypeDef& info) {
  std::lock_guard<std::mutex> lg(mutex_lock3_);
  device_ver_info_ = info;
}

LidarDeviceVerInfoTypeDef LiPkg::GetLidarDeviceVersionInfo(void) {
  std::lock_guard<std::mutex> lg(mutex_lock3_);
  return device_ver_info_;
}

bool LiPkg::IsDeviceInfoReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock4_);
  return is_deviceinfo_ready_;
}

void LiPkg::ResetDeviceInfoReay(void) {
  std::lock_guard<std::mutex> lg(mutex_lock4_);
  is_deviceinfo_ready_ = false;
}

void LiPkg::SetDeviceInfoReady(void) {
  std::lock_guard<std::mutex> lg(mutex_lock4_);
  is_deviceinfo_ready_ = true;
}

bool LiPkg::IsWorkStopReady(void) {
  return is_work_stop_;
}

void LiPkg::ResetWorkStopReady(void) {
  is_work_stop_ = false;
}

} // namespace ldlidar
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/