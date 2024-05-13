/**
 * @file trnet.h
 * @author LDRobot (support@ldrobot.com)
 * @brief  comm trans data handle App
 *         This code is only applicable to LDROBOT LiDAR LD07 products
 * sold by Shenzhen LDROBOT Co., LTD
 * @version 0.1
 * @date 2021-11-14
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
#ifndef _TR_NET_H
#define _TR_NET_H

#include <stdint.h>

#include <vector>
#include <cstring>

#define THIS_DEVICE_ADDRESS 0x07 /*Device address*/

#define LIDAR_DEVICE_01 0x01    // b0001
#define LIDAR_DEVICE_02 0x02    // b0010
#define LIDAR_DEVICE_03 0x04    // b0100

namespace ldlidar {

struct TRData {
  uint8_t device_address;
  uint8_t pack_id;
  uint16_t chunk_offset;
  uint16_t data_len;
  std::vector<uint8_t> data;
};

class TRNet {
 public:
  TRNet();
  ~TRNet();
  const TRData *Unpack(const uint8_t *data, uint32_t len);
  bool Pack(const TRData &in, std::vector<uint8_t> &out);
  bool FindLeadingCode(const uint8_t *buff);  // make sure buffer size bigger than 4
  uint32_t GetParseDataLen();

 protected:
  TRData tr_data_;

 private:
  const uint32_t LEADING_CODE = 0xAAAAAAAA;
  const uint32_t HEADER_LEN = 4;  // device_address , pack_id , chunk_offset len
  const uint32_t EXTRA_LEN = 11;
  uint32_t parse_data_len_;
  uint8_t CalCheckSum(const uint8_t *data, uint16_t len);
};

} // namespace ldlidar

#endif  // _TR_NET_H
/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/