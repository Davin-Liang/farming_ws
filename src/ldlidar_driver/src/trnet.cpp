/**
 * @file trnet.cpp
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
#include "trnet.h"

namespace ldlidar {

TRNet::TRNet() : parse_data_len_(0) {

}

TRNet::~TRNet() {
  
}

uint32_t TRNet::GetParseDataLen() {
  return parse_data_len_; 
}

uint8_t TRNet::CalCheckSum(const uint8_t *data, uint16_t len) {
  uint8_t checksum = 0;
  std::size_t i = 0;

  for (i = 0; i < len; i++) {
    checksum += *data++;
  }

  return checksum;
}

const TRData *TRNet::Unpack(const uint8_t *data, uint32_t len) {
  if (data == nullptr || len < EXTRA_LEN) {
    return nullptr;
  }
  const uint8_t *p = data;
  uint32_t code = *(uint32_t *)data;
  if (code != LEADING_CODE) {
    return nullptr;
  }
  p += 8;
  uint16_t data_len = *(uint16_t *)p;

  if (data_len > (len - EXTRA_LEN)) {
    return nullptr;
  }

  p += 2;

  uint8_t checksum = CalCheckSum(data + 4, 6 + data_len);

  p += data_len;

  if (checksum == *p) {
    p = data;
    p += 4;
    tr_data_.device_address = *p++;
    tr_data_.pack_id = *p++;
    tr_data_.chunk_offset = *(uint16_t *)p;
    p += 2;
    if (data_len > 0) {
      if (tr_data_.data.size() < data_len) tr_data_.data.resize(data_len);
      p += 2;
      std::memcpy(tr_data_.data.data(), p, data_len);
    }
    tr_data_.data_len = data_len;
    parse_data_len_ = data_len + EXTRA_LEN;
    return &tr_data_;
  }

  return nullptr;
}

bool TRNet::Pack(const TRData &in, std::vector<uint8_t> &out) {
  out.resize(EXTRA_LEN + in.data.size());
  uint8_t *p = out.data();
  *(uint32_t *)p = LEADING_CODE;
  p += 4;
  *p++ = in.device_address;
  *p++ = in.pack_id;
  *(uint16_t *)p = in.chunk_offset;
  p += 2;
  *(uint16_t *)p = in.data.size();
  p += 2;
  std::memcpy(p, in.data.data(), in.data.size());
  uint8_t checksum = CalCheckSum(out.data() + 4, out.size() - 5);
  out.back() = checksum;
  return true;
}

bool TRNet::FindLeadingCode(const uint8_t *buff) {
  uint32_t code = *(uint32_t *)buff;
  return (code == LEADING_CODE);
}

} // namespace ldlidar 

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/