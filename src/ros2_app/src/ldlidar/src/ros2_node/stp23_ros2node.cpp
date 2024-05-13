/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
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

#include "ros2_api.h"
#include "lipkg.h"

void  ToMessagePublish(uint16_t distance, LaserSingleMeasureSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr& lidarpub);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("ldlidar_published"); // create a ROS2 Node

  std::string product_name;
	std::string port_name;
  int serial_port_baudrate;
  LaserSingleMeasureSetting setting;
	setting.frame_id = "base_laser";
  
  // declare ros2 param
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", setting.topic_name);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<int>("port_baudrate", serial_port_baudrate);
  node->declare_parameter<std::string>("frame_id", setting.frame_id);

  // get ros2 param
  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", setting.topic_name);
  node->get_parameter("port_name", port_name);
  node->get_parameter("port_baudrate", serial_port_baudrate);
  node->get_parameter("frame_id", setting.frame_id);

  ldlidar::LiPkg *lidar_pkg = new ldlidar::LiPkg();
  ldlidar::CmdSerialInterfaceLinux *cmd_port = new ldlidar::CmdSerialInterfaceLinux();

  RCLCPP_INFO_STREAM(node->get_logger(), " [ldrobot] SDK Pack Version is " << lidar_pkg->GetSdkVersionNumber());
  RCLCPP_INFO(node->get_logger(), " [ldrobot] <product_name>: %s", product_name.c_str());
  RCLCPP_INFO(node->get_logger(), " [ldrobot] <topic_name>: %s", setting.topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), " [ldrobot] <port_name>: %s", port_name.c_str());
  RCLCPP_INFO(node->get_logger(), " [ldrobot] <port_baudrate>: %d", serial_port_baudrate);
  RCLCPP_INFO(node->get_logger(), " [ldrobot] <frame_id>: %s", setting.frame_id.c_str());

  if (port_name.empty()) {
    RCLCPP_ERROR(node->get_logger(), " [ldrobot] input <port_name> param is null");
    exit(EXIT_FAILURE);
  }

  lidar_pkg->SetProductType(ldlidar::LDType::STP_23);

  cmd_port->SetReadCallback(std::bind(&ldlidar::LiPkg::CommReadCallback, lidar_pkg, std::placeholders::_1, std::placeholders::_2));

  if (cmd_port->Open(port_name, (uint32_t)serial_port_baudrate)) {
    RCLCPP_INFO(node->get_logger(), " [ldrobot] open %s device %s success!", product_name.c_str(), port_name.c_str());
  }else {
    RCLCPP_ERROR(node->get_logger(), " [ldrobot] open %s device %s fail!", product_name.c_str(), port_name.c_str());
    exit(EXIT_FAILURE);
  }

  // create ldlidar data topic and publisher
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr publisher = 
    node->create_publisher<sensor_msgs::msg::Range>(setting.topic_name, 10);
  
  rclcpp::WallRate r(10); //10hz
  auto last_time = std::chrono::steady_clock::now();
  while (rclcpp::ok()) {
    if (lidar_pkg->IsFrameReady()) {
      lidar_pkg->ResetFrameReady();
      last_time = std::chrono::steady_clock::now();
      ldlidar::LaserPointDataType laserdata;
      if (lidar_pkg->GetLaserMeasureData(laserdata)) {
        for (int i = 0; i < laserdata.numbers; i++) {
          ToMessagePublish(laserdata.distance[i], setting, node, publisher);
          RCLCPP_INFO(node->get_logger(), "[ldrobot] Measure data: ");
          RCLCPP_INFO(node->get_logger(), "distance(mm): %d, intensity: %d", laserdata.distance[i], laserdata.intensity[i]);
          RCLCPP_INFO(node->get_logger(), "-------------------------------");
        }
      }
    }

    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-last_time).count() > 1000) { 
			RCLCPP_ERROR(node->get_logger(),"[ldrobot] publish data is time out, please check lidar device");
			exit(EXIT_FAILURE);
		}

    r.sleep();
  }

  cmd_port->Close();

  delete lidar_pkg;
  lidar_pkg = nullptr;
  delete cmd_port;
  cmd_port = nullptr;
  
  RCLCPP_INFO(node->get_logger(), "[ldrobot] this node of ldlidar_published is end");
  rclcpp::shutdown();

  return 0;
}

void  ToMessagePublish(uint16_t distance, LaserSingleMeasureSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr& lidarpub) {

  sensor_msgs::msg::Range output;
  output.header.stamp = node->now();
  output.header.frame_id = setting.frame_id;
  output.radiation_type = sensor_msgs::msg::Range::INFRARED;
  output.field_of_view = 0.1;
  output.min_range = 0;
  output.max_range = 12;
  output.range = distance / 1000.f; // unit is meter.
  lidarpub->publish(output);
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/