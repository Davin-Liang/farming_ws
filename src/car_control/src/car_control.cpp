#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <cmath>

using namespace std::chrono_literals;

class PID {
public:
    PID(double Kp, double Ki, double Kd, double max_out, double max_iout)
        : Kp_(Kp), Ki_(Ki), Kd_(Kd), max_out_(max_out), max_iout_(max_iout),
          Pout_(0.0), Iout_(0.0), Dout_(0.0), out_(0.0) 
    {
        std::fill(std::begin(error_), std::end(error_), 0.0);
        std::fill(std::begin(Dbuf_), std::end(Dbuf_), 0.0);
    }

    void pid_calculate(double ref, double goal) 
    {
        error_[2] = error_[1];
        error_[1] = error_[0];
        error_[0] = goal - ref;

        Pout_ = Kp_ * error_[0];
        Iout_ += Ki_ * error_[0];
        Dbuf_[2] = Dbuf_[1];
        Dbuf_[1] = Dbuf_[0];
        Dbuf_[0] = error_[0] - error_[1];
        Dout_ = Kd_ * Dbuf_[0];

        limit_max_iout();
        out_ = Pout_ + Iout_ + Dout_;
        limit_max_out();
    }

    double get_output() const 
    {
        return out_;
    }

private:
    void limit_max_iout() 
    {
        if (Iout_ > max_iout_) 
        {
            Iout_ = max_iout_;
        } 
        else if (Iout_ < -max_iout_) 
        {
            Iout_ = -max_iout_;
        }
    }

    void limit_max_out() 
    {
        if (out_ > max_out_) 
        {
            out_ = max_out_;
        } 
        else if (out_ < -max_out_) 
        {
            out_ = -max_out_;
        }
    }

    double Kp_, Ki_, Kd_, max_out_, max_iout_;
    double Pout_, Iout_, Dout_, out_;
    double error_[3];
    double Dbuf_[3];
};

class CarController : public rclcpp::Node 
{
public:
    CarController(const std::string& name)
        : Node(name),
          start_for_lidar_distance_(false),
          start_for_pid_distance_(false),
          distance_tolerance_(0.03),
          odom_linear_scale_correction_(1.0),
          lidar_threthold_(0.1),
          liear_speed_(0.5),
          ori_angle_pid_(0.685, 0.00079, 0.426, 0.9, 0.0085),
          distance_pid_(0.42, 0.0, 0.08, 0.85, 0.0),
          yaw_angle_(0.0) 
    {

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);
        finish_task_pub_ = this->create_publisher<std_msgs::msg::Bool>("/finish_task", 5);
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::Range>("/laser", 10, std::bind(&CarController::lidar_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&CarController::odom_callback, this, std::placeholders::_1));
        yaw_angle_sub_ = this->create_subscription<std_msgs::msg::Float64>("/yaw_angle", 10, std::bind(&CarController::yaw_angle_callback, this, std::placeholders::_1));
        car_command_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/car_commands", 100, std::bind(&CarController::car_command_callback, this, std::placeholders::_1));

        work_timer_ = this->create_wall_timer(4ms, std::bind(&CarController::timer_work, this));

        std::cout << "初始化完成!!!!!!" << std::endl;
    }

private:
    void car_command_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data[0] == 0.0) // 代表距离
        {
            distance_ = msg->data[1];
            start_for_pid_distance_ = true;
            std::cout << "open distance control !!!!!!" << std::endl;
        }
        else if (msg->data[0] == 1.0) // 代表激光
        {
            start_delay_ = true;
            liear_speed_ = msg->data[1];
            lidar_threthold_ = msg->data[2];
            start_for_lidar_distance_ = true;
            std::cout << "open lidar control !!!!!!" << std::endl;
        }
        else if (msg->data[0] == 2.0) // 代表角度
        {
            angle_ = msg->data[1];
            std::cout << "open angle control !!!!!!" << std::endl;
        }
    }

    void yaw_angle_callback(const std_msgs::msg::Float64::SharedPtr msg) 
    {
        yaw_angle_ = msg->data;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) 
    {
        position_.x = msg->pose.pose.position.x;
        position_.y = msg->pose.pose.position.y;
    }

    void lidar_callback(const sensor_msgs::msg::Range::SharedPtr msg) 
    {
        lidar_distance_ = msg->range;
    }

    void timer_work() 
    {
        // orientation control
        ori_angle_pid_.pid_calculate(yaw_angle_, angle_);
        move_cmd_.angular.z = -ori_angle_pid_.get_output();

        // distance control
        if (start_for_pid_distance_) 
        {
            double o_distance = odom_linear_scale_correction_ * get_O_distance();
            distance_pid_.pid_calculate(o_distance, std::abs(distance_));
            move_cmd_.linear.x = copysign(distance_pid_.get_output(), distance_);
            if (std::abs(o_distance - std::abs(distance_)) < distance_tolerance_) 
            {
                start_for_pid_distance_ = false;
                auto message = std_msgs::msg::Bool();
                message.data = true; 
                finish_task_pub_->publish(message);
                std::cout << "完成距离控制!!!!!!" << std::endl;
            }
        } 
        else if (start_for_lidar_distance_) 
        {
            if (start_delay_) 
            {
                move_cmd_.linear.x = -liear_speed_;
                cmd_vel_pub_->publish(move_cmd_);
                std::this_thread::sleep_for(std::chrono::milliseconds(2500));
                start_delay_ = false;
            }
            move_cmd_.linear.x = -liear_speed_;
            if (lidar_distance_ < lidar_threthold_) 
            {
                start_for_lidar_distance_ = false;
                auto message = std_msgs::msg::Bool();
                message.data = true; 
                finish_task_pub_->publish(message);
                cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
                std::cout << "完成激光控制!!!!!!" << std::endl;
            }
        } 
        else 
        {
            move_cmd_.linear.x = 0.0;
            x_start_ = position_.x;
            y_start_ = position_.y;
        }
        cmd_vel_pub_->publish(move_cmd_);
    }

    double get_O_distance() const 
    {
        return sqrt(pow((position_.x - x_start_), 2) + pow((position_.y - y_start_), 2));
    }

    // Variables
    bool start_for_lidar_distance_;
    bool start_for_pid_distance_;
    bool start_delay_ = false;

    double distance_tolerance_;
    double odom_linear_scale_correction_;
    double lidar_threthold_;
    double liear_speed_;

    double distance_ = 0.0;
    double angle_ = 0.0;
    double yaw_angle_ = 0.0;
    double lidar_distance_ = 0.0;

    geometry_msgs::msg::Twist move_cmd_;
    geometry_msgs::msg::Point position_;
    double x_start_ = 0.0;
    double y_start_ = 0.0;

    PID ori_angle_pid_;
    PID distance_pid_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finish_task_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr car_command_sub_;

    rclcpp::TimerBase::SharedPtr work_timer_;
};

int main(int argc, char* argv[]) 
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CarController>("Car_Controller");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
