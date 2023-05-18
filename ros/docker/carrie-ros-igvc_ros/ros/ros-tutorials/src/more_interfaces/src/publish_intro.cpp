#include <chrono>
#include <complex>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "more_interfaces/msg/intro.hpp"

using namespace std::chrono_literals;

class Introducer : public rclcpp::Node
{
public:
  Introducer()
  : Node("introducer")
  {
    introducer_ =
      this->create_publisher<more_interfaces::msg::Intro>("introducer", 10);

    auto publish_msg = [this]() -> void {
        auto message = more_interfaces::msg::Intro();

        message.name = "cat (me!)";
        message.gender.real = 0.9;
        message.gender.imag = 0.2;
        message.age = 22;

        std::cout << "Introducing:\n"
                  << "\tname: " << message.name << '\n'
                  << "\tgender: " << std::complex{
                      message.gender.real,
                      message.gender.imag
                  } << '\n';

        this->introducer_->publish(message);
      };
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<more_interfaces::msg::Intro>::SharedPtr introducer_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Introducer>());
  rclcpp::shutdown();

  return 0;
}
