#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rmw_fastrtps_shared_cpp/custom_service_info.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("service_reply_test");
  using Service = rcl_interfaces::srv::GetParameters;
  auto service = node->create_service<Service>(
      "/reply_test", [](Service::Request::SharedPtr, Service::Response::SharedPtr) {});
  auto handle = rcl_service_get_rmw_handle(service->get_service_handle().get());
  auto info = static_cast<CustomServiceInfo *>(handle->data);
  const auto wait = info->response_writer_->get_qos().reliability().max_blocking_time;
  if (wait.seconds != 2 || wait.nanosec != 0) {
    std::cerr << "Service reply discovery wait was not loaded\n";
    rclcpp::shutdown();
    return 1;
  }
  for (int i = 0; i < 3; ++i) {
    auto client = node->create_client<Service>("/reply_test");
    if (!client->wait_for_service(std::chrono::seconds(3)))
      return 2;
    auto future = client->async_send_request(std::make_shared<Service::Request>());
    if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(3)) !=
        rclcpp::FutureReturnCode::SUCCESS)
      return 3;
  }
  std::cout << "Actual ROS service reply wait: 2 seconds; three new clients passed\n";
  rclcpp::shutdown();
}
