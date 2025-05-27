# Autoware Universe ROS 2 API Usage Guide

This guide provides examples and best practices for using the ROS 2 API interfaces in Autoware Universe.

## Working with Messages

### Subscribing to Topics

```cpp
// C++ example
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    // Subscribe to a pose topic
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "input/pose", 10, 
      std::bind(&MyNode::on_pose, this, std::placeholders::_1));
  }

private:
  void on_pose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received pose: x=%f, y=%f, z=%f", 
                msg->pose.position.x, 
                msg->pose.position.y, 
                msg->pose.position.z);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};
```

```python
# Python example
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Subscribe to a pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped, 'input/pose', self.on_pose, 10)
            
    def on_pose(self, msg):
        self.get_logger().info(f"Received pose: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Publishing Messages

```cpp
// C++ example
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    // Create a publisher
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("output/pose", 10);
    
    // Publish at 10 Hz
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&MyNode::publish_pose, this));
  }

private:
  void publish_pose()
  {
    auto msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "base_link";
    
    // Set pose values
    msg->pose.position.x = 1.0;
    msg->pose.position.y = 2.0;
    msg->pose.position.z = 0.0;
    msg->pose.orientation.w = 1.0;
    
    pose_pub_->publish(std::move(msg));
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
```

```python
# Python example
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        
        # Create a publisher
        self.pose_pub = self.create_publisher(
            PoseStamped, 'output/pose', 10)
            
        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_pose)
            
    def publish_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        
        # Set pose values
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        
        self.pose_pub.publish(msg)

def main():
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with Services

### Creating a Service Server

```cpp
// C++ example
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    // Create a service
    service_ = create_service<std_srvs::srv::Trigger>(
      "trigger_service", 
      std::bind(&MyNode::handle_service, this, 
                std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handle_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Service called");
    response->success = true;
    response->message = "Service executed successfully";
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};
```

### Calling a Service

```cpp
// C++ example
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class MyNode : public rclcpp::Node
{
public:
  MyNode() : Node("my_node")
  {
    // Create a client
    client_ = create_client<std_srvs::srv::Trigger>("trigger_service");
    
    // Wait for service
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service");
        return;
      }
      RCLCPP_INFO(get_logger(), "Waiting for service...");
    }
    
    // Send request
    send_request();
  }

private:
  void send_request()
  {
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    
    auto callback = [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
      auto response = future.get();
      RCLCPP_INFO(get_logger(), "Service response: %s", 
                  response->success ? "true" : "false");
      RCLCPP_INFO(get_logger(), "Message: %s", response->message.c_str());
    };
    
    client_->async_send_request(request, callback);
  }

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};
```

## Working with Actions

### Creating an Action Server

```cpp
// C++ example
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

class MyNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

  MyNode() : Node("my_node")
  {
    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "follow_joint_trajectory",
      std::bind(&MyNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&MyNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&MyNode::handle_accepted, this, std::placeholders::_1));
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    // Execute the goal
    std::thread{std::bind(&MyNode::execute, this, std::placeholders::_1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    RCLCPP_INFO(get_logger(), "Executing goal");
    
    // Get the goal
    const auto goal = goal_handle->get_goal();
    
    // Feedback
    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    feedback->joint_names = goal->trajectory.joint_names;
    
    // Publish feedback periodically
    for (int i = 0; i < 10 && rclcpp::ok(); ++i) {
      // Check if the goal was canceled
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(std::make_shared<FollowJointTrajectory::Result>());
        RCLCPP_INFO(get_logger(), "Goal canceled");
        return;
      }
      
      // Update feedback
      feedback->actual.positions.clear();
      feedback->actual.positions.push_back(static_cast<double>(i) * 0.1);
      
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(get_logger(), "Publish feedback");
      
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    // Set the result
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
    
    // Succeed the goal
    goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Goal succeeded");
  }

  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
};
```

### Calling an Action

```cpp
// C++ example
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>

class MyNode : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  MyNode() : Node("my_node")
  {
    action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "follow_joint_trajectory");

    // Wait for server
    while (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for action server");
        return;
      }
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }
    
    // Send goal
    send_goal();
  }

private:
  void send_goal()
  {
    // Create goal
    auto goal = FollowJointTrajectory::Goal();
    
    // Set trajectory points
    goal.trajectory.joint_names.push_back("joint1");
    
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.push_back(1.0);
    point.time_from_start = rclcpp::Duration(1, 0);
    goal.trajectory.points.push_back(point);
    
    // Send goal
    auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
    
    // Goal response callback
    send_goal_options.goal_response_callback =
      [this](std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> future) {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(get_logger(), "Goal was rejected");
        } else {
          RCLCPP_INFO(get_logger(), "Goal accepted");
        }
      };
    
    // Feedback callback
    send_goal_options.feedback_callback =
      [this](GoalHandleFollowJointTrajectory::SharedPtr,
             const std::shared_ptr<const FollowJointTrajectory::Feedback> feedback) {
        RCLCPP_INFO(get_logger(), "Got feedback: %f", feedback->actual.positions[0]);
      };
    
    // Result callback
    send_goal_options.result_callback =
      [this](const GoalHandleFollowJointTrajectory::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(get_logger(), "Goal succeeded");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(get_logger(), "Goal was aborted");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(get_logger(), "Goal was canceled");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code");
            break;
        }
      };
    
    auto future = action_client_->async_send_goal(goal, send_goal_options);
  }

  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
};
```

## Best Practices

### Interface Naming Conventions

- **Topics**: Use namespaced hierarchical naming, e.g., `/domain/subdomain/topic_name`
- **Services**: Use verbs to describe actions, e.g., `/get_parameters`, `/set_mode`
- **Actions**: Use verbs in present tense for ongoing tasks, e.g., `/navigate_to_pose`, `/follow_path`

### QoS Settings

Choose appropriate Quality of Service (QoS) settings based on your requirements:

```cpp
// Reliable communication (for critical data)
rclcpp::QoS qos_reliable(10);
qos_reliable.reliable();

// Best effort communication (for high-frequency data)
rclcpp::QoS qos_sensor(10);
qos_sensor.best_effort();

// Keep last communication (retain only the latest messages)
rclcpp::QoS qos_keep_last(rclcpp::KeepLast(5));

// Transient local (useful for late joiners)
rclcpp::QoS qos_transient_local(10);
qos_transient_local.transient_local();
```

### Error Handling

Always implement proper error handling in your code:

```cpp
try {
  // Code that might throw
} catch (const rclcpp::exceptions::RCLError &e) {
  RCLCPP_ERROR(get_logger(), "RCL error: %s", e.what());
} catch (const std::exception &e) {
  RCLCPP_ERROR(get_logger(), "Exception: %s", e.what());
} catch (...) {
  RCLCPP_ERROR(get_logger(), "Unknown exception");
}
```

### Resource Management

Avoid resource leaks by properly managing memory and resources:

- Use `std::unique_ptr` and `std::shared_ptr` for automatic memory management
- Close file handles and release resources in the destructor
- Use RAII (Resource Acquisition Is Initialization) pattern

## References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/)
- [ROS 2 Design Patterns](https://design.ros2.org/)