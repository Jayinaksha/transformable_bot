#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sdf/Element.hh>
#include <chrono>

class ServoYPlugin : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr position_sub_;
  
  std::string arm_name_;
  std::string joint_name_;
  std::string position_topic_;
  
  gz::sim::Entity joint_entity_;
  bool joint_found_ = false;
  bool components_initialized_ = false;
  
  // Control parameters
  double target_position_ = 0.0;
  double current_position_ = 0.0;
  double current_velocity_ = 0.0;
  double kp_ = 1000.0;
  double kd_ = 100.0;
  double max_force_ = 100.0;
  
  // PID state
  double last_error_ = 0.0;
  double last_time_ = 0.0;
  
  // Position limits (safety)
  double min_position_ = -1.57; // -90 degrees
  double max_position_ = 1.57;  // +90 degrees
  
  static int instance_counter_;
  int my_instance_id_;
  int search_attempts_ = 0;
  const int MAX_SEARCH_ATTEMPTS = 100;
  
  // Timing
  std::chrono::steady_clock::time_point last_search_time_;
  const std::chrono::milliseconds SEARCH_INTERVAL{100};

public:
  ServoYPlugin() {
    my_instance_id_ = ++instance_counter_;
    last_search_time_ = std::chrono::steady_clock::now();
  }

  void Configure(const gz::sim::Entity &/*entity*/,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &/*ecm*/,
                 gz::sim::EventManager &/*eventMgr*/) override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    
    // Read arm name from SDF
    if (sdf && sdf->HasElement("arm_name")) {
      arm_name_ = sdf->Get<std::string>("arm_name");
      RCLCPP_INFO_STREAM(rclcpp::get_logger("servo_y_temp"), "✅ Read arm_name from SDF: '" << arm_name_ << "'");
    } else {
      arm_name_ = "arm_" + std::to_string(my_instance_id_);
      RCLCPP_WARN_STREAM(rclcpp::get_logger("servo_y_temp"), "⚠️ No arm_name in SDF, defaulting to: '" << arm_name_ << "'");
    }
    
    std::string node_name = "servo_y_" + arm_name_;
    node_ = rclcpp::Node::make_shared(node_name);
    
    // CORRECTED: Use the actual joint naming convention
    joint_name_ = arm_name_ + "_y_servo_to_arm";
    position_topic_ = "/robot/" + arm_name_ + "/y_servo/position";
    
    // Read SDF parameters with defaults
    if (sdf && sdf->HasElement("joint_name")) {
      joint_name_ = sdf->Get<std::string>("joint_name");
    }
    if (sdf && sdf->HasElement("position_topic")) {
      position_topic_ = sdf->Get<std::string>("position_topic");
    }
    if (sdf && sdf->HasElement("kp")) {
      kp_ = sdf->Get<double>("kp");
    }
    if (sdf && sdf->HasElement("kd")) {
      kd_ = sdf->Get<double>("kd");
    }
    if (sdf && sdf->HasElement("max_force")) {
      max_force_ = sdf->Get<double>("max_force");
    }
    
    // Create ROS subscriber
    position_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
      position_topic_, 10,
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        // Apply position limits for safety
        double requested_position = msg->data;
        target_position_ = std::max(min_position_, std::min(max_position_, requested_position));
        
        if (requested_position != target_position_) {
          RCLCPP_WARN(node_->get_logger(), 
                     "🛡️ %s Y-Servo: Position limited from %.2f to %.2f rad", 
                     arm_name_.c_str(), requested_position, target_position_);
        }
        
        RCLCPP_INFO(node_->get_logger(), 
                   "🎯 %s Y-Servo target: %.3f rad (%.1f°)", 
                   arm_name_.c_str(), target_position_, target_position_ * 180.0 / M_PI);
      });
    
    RCLCPP_INFO(node_->get_logger(), 
                "🔧 Y-Servo Plugin loaded for %s", arm_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), 
                "   Joint: %s", joint_name_.c_str());
    RCLCPP_INFO(node_->get_logger(), 
                "   Position Topic: %s", position_topic_.c_str());
    RCLCPP_INFO(node_->get_logger(), 
                "   PID: Kp=%.1f, Kd=%.1f, MaxForce=%.1f", 
                kp_, kd_, max_force_);
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override
  {
    if (rclcpp::ok()) {
      rclcpp::spin_some(node_);
    }
    
    // Search for joint if not found yet
    if (!joint_found_) {
      auto now = std::chrono::steady_clock::now();
      if (now - last_search_time_ >= SEARCH_INTERVAL) {
        findJoint(ecm);
        last_search_time_ = now;
      }
      return; // Don't proceed until joint is found
    }
    
    // Initialize components if joint found but not initialized
    if (joint_found_ && !components_initialized_) {
      initializeJointComponents(ecm);
      return; // Wait one cycle for components to be ready
    }
    
    // Control the servo only if everything is ready
    if (joint_found_ && components_initialized_) {
      controlServo(info, ecm);
    }
  }

private:
  void findJoint(gz::sim::EntityComponentManager &ecm)
  {
    search_attempts_++;
    
    if (search_attempts_ > MAX_SEARCH_ATTEMPTS) {
      RCLCPP_ERROR(node_->get_logger(), 
                   "❌ %s: Y-Servo joint '%s' search FAILED after %d attempts", 
                   arm_name_.c_str(), joint_name_.c_str(), MAX_SEARCH_ATTEMPTS);
      printAllJointsForDebugging(ecm);
      return;
    }
    
    auto joints = ecm.EntitiesByComponents(gz::sim::components::Joint());
    
    for (auto joint : joints) {
      auto nameComp = ecm.Component<gz::sim::components::Name>(joint);
      if (nameComp && nameComp->Data() == joint_name_) {
        joint_entity_ = joint;
        joint_found_ = true;
        
        RCLCPP_INFO(node_->get_logger(), 
                   "🎯 %s: Found Y-servo joint '%s'", 
                   arm_name_.c_str(), joint_name_.c_str());
        
        return;
      }
    }
    
    // Debug output every 20 attempts
    if (search_attempts_ % 20 == 1) {
      RCLCPP_WARN(node_->get_logger(), 
                  "⚠️ %s: Searching for Y-servo joint '%s' (attempt %d/%d)", 
                  arm_name_.c_str(), joint_name_.c_str(), search_attempts_, MAX_SEARCH_ATTEMPTS);
    }
  }
  
  void initializeJointComponents(gz::sim::EntityComponentManager &ecm)
  {
    RCLCPP_INFO(node_->get_logger(), 
               "🔧 %s: Initializing joint components for Y-servo...", 
               arm_name_.c_str());
    
    // Create position component if it doesn't exist
    if (!ecm.Component<gz::sim::components::JointPosition>(joint_entity_)) {
      ecm.CreateComponent(joint_entity_, gz::sim::components::JointPosition());
      RCLCPP_DEBUG(node_->get_logger(), "   Created JointPosition component");
    }
    
    // Create velocity component if it doesn't exist
    if (!ecm.Component<gz::sim::components::JointVelocity>(joint_entity_)) {
      ecm.CreateComponent(joint_entity_, gz::sim::components::JointVelocity());
      RCLCPP_DEBUG(node_->get_logger(), "   Created JointVelocity component");
    }
    
    // CRITICAL FIX: Create JointForceCmd with proper size
    auto forceCmdComp = ecm.Component<gz::sim::components::JointForceCmd>(joint_entity_);
    if (!forceCmdComp) {
      // Create with proper size for 1 DOF joint
      std::vector<double> initialForce = {0.0};
      ecm.CreateComponent(joint_entity_, gz::sim::components::JointForceCmd(initialForce));
      RCLCPP_INFO(node_->get_logger(), "   ✅ Created JointForceCmd component with 1 DOF");
    } else {
      // Check if existing component has wrong size and fix it
      if (forceCmdComp->Data().size() != 1) {
        RCLCPP_WARN(node_->get_logger(), 
                   "   🔧 Fixing JointForceCmd size from %zu to 1", 
                   forceCmdComp->Data().size());
        std::vector<double> correctedForce = {0.0};
        ecm.SetComponentData<gz::sim::components::JointForceCmd>(joint_entity_, correctedForce);
      }
    }
    
    components_initialized_ = true;
    RCLCPP_INFO(node_->get_logger(), 
               "✅ %s: Joint components initialized successfully", 
               arm_name_.c_str());
  }
  
  void printAllJointsForDebugging(gz::sim::EntityComponentManager &ecm)
  {
    RCLCPP_ERROR(node_->get_logger(), "=== 🔍 ALL JOINTS FOR %s Y-SERVO DEBUG ===", arm_name_.c_str());
    auto joints = ecm.EntitiesByComponents(gz::sim::components::Joint());
    
    std::vector<std::string> all_joints;
    for (auto joint : joints) {
      auto nameComp = ecm.Component<gz::sim::components::Name>(joint);
      if (nameComp) {
        all_joints.push_back(nameComp->Data());
      }
    }
    
    std::sort(all_joints.begin(), all_joints.end());
    
    RCLCPP_ERROR(node_->get_logger(), "📋 ALL JOINTS FOUND (%zu total):", all_joints.size());
    for (const auto& joint : all_joints) {
      if (joint.find(arm_name_) != std::string::npos) {
        RCLCPP_ERROR(node_->get_logger(), "  🎯 %s (ARM RELATED)", joint.c_str());
      } else if (joint.find("servo") != std::string::npos) {
        RCLCPP_ERROR(node_->get_logger(), "  🔧 %s (SERVO)", joint.c_str());
      } else {
        RCLCPP_ERROR(node_->get_logger(), "  - %s", joint.c_str());
      }
    }
    RCLCPP_ERROR(node_->get_logger(), "   Expected joint name: '%s'", joint_name_.c_str());
    RCLCPP_ERROR(node_->get_logger(), "=== 🔍 END JOINT DEBUG ===");
  }
  
  void controlServo(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm)
  {
    // Get current position and velocity
    auto posComp = ecm.Component<gz::sim::components::JointPosition>(joint_entity_);
    auto velComp = ecm.Component<gz::sim::components::JointVelocity>(joint_entity_);
    auto forceCmdComp = ecm.Component<gz::sim::components::JointForceCmd>(joint_entity_);
    
    if (!posComp || !velComp || !forceCmdComp) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "⚠️ %s: Missing joint components", arm_name_.c_str());
      return;
    }
    
    if (posComp->Data().empty() || velComp->Data().empty()) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "⚠️ %s: Joint components have empty data", arm_name_.c_str());
      return;
    }
    
    // Verify force command component has correct size
    if (forceCmdComp->Data().size() != 1) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "⚠️ %s: JointForceCmd has wrong size (%zu), expected 1", 
                           arm_name_.c_str(), forceCmdComp->Data().size());
      return;
    }
    
    current_position_ = posComp->Data()[0];
    current_velocity_ = velComp->Data()[0];
    
    // Calculate PD control
    double current_time = std::chrono::duration<double>(info.simTime).count();
    double dt = current_time - last_time_;
    
    if (dt <= 0.0) {
      return; // Skip if no time has passed
    }
    
    double error = target_position_ - current_position_;
    
    // PD control
    double proportional = kp_ * error;
    double derivative = -kd_ * current_velocity_;
    double force = proportional + derivative;
    
    // Apply force limits
    force = std::max(-max_force_, std::min(max_force_, force));
    
    // CRITICAL FIX: Set force command with proper vector size
    std::vector<double> force_cmd = {force};
    ecm.SetComponentData<gz::sim::components::JointForceCmd>(joint_entity_, force_cmd);
    
    // Update state
    last_error_ = error;
    last_time_ = current_time;
    
    // Debug output (every 100 cycles to avoid spam)
    static int debug_counter = 0;
    if (++debug_counter % 100 == 0 && std::abs(error) > 0.01) {
      RCLCPP_DEBUG(node_->get_logger(), 
                  "🎛️ %s Y-Servo: pos=%.3f, target=%.3f, error=%.3f, force=%.1f", 
                  arm_name_.c_str(), current_position_, target_position_, error, force);
    }
  }
};

int ServoYPlugin::instance_counter_ = 0;

// Register the plugin
GZ_ADD_PLUGIN(ServoYPlugin,
              gz::sim::System,
              ServoYPlugin::ISystemConfigure,
              ServoYPlugin::ISystemPreUpdate)

// Plugin aliases
GZ_ADD_PLUGIN_ALIAS(ServoYPlugin, "servo_y_plugin")
GZ_ADD_PLUGIN_ALIAS(ServoYPlugin, "front_right_servo_y")
GZ_ADD_PLUGIN_ALIAS(ServoYPlugin, "front_left_servo_y")
GZ_ADD_PLUGIN_ALIAS(ServoYPlugin, "back_right_servo_y") 
GZ_ADD_PLUGIN_ALIAS(ServoYPlugin, "back_left_servo_y")