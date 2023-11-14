



//int N =9;


#include "mpc_acado.cpp"
#include "mpc_acado_node.h"


class ModelPredictiveControl : public rclcpp::Node
{
public:
  ModelPredictiveControl() : Node("model_predictive_control")
  {   
        
        //mpc_ptr = std::make_unique<Mpc>();

        //int N = mpc_ptr->getN();
        const int N = 9;
        current_state = Eigen::MatrixXd::Zero(9, 1);
        desired_state = Eigen::MatrixXd::Zero(N+1, 9);


      desired_control = Eigen::MatrixXd::Zero(N+1, 4);


    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);


    mpc_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
    vehicle_rates_setpoint_publisher_ = this->create_publisher<VehicleRatesSetpoint>("/fmu/in/vehicle_rates_setpoint", 10);


     position_subscriber_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos, std::bind(&ModelPredictiveControl::position_cb, this, std::placeholders::_1));





    offboard_setpoint_counter_ = 0;

    auto timer_callback = [this]() -> void {

      if (offboard_setpoint_counter_ == 10) {

        // Change to Offboard mode after 10 setpoints
        this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

        // Arm the vehicle
        this->arm();
      }
      //mpc_ptr.reset(new Mpc());
        
      publish_control_mode();

      //publish_trajectory_setpoint(); //for setpoint position control *PID*

      publish_rates_setpoint();
            
            //
      // stop the counter after reaching 11
      if (offboard_setpoint_counter_ < 11) {
        offboard_setpoint_counter_++;
      }
    };
    timer_ = this->create_wall_timer(2ms, timer_callback);
  }

  void arm();
  void disarm();

private:

  void position_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_;


    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr position_subscriber_;



  rclcpp::Publisher<OffboardControlMode>::SharedPtr mpc_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;




  std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

  uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

  void publish_control_mode();
  void publish_trajectory_setpoint();
  void publish_rates_setpoint();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

};



void ModelPredictiveControl::position_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
      
      //RCLCPP_INFO(this->get_logger(), "position received send");


  
    current_state(0,0) = msg->position[0];
    current_state(1,0) = msg->position[1];
    current_state(2,0) =  -msg->position[2];
    current_state(3,0) = msg->velocity[0];
    current_state(4,0) = msg->velocity[1];
    current_state(5,0) =  msg->velocity[2];

    
    tf2::Quaternion q(
      msg->q[0],
      msg->q[1],
      msg->q[2],
      msg->q[3]);

    tf2::Matrix3x3 m(q);
    m.getRPY(pitch, roll, yaw);

    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_state(6,0) = yaw;
    current_state(7,0) = pitch;
    current_state(8,0) = roll;



}





void ModelPredictiveControl::arm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

  RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void ModelPredictiveControl::disarm()
{
  publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

  RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void ModelPredictiveControl::publish_control_mode()
{
  OffboardControlMode msg{};
  msg.position = false;
  msg.velocity = false;
  msg.acceleration = false;
  msg.attitude = false;
  msg.body_rate = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  mpc_mode_publisher_->publish(msg);
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void ModelPredictiveControl::publish_rates_setpoint()
{    
    
     
    
     for (int row = 0; row < desired_state.rows(); ++row) {
          desired_state(row,2) = 1.0;
      }


     for (int row = 0; row < desired_control.rows(); ++row) {
          desired_control(row,0) = 0.5;
      }




  //VehicleRatesSetpoint msg{};
    VehicleRatesSetpoint msg{};
    


    //Eigen::MatrixXd desired_state = Eigen::MatrixXd::Zero(N+1, 9);

   // Eigen::MatrixXd desired_state1 = desired_state.transpose();

    Eigen::MatrixXd desired_state1 =   desired_state.transpose() ;

    Eigen::MatrixXd desired_control1 =   desired_control.transpose() ;


    rclcpp::Clock clock;
    rclcpp::Time solve_start = clock.now();
    
    
    //auto success = mpc_ptr->solve(current_state, desired_state1, desired_control1);


   // cout << "Current Z position node level: " << current_state(2,0) << endl;
    //cout << "Current X position node level: " << current_state(0,0) << endl;

    //cout << "mpc solve success" << endl;
  


    //auto traj = mpc_ptr->getPredictedTrajectory();

    rclcpp::Time solve_end = clock.now();
    //auto solve_time = solve_end - solve_start;
    //std::cout << "MPC solution time: " << solve_time.seconds() << " seconds" << std::endl;
    //auto control_cmd = mpc_ptr->getControlAction();

    //cout << "got cmd" << endl;
    //cout << "C1: Thrust:" << control_cmd[0] << endl;
    //cout << "C2: Roll_rate:" << control_cmd[1] << endl;
    //cout << "C3: Pitch_rate:" << control_cmd[2] << endl;
    //cout << "C4: Yaw_rate:" << control_cmd[3] << endl;

  //msg.timestamp = timestamp_.load();
  //msg.thrust_body[0] = 0.0;
  //msg.thrust_body[1] = 0.0;
    //std::cout << "Thrust to drone " << control_cmd[0]/30 << std::endl;
      
  //msg.thrust_body[2] = -control_cmd[0]/30.0;
  //msg.roll = control_cmd[1];
  //msg.pitch = control_cmd[2];
  //msg.yaw = control_cmd[3];

  vehicle_rates_setpoint_publisher_->publish(msg);

}


void ModelPredictiveControl::publish_trajectory_setpoint()
{
  TrajectorySetpoint msg{};
  msg.position = {0.0, 0.0, -5.0};
  msg.yaw = -3.14; // [-PI:PI]
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(msg);
}




/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */


void ModelPredictiveControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
  VehicleCommand msg{};
  msg.param1 = param1;
  msg.param2 = param2;
  msg.command = command;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.from_external = true;
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{



    //mpc_ptr.reset(new Mpc());
        
  std::cout << "MPC node..." << std::endl;

  std::cout << "Starting model predictive control node..." << std::endl;


  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ModelPredictiveControl>());
  rclcpp::shutdown();




  return 0;
}