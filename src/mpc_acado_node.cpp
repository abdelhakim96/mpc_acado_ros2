



//int N =9;


#include "mpc_acado.cpp"
#include "mpc_acado_node.h"
#include "mpc_acado.h"

float ii=0.0;

class ModelPredictiveControl : public rclcpp::Node
{

      
public:
  NMPC* nmpc_ptr_;
  //ModelPredictiveControl() : Node("model_predictive_control")
  ModelPredictiveControl(NMPC* nmpc_ptr) : Node("model_predictive_control"), nmpc_ptr_(nmpc_ptr)
  {   
        
       


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

      publish_control();
            
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
  rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr vehicle_rates_setpoint_publisher_;

private:

  void position_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_;


    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr position_subscriber_;



  rclcpp::Publisher<OffboardControlMode>::SharedPtr mpc_mode_publisher_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;




  std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

  uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

  void publish_control_mode();
  void publish_trajectory_setpoint();
  void publish_control();
  void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

};



void ModelPredictiveControl::position_cb(const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
      
      //RCLCPP_INFO(this->get_logger(), "position received send");

    tf2::Quaternion q(
      msg->q[0],
      msg->q[1],
      msg->q[2],
      msg->q[3]);

    tf2::Matrix3x3 m(q);
    m.getRPY(pitch, roll, yaw);

    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);



  //current_states = { msg->position[0], 
  //                    msg->position[0],
  //                    -msg->position[2],
  //                    msg->velocity[0],
  //                   msg->velocity[1],
  //                    msg->velocity[2],
  //                       roll,
  //                        pitch,
  //                        yaw};


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



//void ModelPredictiveControl::publish_rpyFz()
//void ModelPredictiveControl::publish_rpyFz(struct command_struct& commandstruct)

void ModelPredictiveControl::publish_control()
{
  auto control_cmd = nmpc_ptr_->nmpc_cmd_struct; // Accessing nmpc_ptr_
      
  VehicleRatesSetpoint msg{};
  
  //msg.timestamp = timestamp_.load();
  msg.thrust_body[0] = 0.0;
  msg.thrust_body[1] = 0.0;      
  msg.thrust_body[2] = control_cmd.control_thrust_vec[2];
  msg.roll = control_cmd.control_attitude_vec[0];
  msg.pitch = control_cmd.control_attitude_vec[1];
  msg.yaw = control_cmd.control_attitude_vec[2];
   
  ModelPredictiveControl::vehicle_rates_setpoint_publisher_->publish(msg);


  //cout <<"C01: Thrust1:" << msg.thrust_body[0] << endl;
   //cout << "C12: Thrust2:" << msg.thrust_body[1] << endl;

  cout << "C1: Thrust:" << msg.thrust_body[2] << endl;
  
  //cout << "C2: p: " << msg.roll << endl;
  //cout << "C3: q: " << msg.pitch << endl;
  //cout << "C4: r: " << msg.yaw << endl;

    //nmpc_cmd_obj_pub.publish(obj_val_msg);       replace this!!
}


int main(int argc, char *argv[])
{


  ref_trajectory = { 2.0,     // px
                      2.0,    // py
                      2.0,    // pz
                      0.0,    // u
                      0.0,    // v
                      0.0,    // w
                      0.0,    // phi
                      0.0,    // theta
                      0.0     // psi
                      };
  nmpc_struct.U_ref.resize(NMPC_NU);
  nmpc_struct.W.resize(NMPC_NY);

  cout << NMPC_NX ;

  nmpc_struct.W(0) = 10;
  nmpc_struct.W(1) = 10;
  nmpc_struct.W(2) = 10;
  nmpc_struct.W(3) = 1.0;
  nmpc_struct.W(4) = 1.0;
  nmpc_struct.W(5) = 1.0;
  nmpc_struct.W(6) = 0.5;
  nmpc_struct.W(7) = 0.5;
  nmpc_struct.W(8) = 0.5;

  nmpc_struct.W(9) = 1.0;
  nmpc_struct.W(10) = 0.1;
  nmpc_struct.W(11) = 0.1;
  nmpc_struct.W(12) = 0.1;
  //nmpc_struct.W(13) = 0.1;


  nmpc_struct.min_Fz_scale = 0.1;
  nmpc_struct.max_Fz_scale = 10.0;
  nmpc_struct.W_Wn_factor = 0.5;

  nmpc_struct.U_ref(0) = 0.04;
  nmpc_struct.U_ref(1) = 0.01;
  nmpc_struct.U_ref(2) = 0.02;
  nmpc_struct.U_ref(3) = 0.03;


  std::cout << "MPC node..." << std::endl;
  
  std::cout << "Starting model predictive control node..." << std::endl;
   


  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  
  rclcpp::init(argc, argv);
  

    dist_Fx.data.resize(NMPC_N + 1);
    dist_Fy.data.resize(NMPC_N + 1);
    dist_Fz.data.resize(NMPC_N + 1);
    dist_Fx.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fy.data_zeros.resize(NMPC_N + 1, 0.0);
    dist_Fz.data_zeros.resize(NMPC_N + 1, 0.0);

    dist_Fx.data = dist_Fx.data_zeros;
    dist_Fy.data = dist_Fy.data_zeros;
    dist_Fz.data = dist_Fz.data_zeros;

    online_data.distFx = dist_Fx.data;
    online_data.distFy = dist_Fy.data;
    online_data.distFz = dist_Fz.data;

  online_data.distFx = {0.0,0.0,0.0};
  online_data.distFy = {0.0,0.0,0.0};
  online_data.distFz = {0.0,0.0,0.0};


  NMPC* nmpc = new NMPC(nmpc_struct);
  

  pos_ref = current_states;

  if (!nmpc->return_control_init_value())
      {  

          nmpc->nmpc_init(pos_ref, nmpc->nmpc_struct);

          //std::cout << "nmpc_struct.verbose\n" << nmpc_struct.verbose;

          //std::cout << "nmpc->return_control_init_value()\n" << nmpc->return_control_init_value();
          nmpc_struct.verbose = 1;

          if (nmpc_struct.verbose && nmpc->return_control_init_value())
          {
              std::cout << "***********************************\n";
              std::cout << "NMPC: initialized correctly\n";
              std::cout << "***********************************\n";
          }
      }

   // while (rclcpp::ok()) {
   //      std::cout << "NMPC is running \n";
         

    ///*

    auto mpc_node = std::make_shared<ModelPredictiveControl>(nmpc);

     while (rclcpp::ok()){ 

     current_states = { 1.0,     // px
                        0.0,     // py
                       0.0,   // pz
                         0.0,    // u
                         0.0,    // v
                        0.0,     // w
                        0.0,     // phi
                        0.0,     // theta
                        0.0};    // psi

      ii = ii + 0.01;

         nmpc->nmpc_core(nmpc_struct,
                      nmpc->nmpc_struct,
                               nmpc->nmpc_cmd_struct,
                               ref_trajectory,
                               online_data,
                               current_states);

 
          rclcpp::spin_some(mpc_node); // Spin only for the current node

  }

  
   
  rclcpp::shutdown();


  

 
  
 
  //nmpc->publish_rpyFz(nmpc->nmpc_cmd_struct);
  //delete nmpc; 
  return 0;
  }
