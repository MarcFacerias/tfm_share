cmake_minimum_required(VERSION 3.5)
project(l4vehicle_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  DIRECTORY
    msg
  FILES
    VehicleState.msg
    RawVehicleState.msg
    ChassisBrakePressure.msg
    ChassisSteeringAngle.msg
    ChassisWheelSpeed.msg
    ChassisShiftLever.msg
    BrakeCommand.msg
    SteeringTorqueCommand.msg
    ThrottleCommand.msg
    BrakeReport.msg
    SteeringReport.msg
    ThrottleReport.msg
    ControllerReport.msg
    SteeringPiControl.msg
    ModelPredictiveControlCommand.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
