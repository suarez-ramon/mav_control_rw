/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef POSITION_CONTROLLER_INTERFACE_H_
#define POSITION_CONTROLLER_INTERFACE_H_

#include <mav_msgs_rotors/conversions.h>
#include <mav_msgs_rotors/eigen_mav_msgs_rotors.h>
#include <nav_msgs/Odometry.h>

namespace mav_control_interface {

class PositionControllerInterface {
 public:
  PositionControllerInterface();
  virtual ~PositionControllerInterface();

  virtual std::string getName() const = 0;

  virtual bool getUseAttitudeQuaternionCommand() const = 0;

  virtual double getMass() const = 0;

  virtual bool setReference(const mav_msgs_rotors::EigenTrajectoryPoint& reference) = 0;

  virtual bool setReferenceArray(const mav_msgs_rotors::EigenTrajectoryPointDeque& reference_array);

  virtual bool setOdometry(const mav_msgs_rotors::EigenOdometry& odometry) = 0;

  virtual bool getCurrentReference(mav_msgs_rotors::EigenTrajectoryPoint* reference) const = 0;

  virtual bool getCurrentReference(mav_msgs_rotors::EigenTrajectoryPointDeque* reference) const = 0;

  virtual bool getPredictedState(mav_msgs_rotors::EigenTrajectoryPointDeque* predicted_state) const = 0;

  virtual bool calculateRollPitchYawrateThrustCommand(
      mav_msgs_rotors::EigenRollPitchYawrateThrust* attitude_thrust_command);

  virtual bool calculateAttitudeThrustCommand(mav_msgs_rotors::EigenAttitudeThrust* attitude_thrust_command);
};

} /* namespace mav_flight_manager */

#endif /* POSITION_CONTROLLER_INTERFACE_H_ */
