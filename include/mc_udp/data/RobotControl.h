/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_udp/data/api.h>

#include <stdint.h>
#include <string>
#include <vector>
#ifdef APPLY_LINK_EXTFORCES // for use with RTCSimExtForce
#  include <SpaceVecAlg/SpaceVecAlg>
#endif

namespace mc_udp
{

/** Robot control data sent to the server */
struct MC_UDP_DATA_DLLAPI RobotControl
{
  /** Unique identifier for the control, should repeat the same as the sensors' reading */
  uint64_t id;
  /** Joint encoders provided in the robot reference order */
  std::vector<double> encoders;
  /** Desired encoder velocity provided in the robot reference order
   * May be empty */
  std::vector<double> encoderVelocities;

#ifdef APPLY_LINK_EXTFORCES // for use with RTCSimExtForce
  /** Desired external force to be applied in simulation provided in the robot
   * reference order */
  std::vector<sva::ForceVecd> simExtForceVal;
  /** Enable/disable flag status for external force applied in simulation
      Although defined as bool in the controller-side, sent as int because
     std::vector<bool> cannot be const
  */
  std::vector<int> simExtForceFlag;
#endif

  /** Compute required buffer size */
  size_t size() const;
  /** Fill pre-allocated buffer, must be of (at-least) size()
   *
   * Returns the number of bytes written */
  size_t toBuffer(uint8_t * buffer) const;
  /** Fill from a provided buffer
   *
   * Returns the number of bytes consumed */
  size_t fromBuffer(uint8_t * buffer);
};

} // namespace mc_udp
