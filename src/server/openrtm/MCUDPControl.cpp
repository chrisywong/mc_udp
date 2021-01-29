// -*- C++ -*-
/*!
 * @file  MCUDPControl.cpp * @brief Core component for MC control * $Date$
 *
 * $Id$
 */

/*
 * Copyright 2019-2020 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wpedantic"
#ifdef __clang__
#  pragma GCC diagnostic ignored "-Wdelete-incomplete"
#  pragma GCC diagnostic ignored "-Wshorten-64-to-32"
#endif
#include "MCUDPControl.h"

#include <mc_udp/logging.h>

#include <fstream>
#include <iomanip>

// Module specification
// <rtc-template block="module_spec">
// clang-format off
static const char* mccontrol_spec[] =
  {
    "implementation_id", "MCUDPControl",
    "type_name",         "MCUDPControl",
    "description",       "Core component for MC control",
    "version",           "0.1",
    "vendor",            "CNRS",
    "category",          "Generic",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.is_enabled", "0",
    "conf.default.port", "4445",
    "conf.default.robot", "NOT_SET",
    ""
  };
// </rtc-template>

MCUDPControl::MCUDPControl(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_enabled(false),
    port(4445),
    m_qOutOut("qOut", m_qOut),
#ifdef APPLY_LINK_EXTFORCES
    m_extforceFlagOut("extforceFlag", m_extforceFlag),
    m_extforceOutOut("extforceOut", m_extforceOut),
#endif
    server_(),
    got_control_(false),
    control_lost_(false),
    control_lost_iter_(0)
    // </rtc-template>
{
}
// clang-format on

MCUDPControl::~MCUDPControl() {}

RTC::ReturnCode_t MCUDPControl::onInitialize()
{
  MC_UDP_INFO("MCUDPControl::onInitialize() starting")
  // Set OutPort buffer
  addOutPort("qOut", m_qOutOut);
#ifdef APPLY_LINK_EXTFORCES
  addOutPort("extforceOut", m_extforceOutOut);
  addOutPort("extforceFlag", m_extforceFlagOut);
#endif

  // Bind variables and configuration variable
  bindParameter("is_enabled", m_enabled, "0");
  bindParameter("port", port, "4445");
  bindParameter("robot", robot_, "NOT_SET");

  MC_UDP_INFO("MCUDPControl::onInitialize() finished")
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCUDPControl::onActivated(RTC::UniqueId ec_id)
{
  MC_UDP_INFO("MCUDPControl::onActivated")
  server_.restart(port);
#ifdef APPLY_LINK_EXTFORCES
  MC_UDP_SUCCESS("[MCUDPCon-Server] on Activate with SimExtForce")
  m_extforceFlag.data.length(1);
  m_extforceFlag.data[0] = false;
  m_extforceFlagOut.write();
#endif
  MC_UDP_SUCCESS("MCUDPControl started on " << port)
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCUDPControl::onDeactivated(RTC::UniqueId ec_id)
{
  MC_UDP_INFO("MCUDPControl::onDeactivated")
  m_enabled = false;
  server_.stop();
  return RTC::RTC_OK;
}

RTC::ReturnCode_t MCUDPControl::onExecute(RTC::UniqueId ec_id)
{
  coil::TimeValue coiltm(coil::gettimeofday());
  RTC::Time tm;
  tm.sec = static_cast<CORBA::ULong>(coiltm.sec());
  tm.nsec = static_cast<CORBA::ULong>(coiltm.usec()) * 1000;
  if(m_enabled)
  {
    compute_start = std::chrono::system_clock::now();
    server_.send();
    if(server_.recv())
    {
      auto & control = server_.control().messages.at(robot_);
      m_qOut.data.length(control.encoders.size());
      got_control_ = true;
      if(control_lost_)
      {
        MC_UDP_WARNING("Control was lost for " << control_lost_iter_ << " iteration(s)")
        control_lost_iter_ = 0;
      }
      control_lost_ = false;
      for(unsigned int i = 0; i < m_qOut.data.length(); ++i)
      {
        m_qOut.data[i] = control.encoders[i];
      }
      m_qOut.tm = tm;
      m_qOutOut.write();

#ifdef APPLY_LINK_EXTFORCES // for use with RTCSimExtForce
      if(setextForceLength == false)
      {
        numextForce = control.encoders.size() + 1;
        m_extforceOut.data.length(numextForce * 6);
        setextForceLength = true;
      }
      else
      {
        for(int indFext = 0; indFext < numextForce; indFext++)
        {
          m_extforceOut.data[indFext * 6 + 0] = control.simExtForceVal.at(indFext).force().x();
          m_extforceOut.data[indFext * 6 + 1] = control.simExtForceVal.at(indFext).force().y();
          m_extforceOut.data[indFext * 6 + 2] = control.simExtForceVal.at(indFext).force().z();
          m_extforceOut.data[indFext * 6 + 3] = control.simExtForceVal.at(indFext).moment().x();
          m_extforceOut.data[indFext * 6 + 4] = control.simExtForceVal.at(indFext).moment().y();
          m_extforceOut.data[indFext * 6 + 5] = control.simExtForceVal.at(indFext).moment().z();
        }
        m_extforceFlag.data[0] = control.simExtForceFlag.at(0);
        m_extforceOutOut.write();
        m_extforceFlagOut.write();
      }
#endif
    }
    else
    {
      if(got_control_ && !control_lost_)
      {
        MC_UDP_ERROR("[MCUDPControl] Didn't receive new control, writing previous qOut as control")
        control_lost_ = true;
        got_control_ = false;
      }
      if(control_lost_)
      {
        control_lost_iter_++;
      }
    }
    compute_end = std::chrono::system_clock::now();
    compute_time = compute_end - compute_start;
    double elapsed = compute_time.count() * 1000;
    if(elapsed > 5.1)
    {
      MC_UDP_WARNING("Total time spent in MCUDPControl::onExecute (" << elapsed << ") exceeded 5.1ms")
    }
  }
  return RTC::RTC_OK;
}

extern "C"
{

  void MCUDPControlInit(RTC::Manager * manager)
  {
    coil::Properties profile(mccontrol_spec);
    manager->registerFactory(profile, RTC::Create<MCUDPControl>, RTC::Delete<MCUDPControl>);
  }
};

#pragma GCC diagnostic pop
