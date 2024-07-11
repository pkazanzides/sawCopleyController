/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Peter Kazanzides

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

  This component provides an interface to a Copley controller, using a serial port.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsCopleyController_h
#define _mtsCopleyController_h

#include <string>
#include <vector>

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstOSAbstraction/osaSerialPort.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>
#include <cisstParameterTypes/prmOperatingState.h>

#include <sawCopleyController/sawCopleyControllerConfig.h>

// Always include last
#include <sawCopleyController/sawCopleyControllerExport.h>

class CISST_EXPORT mtsCopleyController : public mtsTaskContinuous
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_LOD_RUN_ERROR)

 public:

    mtsCopleyController(const std::string &name);
    mtsCopleyController(const std::string &name, unsigned int sizeStateTable, bool newThread = true);
    mtsCopleyController(const mtsTaskContinuousConstructorArg &arg);

    ~mtsCopleyController();

    // cisstMultiTask functions
    void Configure(const std::string &fileName) override;
    void Startup(void) override;
    void Run(void) override;
    void Cleanup(void) override;

protected:

    sawCopleyControllerConfig::controller m_config;
    unsigned int mNumAxes;                  // Number of axes

    osaSerialPort mSerialPort;
    mtsInterfaceProvided *mInterface;       // Provided interface

    char cmdBuf[64];   // Buffer for sending commands
    char msgBuf[128];  // Buffer for sending messages

    vctLongVec mPosRaw;
    vctDoubleVec mPos;
    vctLongVec mStatus;
    prmConfigurationJoint m_config_j;       // Joint configuration
    prmStateJoint m_measured_js;            // Measured joint state (CRTK)
    vctDoubleVec mDispScale;                // Display scale
    std::vector<std::string> mDispUnits;    // Display units

    vctDoubleVec mSpeed;                    // Max speed for position move
    vctDoubleVec mAccel;                    // Max accel for position move
    vctDoubleVec mDecel;                    // Max decel for position move

    vctUIntVec mState;                      // Internal state machine

    void Init();
    void Close();

    void SetupInterfaces();
    bool LoadCCX(const std::string &fileName);

    // Send the command to the drive; returns 0 on success
    // For a read command, result returned in value
    int SendCommand(const char *cmd, int len, long *value = 0, unsigned int num = 1);

    int ParameterSet(unsigned int addr, long value, unsigned int axis = 0, bool inRAM = true);
    int ParameterGet(unsigned int addr, long &value, unsigned int axis = 0, bool inRAM = true);

    int ParameterSetArray(unsigned int addr, long *value, unsigned int num, unsigned int axis = 0, bool inRAM = true);
    int ParameterGetArray(unsigned int addr, long *value, unsigned int num, unsigned int axis = 0, bool inRAM = true);

    // Methods for provided interface
    void GetConnected(bool &val) const;
    void SendCommandRet(const std::string& cmdString, std::string &retString);

    // Get joint configuration
    void GetConfig_js(prmConfigurationJoint &cfg_j) const
    { cfg_j = m_config_j; }

    void move_jp(const prmPositionJointSet &goal);
    void move_jr(const prmPositionJointSet &goal);

    void move_common(const char *cmdName, const vctDoubleVec &goal, unsigned int profile_type);

    // Set speed, acceleration and deceleration
    void SetSpeed(const vctDoubleVec &spd);
    void SetAccel(const vctDoubleVec &accel);
    void SetDecel(const vctDoubleVec &decel);

    // Enable motor power
    void EnableMotorPower(void);
    // Disable motor power
    void DisableMotorPower(void);

    // Home: all axes
    void HomeAll();
    // Home: mask indicates which axes to home
    void Home(const vctBoolVec &mask);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsCopleyController)

#endif
