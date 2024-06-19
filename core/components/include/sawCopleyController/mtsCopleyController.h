/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Peter Kazanzides, Anton Deguet

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

#include <cisstCommon/cmnJointType.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstOSAbstraction/osaSerialPort.h>

// PK TODO
#include <cisstMultiTask/mtsGenericObjectProxy.h>
typedef mtsGenericObjectProxy<cmnJointType> cmnJointTypeProxy;
CMN_DECLARE_SERVICES_INSTANTIATION(cmnJointTypeProxy);

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

    osaSerialPort mSerialPort;
    mtsInterfaceProvided *mInterface;       // Provided interface

    long mPosRaw;
    double mPos;
    long mStatus;

    void Init();
    void Close();

    void SetupInterfaces();

    // Send the command to the drive; returns 0 on success
    // For a read command, result returned in value
    int SendCommand(const char *cmd, int len, long *value = 0);

    // Methods for provided interface
    void GetConnected(bool &val) const { val = mSerialPort.IsOpened(); }
    void SendCommandRet(const std::string& cmdString, std::string &retString);
    void GetJointType(cmnJointType &jType) const;
    void PosMoveAbsolute(const double &goal);
    void PosMoveRelative(const double &goal);
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsCopleyController)

#endif
