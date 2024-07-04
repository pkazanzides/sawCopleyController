/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s): Peter Kazanzides, Anton Deguet

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <fstream>

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnAssert.h>

#include <sawCopleyController/mtsCopleyController.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsCopleyController, mtsTaskContinuous, mtsStdString)

mtsCopleyController::mtsCopleyController(const std::string &name) : mtsTaskContinuous(name, 1024, true)
{
    Init();
}

mtsCopleyController::mtsCopleyController(const std::string &name, unsigned int sizeStateTable, bool newThread) :
    mtsTaskContinuous(name, sizeStateTable, newThread)
{
    Init();
}

mtsCopleyController::mtsCopleyController(const mtsTaskContinuousConstructorArg & arg) : mtsTaskContinuous(arg)
{
    Init();
}

mtsCopleyController::~mtsCopleyController()
{
    Close();
}

void mtsCopleyController::Init(void)
{
    mNumAxes = 1;
}

void mtsCopleyController::SetupInterfaces(void)
{
    mInterface = AddInterfaceProvided("control");
    if (mInterface) {
        // for Status, Warning and Error with mtsMessage
        mInterface->AddMessageEvents();

        // Stats
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats, "period_statistics");

        mInterface->AddCommandReadState(this->StateTable, mPosRaw, "GetPositionRaw");

        // Standard CRTK interfaces
        mInterface->AddCommandReadState(this->StateTable, m_measured_js, "measured_js");
        mInterface->AddCommandWrite(&mtsCopleyController::move_jp, this, "move_jp");
        mInterface->AddCommandWrite(&mtsCopleyController::move_jr, this, "move_jr");
        mInterface->AddCommandRead(&mtsCopleyController::GetConfig_js, this, "configuration_js");

        mInterface->AddCommandRead(&mtsCopleyController::GetConnected, this, "GetConnected");
        mInterface->AddCommandWriteReturn(&mtsCopleyController::SendCommandRet, this, "SendCommandRet");
        mInterface->AddCommandReadState(this->StateTable, mStatus, "GetStatus");
    }
}

void mtsCopleyController::Close()
{
#ifndef SIMULATION
    mSerialPort.Close();
#endif
}

void mtsCopleyController::Configure(const std::string& fileName)
{
    std::ifstream jsonStream;
    jsonStream.open(fileName.c_str());
    Json::Value jsonConfig;
    Json::Reader jsonReader;
    if (!jsonReader.parse(jsonStream, jsonConfig)) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse " << fileName << " for Copley config" << std::endl
                                 << jsonReader.getFormattedErrorMessages();
        exit(EXIT_FAILURE);
    }
    try {
        m_config.DeSerializeTextJSON(jsonConfig);
    } catch (std::exception & std_exception) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: " << fileName << ": " << std_exception.what() << std::endl;
        exit(EXIT_FAILURE);
    }   

    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed file " << fileName << std::endl
                               << "Loaded configuration:" << std::endl
                               << m_config << std::endl;
    
    // Size of array determines number of axes
    mNumAxes = static_cast<unsigned int>(m_config.axes.size());
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: found " << mNumAxes << " axes" << std::endl;

    // Now, set the data sizes
    m_config_j.Name().SetSize(mNumAxes);
    m_config_j.Type().SetSize(mNumAxes);
    m_config_j.PositionMin().SetSize(mNumAxes);
    m_config_j.PositionMax().SetSize(mNumAxes);
    // Raw encoder position
    mPosRaw.SetSize(mNumAxes);
    mPosRaw.SetAll(0);
    // We have position for measured_js
    m_measured_js.Name().SetSize(mNumAxes);
    m_measured_js.Position().SetSize(mNumAxes);
    m_measured_js.Position().SetAll(0.0);
    // Status
    mStatus.SetSize(mNumAxes);
    mStatus.SetAll(0);

    for (unsigned int axis = 0; axis < mNumAxes; axis++) {
        sawCopleyControllerConfig::axis &axisData = m_config.axes[axis];
        m_measured_js.Name()[axis].assign(1, 'A'+axis);
        m_config_j.Name()[axis].assign(1, 'A'+axis);
        m_config_j.Type()[axis] = static_cast<cmnJointType>(axisData.type);
        m_config_j.PositionMin()[axis] = axisData.position_limits.lower;
        m_config_j.PositionMax()[axis] = axisData.position_limits.upper;
    }

    StateTable.AddData(mPosRaw, "position_raw");
    StateTable.AddData(m_measured_js, "measured_js");
    StateTable.AddData(mStatus, "status");

    // Call SetupInterfaces after Configure because we need to know the correct sizes of
    // the dynamic vectors, which are based on the number of configured axes.
    // These sizes should be set before calling StateTable.AddData and AddCommandReadState;
    // in the latter case, this ensures that the argument prototype has the correct size.
    SetupInterfaces();

#ifndef SIMULATION
    mSerialPort.SetPortName(m_config.port_name);
    // Default constructor sets port to 9600 baud, N,8,1
    if (!mSerialPort.Open()) {
        mInterface->SendError(this->GetName() + ": failed to open serial port: "
                                        + mSerialPort.GetPortName());
        return;
    }
    mSerialPort.Configure();

    // Write a break, to make sure serial port is at 9600 baud
    const double breakTime = 0.5 * cmn_s;
    mSerialPort.WriteBreak(breakTime);
    // Wait for length of break and a bit more
    Sleep(breakTime + 0.5 * cmn_s);

    if (m_config.baud_rate != 9600) {
        // If desired baud rate is not 9600
        osaSerialPort::BaudRateType baudRate;
        switch (m_config.baud_rate) {
        case 19200:  baudRate = osaSerialPort::BaudRate19200;
                     break;
        case 38400:  baudRate = osaSerialPort::BaudRate38400;
                     break;
        case 57600:  baudRate = osaSerialPort::BaudRate57600;
                     break;
        case 115200: baudRate = osaSerialPort::BaudRate115200;
                     break;
        default:     CMN_LOG_CLASS_INIT_ERROR << "Unsupported baud rate " << m_config.baud_rate << std::endl;
                     return;
        }
        std::cout << this->GetName() <<  ": setting baud rate to " << m_config.baud_rate << std::endl;
        
        sprintf(cmdBuf, "s r0x90 %d\r", m_config.baud_rate);
        int nBytes = static_cast<int>(strlen(cmdBuf));
        int nSent = mSerialPort.Write(cmdBuf, nBytes);
        if (nSent != nBytes) {
            CMN_LOG_CLASS_INIT_ERROR << "Failed to set baud rate to " << m_config.baud_rate << std::endl;
            return;
        }
        // Wait at least 100 msec after setting baud rate
        Sleep(0.1);
        mSerialPort.SetBaudRate(baudRate);
        if (!mSerialPort.Configure()) {
            CMN_LOG_CLASS_INIT_ERROR << "Failed to configure serial port" << std::endl;
        }
        std::cout << this->GetName() << ": configuration completed" << std::endl;
    }
#endif
    if (!m_config.ccx_file.empty()) {
        LoadCCX(m_config.ccx_file);
    }
}

void mtsCopleyController::Startup()
{
    //std::cout << this->GetName() << ": saving parameters to file" << std::endl;
    //SaveParameters(m_config.name + ".csv");
    //std::cout << this->GetName() << ": finished saving parameters" << std::endl;
}

void mtsCopleyController::Run()
{
#ifndef SIMULATION
    if (mSerialPort.IsOpened()) {
        long posRaw, status;
        for (unsigned int axis = 0; axis < mNumAxes; axis++) {
            if (ParameterGet(0x32, posRaw, axis) == 0) {
                mPosRaw[axis] = posRaw;
                m_measured_js.Position()[axis] = mPosRaw[axis]/m_config.axes[axis].position_bits_to_SI.scale;
            }
            if (ParameterGet(0xa0, status, axis) == 0) {
                mStatus[axis] = status;
            }
        }
    }
#else
    for (unsigned int axis = 0; axis < mNumAxes; axis++) {
        mPosRaw[axis] = 0;
        m_measured_js.Position()[axis] = 0.0;
        mStatus[axis] = 0;
    }
#endif

    // Advance the state table now, so that any connected components can get
    // the latest data.
    StateTable.Advance();

    // Call any connected components
    RunEvent();

    ProcessQueuedCommands();
}

void mtsCopleyController::Cleanup(){
    Close();
}

int mtsCopleyController::SendCommand(const char *cmd, int len, long *value)
{
    int rc = -1;
#ifndef SIMULATION
    msgBuf[0] = 0;
    if (mSerialPort.IsOpened()) {
        int nSent = mSerialPort.Write(cmd, len);
        // Add CR ('\r') if not already in cmd
        if (cmd[len-1] != '\r') {
            char cr = '\r';
            len++;
            nSent += mSerialPort.Write(&cr, 1);
        }
        if (nSent == len) {
            if (cmd[0] == 'r') {
                // No response to reset command
                mInterface->SendStatus("SendCommand: reset");
                return 0;
            }
            Sleep(0.1);  // TEMP
            char respBuf[64];
            respBuf[0] = 0;
            int nRecv = mSerialPort.Read(respBuf, sizeof(respBuf));
            if (nRecv > 0) {
                respBuf[nRecv] = 0;  // May not be needed
                if (strncmp(respBuf, "ok", 2) == 0) {
                    rc = 0;
                }
                else if (strncmp(respBuf, "e ", 2) == 0) {
                    sscanf(respBuf+2, "%d", &rc);
                    sprintf(msgBuf, "SendCommand %s: error %d", cmd, rc);
                }
                else if ((strncmp(respBuf, "v ", 2) == 0) ||
                         (strncmp(respBuf, "r ", 2) == 0)) {
                    if (value) {
                        if (sscanf(respBuf+2, "%d", value) != 1)
                            sprintf(msgBuf, "SendCommand %s: failed to parse response %s", cmd, respBuf);
                    }
                    else {
                        sprintf(msgBuf, "SendCommand %s: ignoring response %s", cmd, respBuf);
                    }
                }
                else {
                    sprintf(msgBuf, "SendCommand %s: unexpected response %s", cmd, respBuf);
                }
            }
        }
        else {
            sprintf(msgBuf, "SendCommand %s: failed to write command (nSent = %d, len = %d)",
                    cmd, nSent, len);
        }
    }
    else {
        sprintf(msgBuf, "SendCommand %s: serial port not open", cmd);
    }
#else
    sprintf(msgBuf, "SendCommand %s: no serial port in SIMULATION", cmd);
#endif

    if (msgBuf[0])
        mInterface->SendError(msgBuf);

    return rc;
}

int mtsCopleyController::ParameterSet(unsigned int addr, long value, unsigned int axis, bool inRAM)
{
    char *p = cmdBuf;
    if (mNumAxes != 1) {
        sprintf(p, ".%c ", 'A'+axis);
        p += 3;
    }
    char bank = inRAM ? 'r' : 'f';
    sprintf(p, "s %c0x%x %d\r", bank, addr, value);
    return SendCommand(cmdBuf, static_cast<int>(strlen(cmdBuf)));
}

int mtsCopleyController::ParameterGet(unsigned int addr, long &value, unsigned int axis, bool inRAM)
{
    char *p = cmdBuf;
    if (mNumAxes != 1) {
        sprintf(p, ".%c ", 'A'+axis);
        p += 3;
    }
    char bank = inRAM ? 'r' : 'f';
    sprintf(p, "g %c0x%x\r", bank, addr);
    return SendCommand(cmdBuf, static_cast<int>(strlen(cmdBuf)), &value);
}

void mtsCopleyController::SendCommandRet(const std::string &cmdString, std::string &retString)
{
#ifndef SIMULATION
    if (mSerialPort.IsOpened()) {
        int nSent = mSerialPort.Write(cmdString+"\r");
        if (nSent != cmdString.size()+1) {
            CMN_LOG_CLASS_RUN_ERROR << "Failed to write " << cmdString << std::endl;
            retString.assign("Failed to write command");
            return;
        }
        Sleep(0.1);  // TEMP
        char respBuf[64];
        respBuf[0] = 0;
        int nRecv = mSerialPort.Read(respBuf, sizeof(respBuf));
        respBuf[nRecv] = 0;   // May not be needed
        retString.assign(respBuf);
    }
#else
    retString.assign("SIMULATION");
#endif
}

void mtsCopleyController::move_jp(const prmPositionJointSet &goal)
{
    // profile_type 0 means absolute move, trapezoidal profile
    move_common("move_jp", goal.Goal(), 0);
}

void mtsCopleyController::move_jr(const prmPositionJointSet &goal)
{
    // profile_type 256 means relative move, trapezoidal profile
    move_common("move_jr", goal.Goal(), 256);
}

void mtsCopleyController::move_common(const char *cmdName, const vctDoubleVec &goal,
                                      unsigned int profile_type)
{
    if (goal.size() != mNumAxes) {
        mInterface->SendError(this->GetName() + ": size mismatch in " + std::string(cmdName));
        return;
    }

#ifndef SIMULATION
    unsigned int axis;
    for (axis = 0; axis < mNumAxes; axis++) {
        ParameterSet(0xc8, profile_type, axis);
        long goalCnts = static_cast<long>(goal[axis]/m_config.axes[axis].position_bits_to_SI.scale);
        ParameterSet(0xca, goalCnts, axis);  // Position goal
        // TODO: set velocity, accel, decel
        long desiredState = -1;
        ParameterGet(0x24, desiredState, axis);
        if (desiredState != 21) {
            sprintf(msgBuf, "%s: changing state for axis %d from %ld to 21", cmdName, axis, desiredState);
            mInterface->SendStatus(msgBuf);
            ParameterSet(0x24, 21, axis);
        }
    }
    // Note that newer multi-axis drives allow the axes to be specified in the command code;
    // the following code could be updated to use that feature.
    for (axis = 0; axis < mNumAxes; axis++) {
        int rc;
        if (mNumAxes == 1) {
            rc = SendCommand("t 1\r", 4);
        }
        else {
            sprintf(cmdBuf, ".%c t 1\r", 'A'+axis);
            rc = SendCommand(cmdBuf, 7);
        }
        if (rc != 0) {
            sprintf(msgBuf, "%s: axis %d, error %d", cmdName, axis, rc);
            mInterface->SendStatus(msgBuf);
        }
    }
#endif
}

// For testing

struct ParameterList {
    unsigned int addr;
    std::string desc;
};

ParameterList parms[] = { // Programmed Position Mode Parameters
                          { 0xc8, "profile type"   },
                          { 0xcb, "max velocity"   },
                          { 0xcc, "max accel"      },
                          { 0xcd, "max decel"      },
                          // Homing Mode Parameters
                          { 0xc2, "homing method"  },
                          { 0xc3, "fast vel"       },
                          { 0xc4, "slow vel"       },
                          { 0xc5, "accel/decel"    },
                          { 0xc6, "home offset"    },
                          { 0xc7, "current limit"  },
                          { 0xb8, "swlim+"         },
                          { 0xb9, "swlim-"         },
                          // Current Loop Limits Parameters
                          { 0x21, "peak current"   },
                          { 0x22, "cont current"   },
                          { 0x23, "I2T time"       },
                          { 0xae, "current offset" },
                          // Current Loop Gains Parameters
                          { 0x00, "Cp"             },
                          { 0x01, "Ci"             },
                          // Velocity Loop Limits Parameters
                          { 0x3a, "vel limit"      },
                          { 0x36, "accel limit"    },
                          { 0x37, "decel limit"    },
                          { 0xcf, "fast stop ramp" },
                          // Velocity Loop Gains Parameters
                          { 0x27, "Vp",            },
                          { 0x28, "Vi",            },
                          // Position Loop Gains Parameters
                          { 0x30, "Pp"             },
                          { 0x33, "vel ff"         },
                          { 0x34, "accel ff"       },
                          { 0xe3, "gain mult"      }
                        };

void mtsCopleyController::SaveParameters(const std::string &fileName)
{
    std::ofstream csvFile(fileName.c_str());
    size_t numParms = sizeof(parms)/sizeof(ParameterList);
    for (size_t i = 0; i < numParms; i++) {
        csvFile << parms[i].desc << ", " << std::hex << parms[i].addr << ", ";
        long fromRAM, fromFlash;
        if (ParameterGet(parms[i].addr, fromRAM) == 0)
            csvFile << std::hex << fromRAM << ", ";
        else
            csvFile << "ERROR, ";
        if (ParameterGet(parms[i].addr, fromFlash, false) == 0)
            csvFile << std::hex << fromFlash;
        else
            csvFile << "ERROR";
        csvFile << std::dec << std::endl;
    }
}

bool mtsCopleyController::LoadCCX(const std::string &fileName)
{
    // CCX File Format
    // First line:  version number (14)
    // Second line:  number of axes (1)
    // Subsequent lines are data, of format
    //    param-id (hex), axis-num (0), name, value
    // In most cases, value is a decimal integer, but there are some special cases:
    //    - strings
    //    - multiple values, which are separated by colon, e.g., 0:1:2
    // Special cases:
    //    - param-ids 70-77 (programmable outputs) require 3 values, in hex
    //    - param-id 95 is host config state; used by host software
    char buf[128];
    std::ifstream ccxFile(fileName.c_str());
    if (!ccxFile.is_open()) {
        CMN_LOG_CLASS_INIT_ERROR << "LoadCCX: failed to open " << fileName << std::endl;
        return false;
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "LoadCCX: parsing file " << fileName << std::endl;
    ccxFile.getline(buf, sizeof(buf));
    unsigned int fileVer;
    if (sscanf(buf, "%d", &fileVer) != 1) {
        CMN_LOG_CLASS_INIT_ERROR << "LoadCCX: failed to parse file version from [" << buf << "]" << std::endl;
        return false;
    }
    if ((fileVer != 13) && (fileVer != 14))
        CMN_LOG_CLASS_INIT_WARNING << "LoadCCX: unsupported file version " << fileVer << std::endl;
    ccxFile.getline(buf, sizeof(buf));
    unsigned int numAxes;
    if (sscanf(buf, "%d", &numAxes) != 1) {
        CMN_LOG_CLASS_INIT_ERROR << "LoadCCX: failed to parse number of axes from [" << buf << "]" << std::endl;
        return false;
    }
    if (numAxes != mNumAxes) {
        CMN_LOG_CLASS_INIT_ERROR << "LoadCCX: inconsistent number of axes (json " << mNumAxes
                                 << ", ccx " << numAxes << ")" << std::endl;
        return false;
    }
    while (ccxFile.good()) {
        ccxFile.getline(buf, sizeof(buf));
        unsigned int param, axis, nchars;
        if (sscanf(buf, "%x,%d,%n", &param, &axis, &nchars) != 2) {
            CMN_LOG_CLASS_INIT_WARNING << "LoadCCX: failed to parse line [" << buf << "]" << std::endl;
            continue;
        }
        char *name = strtok(buf+nchars, ",\n");
        if (!name) {
            CMN_LOG_CLASS_INIT_WARNING << "LoadCCX: failed to parse parameter name, param " << std::hex
                                       << param << std::dec << std::endl;
            continue;
        }
        char *valueStr = strtok(0, ",\n");
        std::cout << "Parameter " << std::hex << param << ", axis " << axis << ", name " << name
                  << ", value " << valueStr << std::endl;
    }
    return true;
}
