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
    mPosRaw = 0;
    mPos = 0.0;
    mStatus = 0;
    StateTable.AddData(mPosRaw, "position_raw");
    StateTable.AddData(mPos,    "position");
    StateTable.AddData(mStatus, "status");
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
        mInterface->AddCommandReadState(this->StateTable, mPos, "GetPosition");
        mInterface->AddCommandReadState(this->StateTable, mStatus, "GetStatus");

        mInterface->AddCommandRead(&mtsCopleyController::GetConnected, this, "GetConnected");
        mInterface->AddCommandRead(&mtsCopleyController::GetJointType, this, "GetJointType");
        mInterface->AddCommandWriteReturn(&mtsCopleyController::SendCommandRet, this, "SendCommandRet");
        mInterface->AddCommandWrite(&mtsCopleyController::PosMoveAbsolute, this, "PosMoveAbsolute");
        mInterface->AddCommandWrite(&mtsCopleyController::PosMoveRelative, this, "PosMoveRelative");
    }
}

void mtsCopleyController::Close()
{
    mSerialPort.Close();
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
    
    SetupInterfaces();

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
}

void mtsCopleyController::Startup()
{
    //std::cout << this->GetName() << ": saving parameters to file" << std::endl;
    //SaveParameters(m_config.name + ".csv");
    //std::cout << this->GetName() << ": finished saving parameters" << std::endl;
}

void mtsCopleyController::Run()
{
    if (mSerialPort.IsOpened()) {
        if (ParameterGet(0x32, mPosRaw) == 0)
            mPos = mPosRaw/m_config.drive.position_bits_to_SI.scale;
        ParameterGet(0xa0, mStatus);
    }

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

    if (msgBuf[0])
        mInterface->SendError(msgBuf);

    return rc;
}

int mtsCopleyController::ParameterSet(unsigned int addr, long value, bool inRAM)
{
    char bank = inRAM ? 'r' : 'f';
    sprintf(cmdBuf, "s %c0x%x %d\r", bank, addr, value);
    return SendCommand(cmdBuf, static_cast<int>(strlen(cmdBuf)));
}

int mtsCopleyController::ParameterGet(unsigned int addr, long &value, bool inRAM)
{
    char bank = inRAM ? 'r' : 'f';
    sprintf(cmdBuf, "g %c0x%x\r", bank, addr);
    return SendCommand(cmdBuf, static_cast<int>(strlen(cmdBuf)), &value);
}

void mtsCopleyController::SendCommandRet(const std::string &cmdString, std::string &retString)
{
    if (mSerialPort.IsOpened()) {
        int nSent = mSerialPort.Write(cmdString+"\r");
        if (nSent != cmdString.size()+1) {
            CMN_LOG_RUN_ERROR << "Failed to write " << cmdString << std::endl;
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
}

void mtsCopleyController::GetJointType(cmnJointType &jType) const
{
    jType = static_cast<cmnJointType>(m_config.drive.type);
}

void mtsCopleyController::PosMoveAbsolute(const double &goal)
{
    ParameterSet(0xc8, 0);         // Absolute move, trapezoidal profile
    long goalCnts = static_cast<long>(goal/m_config.drive.position_bits_to_SI.scale);
    ParameterSet(0xca, goalCnts);  // Position goal
    // TODO: set velocity, accel, decel
    long desiredState = -1;
    ParameterGet(0x24, desiredState);
    if (desiredState != 21) {
        sprintf(msgBuf, "PosMoveAbsolute: changing state from %ld to 21", desiredState);
        mInterface->SendStatus(msgBuf);
        ParameterSet(0x24, 21);
    }
    int rc = SendCommand("t 1\r", 4);
    if (rc != 0) {
        sprintf(msgBuf, "PosMoveAbsolute: error %d", rc);
        mInterface->SendStatus(msgBuf);
    }
}

void mtsCopleyController::PosMoveRelative(const double &goal)
{
    ParameterSet(0xc8, 256);       // Relative move, trapezoidal profile
    long goalCnts = static_cast<long>(goal/m_config.drive.position_bits_to_SI.scale);
    ParameterSet(0xca, goalCnts);  // Position goal
    // TODO: set velocity, accel, decel
    long desiredState = -1;
    ParameterGet(0x24, desiredState);
    if (desiredState != 21) {
        sprintf(msgBuf, "PosMoveRelative: changing state from %ld to 21", desiredState);
        mInterface->SendStatus(msgBuf);
        ParameterSet(0x24, 21);
    }
    int rc = SendCommand("t 1\r", 4);
    if (rc != 0) {
        sprintf(msgBuf, "PosMoveRelative: error %d", rc);
        mInterface->SendStatus(msgBuf);
    }
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
        long fromRAM, fromFlash;
        ParameterGet(parms[i].addr, fromRAM);
        ParameterGet(parms[i].addr, fromFlash, false);
        csvFile << parms[i].desc << ", " << std::hex << parms[i].addr << ", "
                << fromRAM << ", " << fromFlash << std::dec << std::endl;
    }
}
