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
}

void mtsCopleyController::SetupInterfaces(void)
{
    mInterface = AddInterfaceProvided("control");
    if (mInterface) {
        // for Status, Warning and Error with mtsMessage
        mInterface->AddMessageEvents();

        // Stats
        mInterface->AddCommandReadState(StateTable, StateTable.PeriodStats, "period_statistics");

        // Extra stuff
        mInterface->AddCommandRead(&mtsCopleyController::GetConnected, this, "GetConnected");
        mInterface->AddCommandWrite(&mtsCopleyController::SendCommand, this, "SendCommand");
        mInterface->AddCommandWriteReturn(&mtsCopleyController::SendCommandRet, this, "SendCommandRet");
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
        char buf[64];
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
        sprintf(buf, ": setting baud rate to %d", m_config.baud_rate);
        mInterface->SendStatus(this->GetName() + buf);
        
        sprintf(buf, "s r0x90 %d\r", m_config.baud_rate);
        int nBytes = strlen(buf);
        int nSent = mSerialPort.Write(buf, nBytes); 
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
    }
}

void mtsCopleyController::Startup()
{
}

void mtsCopleyController::Run()
{
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

void mtsCopleyController::SendCommand(const std::string &cmdString)
{
    if (mSerialPort.IsOpened()) {
        int nSent = mSerialPort.Write(cmdString);
        if (nSent != cmdString.size()) {
            CMN_LOG_RUN_ERROR << "Failed to write " << cmdString << std::endl;
            return;
        }
        Sleep(0.1);  // TEMP
        char respBuf[64];
        respBuf[0] = 0;
        int nRecv = mSerialPort.Read(respBuf, sizeof(respBuf));
        if (strncmp(respBuf, "ok", 2) != 0)
            mInterface->SendError(cmdString + ": response was " + respBuf);
    }
}

void mtsCopleyController::SendCommandRet(const std::string &cmdString, std::string &retString)
{
    if (mSerialPort.IsOpened()) {
        int nSent = mSerialPort.Write(cmdString);
        if (nSent != cmdString.size()) {
            CMN_LOG_RUN_ERROR << "Failed to write " << cmdString << std::endl;
            return;
        }
        Sleep(0.1);  // TEMP
        char respBuf[64];
        respBuf[0] = 0;
        mSerialPort.Read(respBuf, sizeof(respBuf));
        retString.assign(respBuf);
    }
}
