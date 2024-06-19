/*-*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-   */
/*ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab:*/

/*
  Author(s): Peter Kazanzides

  (C) Copyright 2024 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <string>

#include <cisstCommon/cmnLogger.h>
#include <cisstCommon/cmnKbHit.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstCommon/cmnJointType.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawCopleyController/mtsCopleyController.h>

class CopleyClient : public mtsTaskMain {

private:
    cmnJointType jType;
    double jtScale;
    std::string jtUnits;
    long mPosRaw;
    double mPos;
    long mStatus;
    unsigned int numCharsPrev;

    mtsFunctionRead GetJointType;   // Prismatic or revolute
    mtsFunctionRead GetPosition;    // Position in SI units
    mtsFunctionRead GetPositionRaw; // Position in encoder counts
    mtsFunctionRead GetStatus;      // Drive status
    mtsFunctionWrite PosMoveAbsolute;
    mtsFunctionWrite PosMoveRelative;
    mtsFunctionRead GetConnected;   // Whether serial port is open
    mtsFunctionWriteReturn SendCommandRet;

    void OnStatusEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Status: " << msg.Message << std::endl;
    }
    void OnWarningEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Warning: " << msg.Message << std::endl;
    }
    void OnErrorEvent(const mtsMessage &msg) {
        std::cout << std::endl << "Error: " << msg.Message << std::endl;
    }

public:

    CopleyClient() : mtsTaskMain("CopleyClient"), numCharsPrev(0)
    {
        mtsInterfaceRequired *req = AddInterfaceRequired("Input", MTS_OPTIONAL);
        if (req) {
            req->AddFunction("GetPosition", GetPosition);
            req->AddFunction("GetPositionRaw", GetPositionRaw);
            req->AddFunction("GetStatus", GetStatus);
            req->AddFunction("PosMoveAbsolute", PosMoveAbsolute);
            req->AddFunction("PosMoveRelative", PosMoveRelative);
            req->AddFunction("SendCommandRet", SendCommandRet);
            req->AddFunction("GetJointType", GetJointType);
            req->AddEventHandlerWrite(&CopleyClient::OnStatusEvent, this, "status");
            req->AddEventHandlerWrite(&CopleyClient::OnWarningEvent, this, "warning");
            req->AddEventHandlerWrite(&CopleyClient::OnErrorEvent, this, "error");
        }
    }

    void Configure(const std::string&) {}

    void PrintHelp()
    {
        std::cout << "Available commands:" << std::endl
                  << "  m: absolute position move" << std::endl
                  << "  r: relative position move" << std::endl
                  << "  c: send command" << std::endl
                  << "  h: display help information" << std::endl
                  << "  q: quit" << std::endl;
    }

    void Startup()
    {
        GetJointType(jType);
        if (jType == CMN_JOINT_PRISMATIC) {
            jtScale = 1000.0;     // meters --> millimeters
            jtUnits.assign("mm");
        }
        else if (jType == CMN_JOINT_REVOLUTE) {
            jtScale = cmn180_PI;  // radians --> degrees
            jtUnits.assign("deg");
        }
        else {
            std::cout << "CopleyClient: unknown joint type (" << jType << ")" << std::endl;
            jtScale = 1.0;
            jtUnits.assign("cnts");
        }
        PrintHelp();
    }

    void Run() {

        ProcessQueuedEvents();

        bool copleyOK;
        GetConnected(copleyOK);

        if (copleyOK) {
            GetPositionRaw(mPosRaw);
            GetPosition(mPos);
            GetStatus(mStatus);
        }

        char c = 0;
        double jtGoal;
        if (cmnKbHit()) {
            c = cmnGetChar();
            switch (c) {

            case 'm':   // position move joint
                std::cout << std::endl << "Enter absolute position (" << jtUnits << "): ";
                std::cin >> jtGoal;
                std::cout << "Moving to " << jtGoal << std::endl;
                PosMoveAbsolute(jtGoal/jtScale);
                break;

            case 'r':   // relative move joint
                std::cout << std::endl << "Enter relative position (" << jtUnits << "): ";
                std::cin >> jtGoal;
                std::cout << "Relative move by " << jtGoal << std::endl;
                PosMoveRelative(jtGoal/jtScale);
                break;

            case 'c':
                if (copleyOK) {
                    std::string cmdString;
                    std::string retString;
                    std::cout << std::endl << "Enter command: ";
                    for (;;) {
                        std::getline(std::cin, cmdString);
                        if (!cmdString.empty()) {
                            SendCommandRet(cmdString, retString);
                            std::cout << "Return: " << retString << std::endl;
                            break;
                        }
                    }
                }
                else {
                    std::cout << std::endl << "Command not available - Copley not connected" << std::endl;
                }
                break;
                
            case 'h':
                std::cout << std::endl;
                PrintHelp();
                break;

            case 'q':   // quit program
                std::cout << std::endl << "Exiting.. " << std::endl;
                this->Kill();
                break;

            }
        }

        if (copleyOK) {
            printf("Pos: %8.2lf (%08lx), Status: %08lx", mPos*jtScale, mPosRaw, mStatus);
            unsigned int numChars = 0;
            if (mStatus & (1<<9)) {
                printf(", PosLim");
                numChars += 8;
            }
            if (mStatus & (1<<10)) {
                printf(", NegLim");
                numChars += 8;
            }
            if (mStatus & (1<<27)) {
                printf(", Moving");
                numChars += 8;
            }
            if (numChars < numCharsPrev) {
                for (unsigned int i = numChars; i < numCharsPrev; i++)
                    printf(" ");
            }
            numCharsPrev = numChars;
            printf("\r");
        }
        else {
            printf("Copley not connected\r");
        }

        osaSleep(0.01);  // to avoid taking too much CPU time
    }

    void Cleanup() {}

};

int main(int argc, char **argv)
{
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    // Using SendStatus, SendWarning and SendError instead
    // cmnLogger::AddChannel(std::cout, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    if (argc < 2) {
        std::cout << "Syntax: sawCopleyConsole <config>" << std::endl;
        std::cout << "        <config>      Configuration file (JSON format)" << std::endl;
        return 0;
    }

    std::cout << "Starting mtsCopleyController" << std::endl;
    mtsCopleyController *copleyServer;
    copleyServer = new mtsCopleyController("copleyServer");
    copleyServer->Configure(argv[1]);

    mtsComponentManager * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(copleyServer);

    CopleyClient client;
    componentManager->AddComponent(&client);

    if (!componentManager->Connect(client.GetName(), "Input", copleyServer->GetName(), "control")) {
        std::cout << "Failed to connect: "
            << client.GetName() << "::Input to "
            << copleyServer->GetName() << "::control" << std::endl;
        delete copleyServer;
        return -1;
    }

    componentManager->CreateAll();
    componentManager->StartAll();

    // Main thread passed to client task

    copleyServer->Kill();
    componentManager->WaitForStateAll(mtsComponentState::FINISHED, 2.0 * cmn_s);

    componentManager->Cleanup();

    // stop all logs
    cmnLogger::SetMask(CMN_LOG_ALLOW_NONE);
    delete copleyServer;

    return 0;
}
