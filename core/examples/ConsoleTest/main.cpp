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
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsManagerLocal.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstMultiTask/mtsInterfaceRequired.h>

#include <sawCopleyController/mtsCopleyController.h>

class CopleyClient : public mtsTaskMain {

private:
    size_t NumAxes;
    prmConfigurationJoint m_config_js;
    prmStateJoint m_measured_js;
    prmPositionJointSet jtposSet;

    vctDoubleVec jtScale;
    std::vector<std::string> jtUnits;
    vctDoubleVec jtgoal;
    vctDoubleVec jtpos;
    vctLongVec mPosRaw;
    vctLongVec mStatus;
    unsigned int numCharsPrev;
    vctDoubleVec jtSpeed;
    vctDoubleVec jtAccel;
    vctDoubleVec jtDecel;

    mtsFunctionRead get_config_js;
    mtsFunctionRead measured_js;

    mtsFunctionRead GetPositionRaw; // Position in encoder counts
    mtsFunctionRead GetStatus;      // Drive status
    mtsFunctionWrite move_jp;
    mtsFunctionWrite move_jr;
    mtsFunctionRead GetConnected;   // Whether serial port is open
    mtsFunctionWriteReturn SendCommandRet;
    mtsFunctionWrite SetSpeed;
    mtsFunctionRead GetSpeed;
    mtsFunctionWrite SetAccel;
    mtsFunctionRead GetAccel;
    mtsFunctionWrite SetDecel;
    mtsFunctionRead GetDecel;
    mtsFunctionVoid crtk_enable;
    mtsFunctionVoid crtk_disable;
    mtsFunctionVoid Home;

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

    CopleyClient() : mtsTaskMain("CopleyClient"), NumAxes(0), numCharsPrev(0)
    {
        mtsInterfaceRequired *req = AddInterfaceRequired("Input", MTS_OPTIONAL);
        if (req) {
            req->AddFunction("measured_js", measured_js);
            req->AddFunction("GetPositionRaw", GetPositionRaw);
            req->AddFunction("GetStatus", GetStatus);
            req->AddFunction("move_jp", move_jp);
            req->AddFunction("move_jr", move_jr);
            req->AddFunction("SendCommandRet", SendCommandRet);
            req->AddFunction("configuration_js", get_config_js);
            req->AddFunction("SetSpeed", SetSpeed);
            req->AddFunction("GetSpeed", GetSpeed);
            req->AddFunction("SetAccel", SetAccel);
            req->AddFunction("GetAccel", GetAccel);
            req->AddFunction("SetDecel", SetDecel);
            req->AddFunction("GetDecel", GetDecel);
            req->AddFunction("EnableMotorPower", crtk_enable);
            req->AddFunction("DisableMotorPower", crtk_disable);
            req->AddFunction("Home", Home);
            req->AddEventHandlerWrite(&CopleyClient::OnStatusEvent, this, "status");
            req->AddEventHandlerWrite(&CopleyClient::OnWarningEvent, this, "warning");
            req->AddEventHandlerWrite(&CopleyClient::OnErrorEvent, this, "error");
        }
    }

    void Configure(const std::string&) {}

    void PrintHelp()
    {
        std::cout << "Available commands:" << std::endl
                  << "  m: absolute position move (move_jp)" << std::endl
                  << "  r: relative position move (move_jr)" << std::endl
                  << "  s: set speed" << std::endl
                  << "  a: set accel" << std::endl
                  << "  d: set decel" << std::endl
                  << "  c: send command" << std::endl
                  << "  h: display help information" << std::endl
                  << "  e: enable motor power" << std::endl
                  << "  n: disable motor power" << std::endl
                  << "  z: home robot" << std::endl
                  << "  q: quit" << std::endl;
    }

    void Startup()
    {
        NumAxes = 0;
        const mtsGenericObject *p = measured_js.GetArgumentPrototype();
        const prmStateJoint *psj = dynamic_cast<const prmStateJoint *>(p);
        if (psj) NumAxes = psj->Position().size();
        std::cout << "CopleyClient: Detected " << NumAxes << " axes" << std::endl;

        jtgoal.SetSize(NumAxes);
        jtpos.SetSize(NumAxes);
        jtScale.SetSize(NumAxes);
        jtUnits.resize(NumAxes);
        jtposSet.Goal().SetSize(NumAxes);
        jtSpeed.SetSize(NumAxes);
        jtAccel.SetSize(NumAxes);
        jtDecel.SetSize(NumAxes);

        // Get joint configuration
        get_config_js(m_config_js);
        // Set jtScale based on joint type (prismatic or revolute)
        for (size_t i = 0; i < NumAxes; i++) {
            if (m_config_js.Type()[i] == PRM_JOINT_PRISMATIC) {
                jtScale[i] = 1000.0;     // meters --> millimeters
                jtUnits[i].assign("mm");
            }
            else if (m_config_js.Type()[i] == PRM_JOINT_REVOLUTE) {
                jtScale[i] = cmn180_PI;  // radians --> degrees
                jtUnits[i].assign("deg");
            }
            else {
                std::cout << "CopleyClient: joint " << i << " is unknown type ("
                          << m_config_js.Type()[i] << ")" << std::endl;
                jtScale[i] = 1.0;
                jtUnits[i].assign("cnts");
            }
        }

        PrintHelp();
    }

    void Run() {

        ProcessQueuedEvents();

        bool copleyOK;
        GetConnected(copleyOK);

        if (copleyOK) {
            GetPositionRaw(mPosRaw);
            measured_js(m_measured_js);
            m_measured_js.GetPosition(jtpos);
            GetStatus(mStatus);
        }

        int i;
        char c = 0;
        if (cmnKbHit()) {
            c = cmnGetChar();
            switch (c) {

            case 'm':   // position move joint
                std::cout << std::endl << "Enter absolute position";
                if (NumAxes == 1)
                    std::cout << " (" << jtUnits[0] << ")";
                std::cout << ": ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtgoal[i];
                std::cout << "Moving to " << jtgoal << std::endl;
                jtgoal.ElementwiseDivide(jtScale);
                jtposSet.SetGoal(jtgoal);
                move_jp(jtposSet);
                break;

            case 'r':   // relative move joint
                std::cout << std::endl << "Enter relative position";
                if (NumAxes == 1)
                    std::cout << " (" << jtUnits[0] << ")";
                std::cout << ": ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtgoal[i];
                std::cout << "Relative move by " << jtgoal << std::endl;
                jtgoal.ElementwiseDivide(jtScale);
                jtposSet.SetGoal(jtgoal);
                move_jr(jtposSet);
                break;

            case 's':  // set speed
                GetSpeed(jtSpeed);
                jtSpeed.ElementwiseMultiply(jtScale);
                std::cout << std::endl << "Current speed: " << jtSpeed;
                if (NumAxes == 1)
                    std::cout << " " << jtUnits[0] << "/s";
                std::cout << std::endl << "Enter new speed";
                if (NumAxes == 1)
                    std::cout << " (" << jtUnits[0] << "/s)";
                std::cout << ": ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtSpeed[i];
                jtSpeed.ElementwiseDivide(jtScale);
                SetSpeed(jtSpeed);
                break;

            case 'a':  // set accel
                GetAccel(jtAccel);
                jtAccel.ElementwiseMultiply(jtScale);
                std::cout << std::endl << "Current accel: " << jtAccel;
                if (NumAxes == 1)
                    std::cout << " " << jtUnits[0] << "/s^2";
                std::cout << std::endl << "Enter new accel";
                if (NumAxes == 1)
                    std::cout << " (" << jtUnits[0] << "/s^2)";
                std::cout << ": ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtAccel[i];
                jtAccel.ElementwiseDivide(jtScale);
                SetAccel(jtAccel);
                break;

            case 'd':  // set decel
                GetDecel(jtDecel);
                jtDecel.ElementwiseMultiply(jtScale);
                std::cout << std::endl << "Current decel: " << jtDecel;
                if (NumAxes == 1)
                    std::cout << " " << jtUnits[0] << "/s^2";
                std::cout << std::endl << "Enter new decel";
                if (NumAxes == 1)
                    std::cout << " (" << jtUnits[0] << "/s^2)";
                std::cout << ": ";
                for (i = 0; i < NumAxes; i++)
                    std::cin >> jtDecel[i];
                jtDecel.ElementwiseDivide(jtScale);
                SetDecel(jtDecel);
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

            case 'e':   // enable motor power
                crtk_enable();
                break;

            case 'n':   // disable motor power
                crtk_disable();
                break;

            case 'z':
                if (copleyOK) {
                    std::cout << std::endl << "Homing" << std::endl;
                    Home();
                }
                else {
                    std::cout << std::endl << "Command not available - Copley not connected" << std::endl;
                }
                break;

            case 'q':   // quit program
                std::cout << std::endl << "Exiting.. " << std::endl;
                this->Kill();
                break;

            }
        }

        if (copleyOK) {
            unsigned int axis;
            printf("Pos:");
            for (axis = 0; axis < NumAxes; axis++)
                printf(" %8.2lf (%08lx)", jtpos[axis]*jtScale[axis], mPosRaw[axis]);
            printf(", Status:");
            unsigned int numChars = 0;
            for (axis = 0; axis < NumAxes; axis++) {
                printf(" %08lx", mStatus[axis]);
                if (mStatus[axis] & (1<<9)) {
                    printf(", PosLim");
                    numChars += 8;
                }
                if (mStatus[axis] & (1<<10)) {
                    printf(", NegLim");
                    numChars += 8;
                }
                if (mStatus[axis] & (1<<12)) {
                    printf(", Disabled");
                    numChars += 10;
                }
                if (mStatus[axis] & (1<<26)) {
                    printf(", Home");
                    numChars += 6;
                }
                if (mStatus[axis] & (1<<27)) {
                    printf(", Moving");
                    numChars += 8;
                }
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
