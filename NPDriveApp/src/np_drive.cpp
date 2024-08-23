#include <asynOctetSyncIO.h>
#include <cstdio>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <iostream>

#include "np_drive.hpp"
#include "rpc.hpp"

constexpr int NUM_PARAMS = 0;
constexpr double DRIVER_RESOLUTION = 1e9;

NPDriveMotorController::NPDriveMotorController(const char *portName,
                                               const char *NPDriveMotorPortName, int numAxes,
                                               double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_PARAMS,
                          0, // No additional interfaces beyond the base class
                          0, // No additional callback interfaces beyond those in base class
                          ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                          1,    // autoconnect
                          0, 0) // Default priority and stack size
{
    asynStatus status;
    int axis;
    NPDriveMotorAxis *pAxis;
    static const char *functionName = "NPDriveMotorController::NPDriveMotorController";

    createParam(DRIVE_FREQUENCY_STRING, asynParamInt32, &driveFrequencyIndex_);
    createParam(DRIVE_AMPLITUDE_STRING, asynParamInt32, &driveAmplitudeIndex_);

    // Connect to motor controller
    status = pasynOctetSyncIO->connect(NPDriveMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: cannot connect to NPDrive motor controller\n", functionName);
    }

    // // Only 2 axes are supported
    // if (numAxes > 2) {
    // asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Requested %d axes but only 2 are
    // supported\n", numAxes);
    // }

    // Create NPDriveMotorAxis object for each axis
    // if not done here, user must call NPDriveMotorCreateAxis from cmd file
    for (axis = 0; axis < numAxes; axis++) {
        pAxis = new NPDriveMotorAxis(this, axis);
    }

    std::cout << "movingPollPeriod = " << movingPollPeriod << std::endl;
    std::cout << "idlePollPeriod = " << idlePollPeriod << std::endl;
    startPoller(movingPollPeriod, idlePollPeriod, 2);
}

extern "C" int NPDriveMotorCreateController(const char *portName, const char *NPDriveMotorPortName,
                                            int numAxes, int movingPollPeriod, int idlePollPeriod) {
    NPDriveMotorController *pNPDriveMotorController = new NPDriveMotorController(
        portName, NPDriveMotorPortName, numAxes, movingPollPeriod / 1000., idlePollPeriod / 1000.);
    pNPDriveMotorController = NULL;
    return (asynSuccess);
}

void NPDriveMotorController::report(FILE *fp, int level) {
    // "dbior" from iocsh can be useful to see what's going on here
    fprintf(fp, "NPDrive Motor Controller driver %s\n", this->portName);
    fprintf(fp, "    numAxes=%d\n", numAxes_);
    fprintf(fp, "    moving poll period=%f\n", movingPollPeriod_);
    fprintf(fp, "    idle poll period=%f\n", idlePollPeriod_);

    // Call the base class method
    asynMotorController::report(fp, level);
}

NPDriveMotorAxis *NPDriveMotorController::getAxis(asynUser *pasynUser) {
    return static_cast<NPDriveMotorAxis *>(asynMotorController::getAxis(pasynUser));
}

NPDriveMotorAxis *NPDriveMotorController::getAxis(int axisNo) {
    return static_cast<NPDriveMotorAxis *>(asynMotorController::getAxis(axisNo));
}

NPDriveMotorAxis::NPDriveMotorAxis(NPDriveMotorController *pC, int axisNo)
    : asynMotorAxis(pC, axisNo), pC_(pC) {

    axisIndex_ = axisNo + 1;
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "NPDriveMotorAxis created with axis index %d\n",
              axisIndex_);

    // Gain Support is required for setClosedLoop to be called
    setIntegerParam(pC->motorStatusHasEncoder_, 1);
    setIntegerParam(pC->motorStatusGainSupport_, 1);

    callParamCallbacks();
}

void NPDriveMotorAxis::report(FILE *fp, int level) {
    if (level > 0) {
        fprintf(fp, " Axis #%d\n", axisNo_);
        fprintf(fp, " axisIndex_=%d\n", axisIndex_);
    }
    asynMotorAxis::report(fp, level);
}

template <typename T> T parse_json_result(const std::string &in_string) {
    // '}\n' must be input terminator for asyn, so we add the '}' back
    std::string in_string_fix = in_string;
    in_string_fix.push_back('}');
    return json::parse(in_string_fix)["result"].template get<T>();
}

asynStatus NPDriveMotorAxis::stop(double acceleration) {

    asynStatus asyn_status = asynSuccess;

    // stop both closed and open loop motion
    sprintf(pC_->outString_, "%s", NPDriveCmd::stop_positioning().c_str());
    asyn_status = pC_->writeReadController();
    sprintf(pC_->outString_, "%s", NPDriveCmd::stop_motion().c_str());
    asyn_status = pC_->writeReadController();

    callParamCallbacks();
    return asyn_status;
}

asynStatus NPDriveMotorAxis::move(double position, int relative, double min_velocity,
                                  double max_velocity, double acceleration) {
    
    asynStatus asyn_status = asynSuccess;

    std::string cmd_string;
    int frequency = 0;
    int amplitude = 0;
    pC_->getIntegerParam(pC_->driveFrequencyIndex_, &frequency);
    pC_->getIntegerParam(pC_->driveAmplitudeIndex_, &amplitude);

    // For now, only closed loop motion is supported through the motor record.
    // Open loop motion is available through additional asyn parameters
    cmd_string = NPDriveCmd::go_position(axisIndex_, position / DRIVER_RESOLUTION, amplitude, frequency);
    sprintf(pC_->outString_, "%s", cmd_string.c_str());
    asyn_status = pC_->writeReadController();
   
    if (not parse_json_result<bool>(pC_->inString_)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Move command failed\n");
    }

    callParamCallbacks();
    return asyn_status;
}

asynStatus NPDriveMotorAxis::home(double minVelocity, double maxVelocity, double acceleration,
                                  int forwards) {
    asynStatus asyn_status = asynSuccess;

    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "NPDriveMotorAxis::home() called!\n");

    callParamCallbacks();
    return asyn_status;
}

asynStatus NPDriveMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;
    std::string cmd_string;
    std::string in_string;
    long long_position_nm = 0;
    double position_m = 0.0;
    int open_loop_done = 1;
    int closed_loop_done = 1;
    int done = 1;

    // Read axis position
    cmd_string = NPDriveCmd::get_position(axisIndex_);
    sprintf(pC_->outString_, "%s", cmd_string.c_str());
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    position_m = parse_json_result<double>(pC_->inString_);       // meters
    long_position_nm = DRIVER_RESOLUTION * position_m;            // convert to nanometer "steps"
    setDoubleParam(pC_->motorPosition_, long_position_nm);        // RRBV [nanometers]
    setDoubleParam(pC_->motorEncoderPosition_, long_position_nm); // RRBV [nanometers]

    // Check if open-loop command is done
    cmd_string = NPDriveCmd::get_status_drive_busy();
    sprintf(pC_->outString_, "%s", cmd_string.c_str());
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    open_loop_done = not parse_json_result<int>(pC_->inString_);

    // Check if closed-loop command is done
    cmd_string = NPDriveCmd::get_status_positioning();
    sprintf(pC_->outString_, "%s", cmd_string.c_str());
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    closed_loop_done = not parse_json_result<int>(pC_->inString_);

    // done when both closed and open loop processes are done
    done = (open_loop_done & closed_loop_done);
    setIntegerParam(pC_->motorStatusDone_, done);
    setIntegerParam(pC_->motorStatusMoving_, not done);
    *moving = not done;

skip:
    setIntegerParam(pC_->motorStatusProblem_, asyn_status ? 1 : 0);
    callParamCallbacks();
    return asyn_status ? asynError : asynSuccess;
}

asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value) {
    asynStatus asyn_status = asynSuccess;

    int function = pasynUser->reason;

    return asyn_status;
}

asynStatus NPDriveMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus asyn_status = asynSuccess;

    callParamCallbacks();
    return asyn_status;
}

// ==================
// iosch registration
// ==================

static const iocshArg NPDriveMotorCreateControllerArg0 = {"asyn port name", iocshArgString};
static const iocshArg NPDriveMotorCreateControllerArg1 = {"Controller port name", iocshArgString};
static const iocshArg NPDriveMotorCreateControllerArg2 = {"Number of axes", iocshArgInt};
static const iocshArg NPDriveMotorCreateControllerArg3 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg NPDriveMotorCreateControllerArg4 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg *const NPDriveMotorCreateControllerArgs[] = {
    &NPDriveMotorCreateControllerArg0, &NPDriveMotorCreateControllerArg1,
    &NPDriveMotorCreateControllerArg2, &NPDriveMotorCreateControllerArg3,
    &NPDriveMotorCreateControllerArg4};
static const iocshFuncDef NPDriveMotorCreateControllerDef = {"NPDriveMotorCreateController", 5,
                                                             NPDriveMotorCreateControllerArgs};

static void NPDriveMotorCreateControllerCallFunc(const iocshArgBuf *args) {
    NPDriveMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival,
                                 args[4].ival);
}

static void NPDriveMotorRegister(void) {
    iocshRegister(&NPDriveMotorCreateControllerDef, NPDriveMotorCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(NPDriveMotorRegister);
}
