#include <asynOctetSyncIO.h>
#include <cstdio>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <iostream>

#include "np_drive.hpp"
#include "rpc.hpp"

constexpr int NUM_PARAMS = 0;

NPDriveMotorController::NPDriveMotorController(const char *portName, const char *NPDriveMotorPortName, int numAxes,
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

    // Connect to motor controller
    status = pasynOctetSyncIO->connect(NPDriveMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(
            this->pasynUserSelf,
            ASYN_TRACE_ERROR, "%s: cannot connect to NPDrive motor controller\n",
            functionName
        );
    }
    
    // // Only 2 axes are supported
    // if (numAxes > 2) {
        // asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Requested %d axes but only 2 are supported\n",
                  // numAxes);
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

extern "C" int NPDriveMotorCreateController(const char *portName, const char *NPDriveMotorPortName, int numAxes,
                                          int movingPollPeriod, int idlePollPeriod) {
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

NPDriveMotorAxis::NPDriveMotorAxis(NPDriveMotorController *pC, int axisNo) : asynMotorAxis(pC, axisNo), pC_(pC) {

    axisIndex_ = axisNo + 1;
    asynPrint(pasynUser_, ASYN_REASON_SIGNAL, "NPDriveMotorAxis created with axis index %d\n", axisIndex_);

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

asynStatus NPDriveMotorAxis::stop(double acceleration) {

    asynStatus asyn_status = asynSuccess;
    
    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "NPDriveMotorAxis::stop() called!\n");

    callParamCallbacks();
    return asyn_status;
}

asynStatus NPDriveMotorAxis::move(double position, int relative, double min_velocity, double max_velocity,
                                double acceleration) {

    asynStatus asyn_status = asynSuccess;

    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "NPDriveMotorAxis::move() called!\n");

    callParamCallbacks();
    return asyn_status;
}

asynStatus NPDriveMotorAxis::home(double minVelocity, double maxVelocity, double acceleration, int forwards) {
    asynStatus asyn_status = asynSuccess;

    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "NPDriveMotorAxis::home() called!\n");

    callParamCallbacks();
    return asyn_status;
}


template <typename T>
T parse_json_result(const std::string &in_string) {
    // '}\n' must be input terminator for asyn, so we add the '}' back
    std::string in_string_fix = in_string;
    in_string_fix.push_back('}');
    const json response_json = json::parse(in_string_fix);
    return response_json["result"].template get<T>();
}

asynStatus NPDriveMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;
    std::string cmd_string;
    std::string in_string;
    long long_position_nm = 0;
    double position_m = 0.0;
    // int done = 1;
    
    // Send command to read axis position
    cmd_string = NPDriveCmd::get_position(axisIndex_);
    sprintf(pC_->outString_, "%s", cmd_string.c_str());
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    position_m = parse_json_result<double>(pC_->inString_);
    
    // Convert to nanometer "steps"
    long_position_nm = 1e9 * position_m;
    setDoubleParam(pC_->motorPosition_, long_position_nm); // RRBV [nanometers]
    
    // Check if open-loop command is busy
    cmd_string = NPDriveCmd::get_status_drive_busy();
    sprintf(pC_->outString_, "%s", cmd_string.c_str());
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    // done = not parse_json_result<int>(in_string);
    // setIntegerParam(pC_->motorStatusDone_, done);
    // setIntegerParam(pC_->motorStatusMoving_, not done);

    skip: 
        setIntegerParam(pC_->motorStatusProblem_, asyn_status ? 1:0);
        callParamCallbacks();
        return asyn_status ? asynError : asynSuccess;
}


asynStatus NPDriveMotorAxis::setClosedLoop(bool closedLoop) {
    asynStatus asyn_status = asynSuccess;

    asynPrint(pC_->pasynUserSelf, ASYN_REASON_SIGNAL, "NPDriveMotorAxis::setClosedLoop() called!\n");

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
    &NPDriveMotorCreateControllerArg0, &NPDriveMotorCreateControllerArg1, &NPDriveMotorCreateControllerArg2,
    &NPDriveMotorCreateControllerArg3, &NPDriveMotorCreateControllerArg4};
static const iocshFuncDef NPDriveMotorCreateControllerDef = {"NPDriveMotorCreateController", 5,
                                                           NPDriveMotorCreateControllerArgs};

static void NPDriveMotorCreateControllerCallFunc(const iocshArgBuf *args) {
    NPDriveMotorCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].ival);
}

static void NPDriveMotorRegister(void) {
    iocshRegister(&NPDriveMotorCreateControllerDef, NPDriveMotorCreateControllerCallFunc);
}

extern "C" {
epicsExportRegistrar(NPDriveMotorRegister);
}
