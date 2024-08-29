#include <asynOctetSyncIO.h>
#include <cstdio>
#include <epicsExport.h>
#include <epicsThread.h>
#include <iocsh.h>
#include <iostream>

#include "np_drive.hpp"
#include "rpc.hpp"

constexpr double DRIVER_RESOLUTION = 1e9;

NPDriveMotorController::NPDriveMotorController(const char *portName,
                                               const char *NPDriveMotorPortName, int numAxes,
                                               double movingPollPeriod, double idlePollPeriod)
    : asynMotorController(portName, numAxes, NUM_NPDRIVE_PARAMS,
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

    createParam(FREQUENCY_STRING, asynParamInt32, &driveFrequencyIndex_);
    createParam(AMPLITUDE_STRING, asynParamInt32, &driveAmplitudeIndex_);
    createParam(HOLD_POSITION_STRING, asynParamInt32, &holdPositionIndex_);
    createParam(HOLD_POSITION_TIMEOUT_STRING, asynParamInt32, &holdPositionTimeoutIndex_);
    createParam(SET_SENSORS_OFF_STRING, asynParamInt32, &setSensorsOffIndex_);
    createParam(SET_DRIVE_CHANNELS_OFF_STRING, asynParamInt32, &setDriveChannelsOffIndex_);
    createParam(GO_STEPS_FORWARD_STRING, asynParamInt32, &goStepsForwardIndex_);
    createParam(GO_STEPS_REVERSE_STRING, asynParamInt32, &goStepsReverseIndex_);
    createParam(OPEN_LOOP_STEPS_STRING, asynParamInt32, &openLoopStepsIndex_);
    createParam(GO_CONTINUOUS_FORWARD_STRING, asynParamInt32, &goContinuousForwardIndex_);
    createParam(GO_CONTINUOUS_REVERSE_STRING, asynParamInt32, &goContinuousReverseIndex_);
    createParam(STOP_LIMIT_STRING, asynParamFloat64, &stopLimitIndex_);
    createParam(HOLD_POSITION_TARGET_STRING, asynParamFloat64, &holdPositionTargetIndex_);

    // Connect to motor controller
    status = pasynOctetSyncIO->connect(NPDriveMotorPortName, 0, &pasynUserController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: cannot connect to NPDrive motor controller\n", functionName);
    }

    // Only 3 axes are supported
    if (numAxes > 3) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "Requested %d axes but only 3 are supported\n", numAxes);
    }

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

// returns the "result" field of the JSON string from the controller and as the specified type
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

    // For now, only closed loop motion is supported through the motor record
    sprintf(pC_->outString_, "%s",
            NPDriveCmd::go_position(axisIndex_, position / DRIVER_RESOLUTION, this->amplitude_,
                                    this->frequency_)
                .c_str());
    asyn_status = pC_->writeReadController();

    if (not parse_json_result<bool>(pC_->inString_)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Move command failed\n");
        std::cout << "Read: " << pC_->inString_ << std::endl;
    }

    callParamCallbacks();
    return asyn_status;
}

asynStatus NPDriveMotorAxis::poll(bool *moving) {
    asynStatus asyn_status = asynSuccess;
    long long_position_nm = 0;
    double position_m = 0.0;
    int open_loop_done = 1;
    int closed_loop_done = 1;
    int done = 1;

    // Read axis position
    sprintf(pC_->outString_, "%s", NPDriveCmd::get_position(axisIndex_).c_str());
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    position_m = parse_json_result<double>(pC_->inString_); // meters
    long_position_nm = DRIVER_RESOLUTION * position_m;      // convert to nanometer "steps"
    setDoubleParam(pC_->motorPosition_, long_position_nm);  // RRBV [nanometers]
    setDoubleParam(pC_->motorEncoderPosition_, long_position_nm);

    // Check if open-loop command is done
    sprintf(pC_->outString_, "%s", NPDriveCmd::get_status_drive_busy().c_str());
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    open_loop_done = not parse_json_result<int>(pC_->inString_);

    // Check if closed-loop command is done
    sprintf(pC_->outString_, "%s", NPDriveCmd::get_status_positioning().c_str());
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

    // Check if drive overloaded
    sprintf(pC_->outString_, "%s", NPDriveCmd::get_status_drive_overload().c_str());
    asyn_status = pC_->writeReadController();
    if (asyn_status) {
        goto skip;
    }
    if (parse_json_result<bool>(pC_->inString_)) {
        asynPrint(pC_->pasynUserSelf, ASYN_TRACE_ERROR, "Drive overloaded!\n");
    }

skip:
    setIntegerParam(pC_->motorStatusProblem_, asyn_status ? 1 : 0);
    callParamCallbacks();
    return asyn_status ? asynError : asynSuccess;
}

asynStatus NPDriveMotorController::writeInt32(asynUser *pasynUser, epicsInt32 value) {

    asynStatus asyn_status = asynSuccess;
    int function = pasynUser->reason;
    NPDriveMotorAxis *pAxis;

    pAxis = getAxis(pasynUser);
    if (!pAxis) {
        return asynError;
    }

    if (function == driveAmplitudeIndex_) {
        pAxis->amplitude_ = value;
    } else if (function == driveFrequencyIndex_) {
        pAxis->frequency_ = value;
    }

    // TODO: Test
    else if (function == holdPositionIndex_) {
        asynPrint(this->pasynUserSelf, ASYN_REASON_SIGNAL,
                  "Holding position %e meters for %d seconds\n",
                  pAxis->hold_target_ / DRIVER_RESOLUTION, pAxis->hold_timeout_);

        sprintf(this->outString_, "%s",
                NPDriveCmd::hold_position(
                    pAxis->axisIndex_,
                    pAxis->hold_target_ / DRIVER_RESOLUTION,
                    pAxis->amplitude_,
                    pAxis->hold_timeout_
                ).c_str());
        asyn_status = this->writeReadController();
        if (asyn_status) {
            goto skip;
        }
        if (not parse_json_result<bool>(this->inString_)) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Error enabling hold position mode\n");
        }
    } else if (function == holdPositionTimeoutIndex_) {
        pAxis->hold_timeout_ = value;
    }

    else if (function == openLoopStepsIndex_) {
        pAxis->cmd_steps_ = value;
    }

    else if (function == goStepsForwardIndex_) {
        sprintf(this->outString_, "%s",
                NPDriveCmd::go_steps_forward(
                    pAxis->axisIndex_,
                    pAxis->cmd_steps_,
                    pAxis->amplitude_,
                    pAxis->frequency_
                ).c_str());
        asyn_status = this->writeReadController();
        if (asyn_status) {
            goto skip;
        }
        if (not parse_json_result<bool>(this->inString_)) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Error in open loop step forward\n");
        }
    }

    else if (function == goStepsReverseIndex_) {
        sprintf(this->outString_, "%s",
                NPDriveCmd::go_steps_reverse(
                    pAxis->axisIndex_,
                    pAxis->cmd_steps_,
                    pAxis->amplitude_,
                    pAxis->frequency_
                ).c_str());
        asyn_status = this->writeReadController();
        if (asyn_status) {
            goto skip;
        }
        if (not parse_json_result<bool>(this->inString_)) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Error in open loop step reverse\n");
        }
    }
    

    // FIX: json error when parsing result?
    //
    // else if (function == goContinuousForwardIndex_) {
        // sprintf(this->outString_, "%s",
                // NPDriveCmd::go_continuous_forward(
                    // pAxis->axisIndex_,
                    // pAxis->amplitude_,
                    // pAxis->frequency_
                // ).c_str());
        // asyn_status = this->writeReadController();
        // if (asyn_status) {
            // goto skip;
        // }
        // if (not parse_json_result<bool>(this->inString_)) {
            // asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Error in jog forward\n");
        // }
    // }

    // else if (function == goContinuousReverseIndex_) {
        // sprintf(this->outString_, "%s",
                // NPDriveCmd::go_continuous_reverse(
                    // pAxis->axisIndex_,
                    // pAxis->amplitude_,
                    // pAxis->frequency_
                // ).c_str());
        // asyn_status = this->writeReadController();
        // if (asyn_status) {
            // goto skip;
        // }
        // if (not parse_json_result<bool>(this->inString_)) {
            // asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Error in jog reverse\n");
        // }
    // }

    // TODO: Test
    else if (function == setSensorsOffIndex_) {
        asynPrint(this->pasynUserSelf, ASYN_REASON_SIGNAL, "Setting all sensors off\n");
        sprintf(this->outString_, "%s", NPDriveCmd::set_sensors_off().c_str());
        asyn_status = this->writeReadController();
        if (asyn_status) {
            goto skip;
        }
        if (not parse_json_result<bool>(this->inString_)) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Error turning off sensors\n");
        }
    }

    // TODO: Test
    else if (function == setDriveChannelsOffIndex_) {
        asynPrint(this->pasynUserSelf, ASYN_REASON_SIGNAL, "Setting all drive channels off\n");
        sprintf(this->outString_, "%s", NPDriveCmd::set_drive_channels_off().c_str());
        asyn_status = this->writeReadController();
        if (asyn_status) {
            goto skip;
        }
        if (not parse_json_result<bool>(this->inString_)) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Error turning off drive channels\n");
        }
    }

    // Call base class method
    else {
        asyn_status = asynMotorController::writeInt32(pasynUser, value);
    }

skip:
    pAxis->callParamCallbacks();
    return asyn_status;
}

asynStatus NPDriveMotorController::writeFloat64(asynUser *pasynUser, epicsFloat64 value) {

    int function = pasynUser->reason;
    NPDriveMotorAxis *pAxis;
    asynStatus asyn_status = asynSuccess;
    pAxis = getAxis(pasynUser);
    if (!pAxis) {
        return asynError;
    }

    if (function == holdPositionTargetIndex_) {
        pAxis->hold_target_ = value;
    } else if (function == stopLimitIndex_) {
        double mres = 0.0;
        this->getDoubleParam(this->motorRecResolution_, &mres);
        pAxis->stop_limit_ = (value / mres) / DRIVER_RESOLUTION;
        asynPrint(this->pasynUserSelf, ASYN_REASON_SIGNAL, "Setting stop limit to %e meters\n",
                  pAxis->stop_limit_);
        sprintf(this->outString_, "%s",
                NPDriveCmd::set_stop_limit(pAxis->axisIndex_, pAxis->stop_limit_).c_str());
        asyn_status = this->writeReadController();
        if (asyn_status) {
            goto skip;
        }
        if (not parse_json_result<bool>(this->inString_)) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "Error setting stop limit\n");
        }
    }

    // Call base class method
    else {
        asyn_status = asynMotorController::writeFloat64(pasynUser, value);
    }

skip:
    // Do callbacks so higher layers see any changes
    pAxis->callParamCallbacks();
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
