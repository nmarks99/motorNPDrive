#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"

static constexpr char FREQUENCY_STRING[] = "FREQUENCY";
static constexpr char AMPLITUDE_STRING[] = "AMPLITUDE";
static constexpr char STOP_LIMIT_STRING[] = "STOP_LIMIT";
static constexpr char HOLD_POSITION_STRING[] = "HOLD_POSITION";
static constexpr char HOLD_POSITION_TARGET_STRING[] = "HOLD_POSITION_TARGET";
static constexpr char HOLD_POSITION_TIMEOUT_STRING[] = "HOLD_POSITION_TIMEOUT";
static constexpr char SET_SENSORS_OFF_STRING[] = "SET_SENSORS_OFF";
static constexpr char SET_DRIVE_CHANNELS_OFF_STRING[] = "SET_DRIVE_CHANNELS_OFF";

static constexpr char GO_STEPS_FORWARD_STRING[] = "GO_STEPS_FORWARD";
static constexpr char GO_STEPS_REVERSE_STRING[] = "GO_STEPS_REVERSE";
static constexpr char OPEN_LOOP_STEPS_STRING[] = "OPEN_LOOP_STEPS";

static constexpr char GO_CONTINUOUS_FORWARD_STRING[] = "GO_CONTINUOUS_FORWARD";
static constexpr char GO_CONTINUOUS_REVERSE_STRING[] = "GO_CONTINUOUS_REVERSE";

class epicsShareClass NPDriveMotorAxis : public asynMotorAxis {
  public:
    NPDriveMotorAxis(class NPDriveMotorController *pC, int axisNo);
    void report(FILE *fp, int level);
    asynStatus stop(double acceleration);
    asynStatus move(double position, int relative, double min_velocity, double max_velocity, double acceleration);
    asynStatus poll(bool *moving);
    // asynStatus setClosedLoop(bool closedLoop);

  private:
    NPDriveMotorController *pC_;
    int axisIndex_;
    int amplitude_ = 0;
    int frequency_ = 0;
    double hold_target_ = 0.0;
    int hold_timeout_ = 0;
    double stop_limit_ = 0.0;
    int cmd_steps_ = 0; 
    

    friend class NPDriveMotorController;
};

class epicsShareClass NPDriveMotorController : public asynMotorController {
  public:
    /// \brief Create a new NPDriveMotorController object
    ///
    /// \param[in] portName             The name of the asyn port that will be created for this
    /// driver \param[in] NPDrivePortName        The name of the drvAsynIPPort that was created
    /// previously \param[in] numAxes              The number of axes that this controller supports
    /// \param[in] movingPollPeriod     The time between polls when any axis is moving
    /// \param[in] idlePollPeriod       The time between polls when no axis is moving
    NPDriveMotorController(const char *portName, const char *NPDriveMotorController, int numAxes,
                           double movingPollPeriod, double idlePollPeriod);
    void report(FILE *fp, int level);
    NPDriveMotorAxis *getAxis(asynUser *pasynUser);
    NPDriveMotorAxis *getAxis(int axisNo);
    asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
    asynStatus poll();

  private:
    int motion_done_ = 1;
    int num_axes_ = 0;
    int drive_channel_ = 0;

  protected:
#define FIRST_NPDRIVE_PARAM driveFrequencyIndex_
    int driveFrequencyIndex_;
    int driveAmplitudeIndex_;
    int stopLimitIndex_;
    int holdPositionIndex_;
    int holdPositionTargetIndex_;
    int holdPositionTimeoutIndex_;
    int setSensorsOffIndex_;
    int setDriveChannelsOffIndex_;
    int goStepsForwardIndex_;
    int goStepsReverseIndex_;
    int openLoopStepsIndex_;
    int goContinuousForwardIndex_;
    int goContinuousReverseIndex_;
#define LAST_NPDRIVE_PARAM goContinuousReverseIndex_

    friend class NPDriveMotorAxis;
};

#define NUM_NPDRIVE_PARAMS ((int)(&LAST_NPDRIVE_PARAM - &FIRST_NPDRIVE_PARAM + 1))
