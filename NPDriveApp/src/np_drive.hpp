#include "asynDriver.h"
#include "asynMotorAxis.h"
#include "asynMotorController.h"

static constexpr char DRIVE_FREQUENCY_STRING[] = "DRIVE_FREQUENCY";
static constexpr char DRIVE_AMPLITUDE_STRING[] = "DRIVE_AMPLITUDE";

class epicsShareClass NPDriveMotorAxis : public asynMotorAxis {
  public:
    NPDriveMotorAxis(class NPDriveMotorController *pC, int axisNo);
    void report(FILE *fp, int level);
    asynStatus stop(double acceleration);
    asynStatus move(double position, int relative, double min_velocity, double max_velocity,
                    double acceleration);
    asynStatus poll(bool *moving);
    asynStatus setClosedLoop(bool closedLoop);
    asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);

  private:
    NPDriveMotorController *pC_;
    int axisIndex_;

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

  protected:
    int driveFrequencyIndex_;
    int driveAmplitudeIndex_;

    friend class NPDriveMotorAxis;
};
