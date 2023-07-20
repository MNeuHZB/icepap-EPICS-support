
#include "asynMotorController.h"
#include "asynMotorAxis.h"

using namespace std;

#define NUM_ICEPAP_PARAMS 10

class icepapController : public asynMotorController
{
public:
	int axisSelect_;
	int blink_;
	int reset_;
	int allreset_;
	int powerOn_;
	int powerOff_;
	int wtemp_;
	int status_;
	int asciiCMD_;
	int asciiRESPONS_;

	char outString_[256];
	int axisSelection;
	
	icepapController(const char *portName, const char *phytronPortName, double movingPollPeriod, double idlePollPeriod, double timeout);
	asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
 	asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
 	asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
 	asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
 	asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual);
 	//asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason);

 	//asynStatus sendPapCommand(string arg, string *result = nullptr);
 	asynStatus sendPapCommand(const char *arg, string *result = nullptr);
 	
private:
	double timeout_;


friend class icepapAxis;
};

class icepapAxis : public asynMotorAxis
{
public:
	icepapAxis(class icepapController *pC, int axis);

	asynStatus setVelocity(int minVelocity, int maxVelocity);
	asynStatus setAcceleration(double acceleration, double maxVelocity);
	asynStatus setEnableBoard(int powerState);

	asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
	asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
	asynStatus stop(double acceleration);
	asynStatus poll(bool *moving);

private:
	icepapController *pC_;

	int poweronPull_;
	int motorDisable_;

friend class icepapController;
};

int cutCommand(string *cmd);
