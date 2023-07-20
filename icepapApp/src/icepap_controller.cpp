
#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <string.h>
#include <stdlib.h>
#include <vector>
#include <math.h>
#ifndef _WIN32
#include <unistd.h>
#endif

#include <drvAsynIPPort.h>
#include <iocsh.h>
#include <epicsThread.h>
#include <cantProceed.h>

#include <asynOctetSyncIO.h>

#include "icepap_controller.h"
#include <epicsExport.h>

static vector<icepapController*> controllers;

/** Creates a new icepapController object.
  * \param[in] icepapPortName    The name of the asyn port that will be created for this driver
  * \param[in] phytronPortName   The name of the drvAsynIPPort that was created previously to connect to the phytron controller
  * \param[in] movingPollPeriod  The time between polls when any axis is moving
  * \param[in] idlePollPeriod    The time between polls when no axis is moving
  */
icepapController::icepapController(const char *icepapPortName, const char *asynPortName,
                                 double movingPollPeriod, double idlePollPeriod, double timeout)
  :  asynMotorController(icepapPortName,
                         0xFF, // max amount of axis
                         NUM_ICEPAP_PARAMS,
                         0, //No additional interfaces beyond those in base class
                         0, //No additional callback interfaces beyond those in base class
                         ASYN_CANBLOCK | ASYN_MULTIDEVICE,
                         1, // autoconnect
                         0, 0)// Default priority and stack size
{
	asynStatus status;
	
	timeout_ = timeout;
  axisSelection = 1;
	
  createParam("AxisSelect", asynParamInt32, &this->axisSelect_);
	createParam("BLINK", asynParamFloat64, &this->blink_);
  createParam("RESET", asynParamInt32, &this->reset_);
  createParam("ALLRESET", asynParamInt32, &this->allreset_);
  createParam("POWERON", asynParamInt32, &this->powerOn_);
  createParam("POWEROFF", asynParamInt32, &this->powerOff_);
  createParam("WTEMP", asynParamFloat64, &this->wtemp_);
  createParam("getStatus", asynParamInt32 ,&this->status_);
  createParam("setASCIIcommand", asynParamOctet, &this->asciiCMD_);
  createParam("getASCIIrespons", asynParamOctet, &this->asciiRESPONS_);
	 
	status = pasynOctetSyncIO->connect(asynPortName, 0, &pasynUserController_, NULL);
	
	if (status){
  	asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
    	"icepap cannot be connected\n");
  } else {
  	//icepapCreateAxis will search for the controller for axis registration
  	controllers.push_back(this);
	
  	//epicsThreadSleep(10.0);
	
    startPoller(movingPollPeriod, idlePollPeriod, 5);
  }
}

/** Creates a new icepapController object.
  * Configuration command, called directly or from iocsh
  * \param[in] icepapPortName   The name of the drvAsynIPPPort that was created previously to connect to the phytron controller
  * \param[in] asynPortName      The name of the asyn port that will be created for this driver
  * \param[in] movingPollPeriod  The time in ms between polls when any axis is moving
  * \param[in] idlePollPeriod    The time in ms between polls when no axis is moving
  */
extern "C" int icepapCreateController(const char *icepapPortName, const char *asynPortName,
                                   int movingPollPeriod, int idlePollPeriod, double timeout)
{
  new icepapController(icepapPortName, asynPortName, movingPollPeriod/1000., idlePollPeriod/1000., timeout);
  return asynSuccess;
}

/** asynUsers use this to read integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[out] value      Parameter value
 */
asynStatus icepapController::readInt32(asynUser *pasynUser, epicsInt32 *value){
  asynStatus status;

  //Call base implementation first
  status = asynPortDriver::readInt32(pasynUser, value);

  if(pasynUser->reason == status_){
    string reply;

    sprintf(outString_, "#%d:?status", axisSelection);
    status = sendPapCommand(outString_, &reply);
    try{
        int pos = reply.find(" ");
        *value = stod(reply.substr(pos));
    }
    catch(...){}
  }

  return status;
}

/** asynUsers use this to write integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[in] value       Parameter value to be written
 */
asynStatus icepapController::writeInt32(asynUser *pasynUser, epicsInt32 value){
  asynStatus result;
  string reply;

  //Call base implementation first
  asynMotorController::writeInt32(pasynUser, value);

  //int axis
  //getAddress(pasynUser, &axis);

  if(pasynUser->reason == axisSelect_){
    axisSelection = int(value);
    return asynSuccess;
  }
  else if(pasynUser->reason == allreset_){
    printf(" reset for all axis \n");
    result = sendPapCommand("reset");
    return result;
  }
  else if(pasynUser->reason == reset_){
    sprintf(outString_, "#%d:reset", value);
    result = sendPapCommand(outString_, &reply);
    return result;
  }
  else if(pasynUser->reason == powerOn_){
    sprintf(outString_, "#%d:power on", value);
    result = sendPapCommand(outString_, &reply);
  }
  else if(pasynUser->reason == powerOff_){
    sprintf(outString_, "#%d:power off", value);
    result = sendPapCommand(outString_, &reply);
  }

  return asynSuccess;
}

/** asynUsers use this to read float parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[out] value      Parameter value
 */
asynStatus icepapController::readFloat64(asynUser *pasynUser, epicsFloat64 *value){

	asynStatus status;
  string reply;

  //Call base implementation first
  status = asynPortDriver::readFloat64(pasynUser, value);
	
  if(pasynUser->reason == blink_){
    sprintf(outString_, "#%d:?blink", axisSelection);
  	status = sendPapCommand(outString_, &reply);
    	
  	try{
  		*value = stod(reply.substr(8));
  	}
  	catch(...){
  		//printf("%s\n", reply.c_str());
  	}
  }
  else if(pasynUser->reason == wtemp_){
    sprintf(outString_, "#%d:?wtemp", axisSelection);
    status = sendPapCommand(outString_, &reply);
    //printf("%s \n", reply.c_str());
      
    try{
      cutCommand(&reply);

      if(reply.size() > 4){ // string is invalid if loger than 4 charaters
        *value = -1;
      }
      else{
        *value = stod(reply);
      }

      //printf("(%d) wtemp reply: %s\n" , __LINE__, reply.c_str());
    }
    catch(...){
      //printf("%s\n", reply.c_str());
    }
  } 
	
	return status;
}


/** asynUsers use this to write integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[in] value       Parameter value to be written
 */
asynStatus icepapController::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{

  //Call base implementation first
  asynMotorController::writeFloat64(pasynUser, value);
  
  asynStatus status;
  string reply;

  if(pasynUser->reason == blink_){
    sprintf(outString_, "#%d:blink %f", axisSelection, value);
    status = sendPapCommand(outString_ ,&reply);

    if(status){
      printf("blink command error %d\n", status);
      return status;
    }
  }

  return asynSuccess;
}

/** brief implements Icepap data string send / recieve
 * \param[in]  arg     command string to send
 * \param[out] result  command reply string
 * \return
 */
asynStatus icepapController::sendPapCommand(const char *arg, string *result){
  asynStatus status = asynSuccess;

  if(result){
    size_t nread = 0;
    result->resize(1000);
    
    status = writeReadController(arg ,&result->operator[](0) ,result->size() ,&nread ,timeout_);
    
    //printf("%s - %lu - %d - %s\n", arg, nread, status, result->c_str());
    
    if(status == asynSuccess){
      result->resize(nread);
    }
  }
  else{
    status = writeController(arg ,timeout_);
  }
  
  return status;
}

/** Called when asyn clients call pasynOctet->write().
  * This function performs actions for some parameters, including AttributesFile.
  * For all parameters it sets the value in the parameter library and calls any registered callbacks..
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the string to write.
  * \param[in] nChars Number of characters to write.
  * \param[out] nActual Number of characters actually written. */
asynStatus icepapController::writeOctet(asynUser *pasynUser, const char *value, size_t nChars, size_t *nActual) {
  int addr = 0;
  int function = pasynUser->reason;
  asynStatus status = asynSuccess;
  string reply;

  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess) return (status);
  // Set the parameter in the parameter library.
  status = (asynStatus) setStringParam(addr, function, (char *) value);
  if (status != asynSuccess) return (status);

  if (function == asciiCMD_){
    status = sendPapCommand(value, &reply);
    printf("cmd: %s rsp: %s\n", value, reply.c_str());
    status = (asynStatus) setStringParam(addr, asciiRESPONS_, reply.c_str());
  }

  // Do callbacks so higher layers see any changes
  callParamCallbacks(addr, addr);

  //Call base class method. This will handle callCallbacks even if the function was handled here.
  status = asynMotorController::writeOctet(pasynUser, value, nChars, nActual);

  *nActual = nChars;

  return status;
}

/** Called when asyn clients call pasynOctet->read().
  * The base class implementation simply returns the value from the parameter library.
  * Derived classes rarely need to reimplement this function.
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the string to read.
  * \param[in] maxChars Maximum number of characters to read.
  * \param[out] nActual Number of characters actually read. Base class sets this to strlen(value).
  * \param[out] eomReason Reason that read terminated. Base class sets this to ASYN_EOM_END. */
/*asynStatus icepapController::readOctet(asynUser *pasynUser, char *value, size_t maxChars, size_t *nActual, int *eomReason)
{
    int function;
    const char *paramName;
    int addr;
    asynStatus status = asynSuccess;
    //epicsTimeStamp timeStamp; getTimeStamp(&timeStamp);
    //static const char *functionName = "readOctet";

    status = parseAsynUser(pasynUser, &function, &addr, &paramName);
    if (status != asynSuccess) return status;
    status = asynMotorController::readOctet(pasynUser, value, maxChars, nActual, eomReason);

    printf("readOctet");*/


    //printf("  readOctet sVal[%d]=\"%.*s\" addr=%d %d/%s %d\n", static_cast<int>(*nActual), value, addr, function, paramName, status);


    /* We just read the current value of the parameter from the parameter library.
     * Those values are updated whenever anything could cause them to change */
    //status = (asynStatus)getStringParam(addr, function, (int)maxChars, value);
    /* Set the timestamp */
    /*pasynUser->timestamp = timeStamp;
    getParamAlarmStatus(addr, function, &pasynUser->alarmStatus);
    getParamAlarmSeverity(addr, function, &pasynUser->alarmSeverity);
    if (status)
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                  "%s:%s: status=%d, function=%d, name=%s, value=%s",
                  driverName, functionName, status, function, paramName, value);
    else
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s:%s: function=%d, name=%s, value=%s\n",
              driverName, functionName, function, paramName, value);
    if (eomReason) *eomReason = ASYN_EOM_END;
    *nActual = strlen(value)+1;*/
    /*return status;
}*/

/** Creates a new icepapAxis object.
  * \param[in] pC Pointer to the phytronController to which this axis belongs.
  * \param[in] axisNo Index number of this axis, range 0 to pC->numAxes_-1.
  *
  * Initializes register numbers, etc.
  */
icepapAxis::icepapAxis(class icepapController *pC, int axis) : asynMotorAxis(pC, axis), pC_(pC) /*, axisModuleNo_((float)axisNo/10), response_len(0)*/
{
  setIntegerParam(pC->motorStatusHasEncoder_, 1);
  setDoubleParam(pC->motorEncoderRatio_, 1.);

  string reply;
  asynStatus status;
  sprintf(pC_->outString_, "#%i:?status", axisNo_); // prepare and send command for checking state
  status = pC_->sendPapCommand(pC_->outString_, &reply);

  if(status){
     printf("Axis %d error.\n", axisNo_);
     //exit(0);
  }
  else{
    cutCommand(&reply); //remove command from answer

    unsigned int icePapStatus = stoul(reply, nullptr, 16); // convert hexnumber from string to int;
    icePapStatus = icePapStatus & 0x00000003;

    if(icePapStatus == 3){
      printf("Axis %d alive.\n", axisNo_);
    }
    else{
      printf("Axis %d not alive. Status: %d\n", axisNo_, icePapStatus);
    }
  }
}

/** Post-processing of a recieved command.
  * Removes the command header from message --> 1:?STATUS 0x00202063 --> 0x00202063
  * \param[in] cmd recieved command string
  */
int cutCommand(string *cmd){
  int pos = cmd->find(" ");
  *cmd = cmd->substr(pos);

  return 0;
}

/** Sets velocity parameters before the move is executed. Controller produces a
 * trapezoidal speed profile defined by these parmeters.
 * \param[in] minVelocity   Start velocity
 * \param[in] maxVelocity   Maximum velocity
 */
asynStatus icepapAxis::setVelocity(int minVelocity, int maxVelocity){
  asynStatus status;
  string reply;

  sprintf(pC_->outString_, "#%i:velocity %i", axisNo_, maxVelocity);

  status = pC_->sendPapCommand(pC_->outString_, &reply);

  if(status){
    printf("Could not set velocity!\n");
  }

   printf("(%d) %s\n", __LINE__, reply.c_str());

  return status;
}

/** Sets acceleration parameters before the move is executed.
 * \param[in] acceleration  Acceleration to be used in the move
 * \param[in] moveType      Type of movement determines which controller acceleration parameters is set
 */
asynStatus icepapAxis::setAcceleration(double acceleration, double maxVelocity){
  asynStatus status;
  string reply;

  double accelerationTime = maxVelocity / acceleration;

  sprintf(pC_->outString_, "#%i:acctime %f", axisNo_, accelerationTime);

  status = pC_->sendPapCommand(pC_->outString_, &reply);

  if(status){
    printf("Could not set acceleration time!\n");
  }

  printf("(%d) %s\n", __LINE__, reply.c_str());

  return status;
}

/** Tries to enable a disabled dirver board, if not already enabled
 * \param[in] powerState  state of driver board power
 */
asynStatus icepapAxis::setEnableBoard(int powerState){
  asynStatus status = asynSuccess;

  printf("(%d) %d poweron state: %d \n", __LINE__, axisNo_, powerState);

  if(!powerState){ // try to enable axis
    string reply;
    sprintf(pC_->outString_, "#%i:power on", axisNo_);
    status = pC_->sendPapCommand(pC_->outString_, &reply);

    cutCommand(&reply);
    printf("(%d) enable board%i: %s\n", __LINE__, axisNo_, reply.c_str());

    if(strcmp(reply.c_str(), " OK") != 0){
        return asynError;
    }
  }

  return status;
}

/** Execute the move.
 * \param[in] position      Target position (relative or absolute).
 * \param[in] relative      Is the move absolute or relative
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 */
asynStatus icepapAxis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration){
  asynStatus status;
  string reply;

  status = setEnableBoard(poweronPull_);
  if(status){
    printf(" Enable driver %d failed.", axisNo_);
    return status;
  }

  status = setVelocity(minVelocity, maxVelocity);

  if(status){
    printf("Could not set velocity!\n");
    return status;
  }

  status = setAcceleration(acceleration, maxVelocity);

  if(status){
    printf("Could not set acceleration!\n");
    return status;
  }

  if(relative){
    sprintf(pC_->outString_, "#%i:rmove %lld", axisNo_, static_cast<long long>(position));
  }
  else{
    sprintf(pC_->outString_, "#%i:move %lld", axisNo_, static_cast<long long>(position));
  }
  printf("(%d) %s\n", __LINE__, pC_->outString_);

  status = pC_->sendPapCommand(pC_->outString_, &reply);

  printf("(%d) %s\n" , __LINE__, reply.c_str());

  if(status){
    printf("Could not send move command!\n");
    return status;
  }

  return asynSuccess;
}

/** Jog the motor. Direction is determined by sign of the maxVelocity profile
 * \param[in] minVelocity   Lowest velocity of the trapezoidal speed profile.
 * \param[in] maxVelocity   Highest velocity of the trapezoidal speed profile
 * \param[in] acceleration  Acceleration to be used
 */
asynStatus icepapAxis::moveVelocity(double minVelocity, double maxVelocity, double acceleration){
  asynStatus status;
  string reply;

  status = setEnableBoard(poweronPull_);
  if(status){
    printf(" Enable driver %d failed.", axisNo_);
    return status;
  }

  status = setAcceleration(acceleration, maxVelocity);

  if(status){
    printf("Could not set acceleration!\n");
    return status;
  }

  sprintf(pC_->outString_, "#%i:jog %.0f", axisNo_, maxVelocity);

  status = pC_->sendPapCommand(pC_->outString_, &reply);

  if(status){
    printf("Could not send jog command!\n");
    return status;
  }

  return asynSuccess;
}

/** Stop the motor
 * \param[in] acceleration  Deceleration to be used
 */
asynStatus icepapAxis::stop(double acceleration){
  asynStatus status;
  string reply;

  /*status = setAcceleration(acceleration, -1);

  if(status){
    printf("Could not set acceleration!\n");
    return status;
  }*/

  sprintf(pC_->outString_, "#%i:stop", axisNo_);

  status = pC_->sendPapCommand(pC_->outString_, &reply);

  printf("(%d) stop axis %d %s \n", __LINE__, axisNo_, reply.c_str());

  if(status){
    printf("Could not stop axis!\n");
    return status;
  }

  return asynSuccess;
}

/** Polls the axis.
  * This function reads the motor position, the limit status, the moving status,
  * and the drive power-on status.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * and then calls callParamCallbacks() at the end.
  * \param[out] moving A flag that is set indicating that the axis is moving (true) or done (false).
  */
asynStatus icepapAxis::poll(bool *moving){
  asynStatus status;
  string reply;

  sprintf(pC_->outString_, "#%i:?status", axisNo_);
  status = pC_->sendPapCommand(pC_->outString_, &reply);

  if(status){
    printf("(%d) Axis %d status not recieved! -- %d\n", __LINE__, axisNo_, status);
    return status;
  }

  cutCommand(&reply);

  //invalid answer if string is to long
  if (reply.size() > 11){
    cout << __LINE__ << " Status " << reply.size() << " " << reply << endl;
    return asynError;
  }

  unsigned int icePapStatus = 0;
  try
  {
    icePapStatus = stoul(reply, nullptr, 16); // convert hexnumber from string to int;
  }
  catch (...){
    printf("status \n");
    printf("(%d) %s\n", __LINE__, reply.c_str());
    return asynError;
  }

  // read encoder position
  sprintf(pC_->outString_, "#%i:?enc", axisNo_);
  status = pC_->sendPapCommand(pC_->outString_, &reply);

  if(status){
    printf("Axis %d encoder value not recieved! -- %d\n", axisNo_, status);
    return status;
  }

  cutCommand(&reply);
  unsigned int positionEncoder = 0;

  try{
    positionEncoder = stoul(reply, nullptr, 10); // convert decimal-number from string to int
  }
  catch (...){
    printf("(%d) encoder %s\n", __LINE__, reply.c_str());
  }

  // read axis position
  sprintf(pC_->outString_, "#%i:?pos", axisNo_);
  status = pC_->sendPapCommand(pC_->outString_, &reply);

  if(status){
    printf("Axis %d position value not recieved! -- %d\n", axisNo_, status);
    return status;
  }

  cutCommand(&reply);
  unsigned int positionAxis = 0;

  try{
    positionAxis = stoul(reply, nullptr, 10); // convert decimal-number from string to int
  }
  catch (...){
    printf("(%d) encoder %s\n", __LINE__, reply.c_str());
  }

  // write parameter
  setDoubleParam(pC_->motorPosition_, positionAxis);
  setDoubleParam(pC_->motorEncoderPosition_, positionEncoder);

  setIntegerParam(pC_->motorStatusHighLimit_, (icePapStatus >> 18) & 0x00000001);
  setIntegerParam(pC_->motorStatusLowLimit_, (icePapStatus >> 19) & 0x00000001);
  setIntegerParam(pC_->motorStatusMoving_, (icePapStatus >> 10) & 0x00000001);
  setIntegerParam(pC_->motorStatusDone_, !((icePapStatus >> 10) & 0x00000001));
  //setIntegerParam(pC_->motorStatusProblem_, (icePapStatus >> 13) & 0x00000001); //universell
  poweronPull_ = (icePapStatus >> 23) & 0x00000001; // power on
  motorDisable_ = (icePapStatus >> 4) & 0x00000007; // disable / power enabled = 0

  //printf("(%d) %d poweron state: %d \n", __LINE__, axisNo_, pC_->poweronPull_);

  setIntegerParam(pC_->motorStatusProblem_, 0);

  callParamCallbacks();
  
  return asynSuccess;
}

/** Creates a new icepapAxis object.
  * Configuration command, called directly or from iocsh
  * \param[in] controllerName    Name of the asyn port created by calling icepapCreateController from st.cmd
  * \param[in] axis              Axis index
  */
extern "C" int icepapCreateAxis(const char* controllerName, int axis){

  icepapAxis *pAxis;

  //Find the controller
  uint32_t i;
  for(i = 0; i < controllers.size(); i++){
    if(!strcmp(controllers[i]->portName, controllerName)) {

      //todo: check if axis exists // if not break

      pAxis = new icepapAxis(controllers[i], axis);
      //controllers[i]->axes.push_back(pAxis);
      break;
    }
  }

  //If controller is not found, report error
  if(i == controllers.size()){
    printf("ERROR: icepapCreateAxis: Controller %s is not registered\n", controllerName);
    return asynError;
  }

  return asynSuccess;
}


/** Parameters for iocsh icepap axis registration*/
static const iocshArg icepapCreateAxisArg0 = {"Controller Name", iocshArgString};
static const iocshArg icepapCreateAxisArg1 = {"Axis index", iocshArgInt};
static const iocshArg* const icepapCreateAxisArgs[] = {&icepapCreateAxisArg0,
                                                      &icepapCreateAxisArg1};
/** Parameters for iocsh icepap controller registration */
static const iocshArg icepapCreateControllerArg0 = {"Port name", iocshArgString};
static const iocshArg icepapCreateControllerArg1 = {"icepapAxis port name", iocshArgString};
static const iocshArg icepapCreateControllerArg2 = {"Moving poll period (ms)", iocshArgInt};
static const iocshArg icepapCreateControllerArg3 = {"Idle poll period (ms)", iocshArgInt};
static const iocshArg icepapCreateControllerArg4 = {"Timeout (ms)", iocshArgDouble};
static const iocshArg * const icepapCreateControllerArgs[] = {&icepapCreateControllerArg0,
                                                             &icepapCreateControllerArg1,
                                                             &icepapCreateControllerArg2,
                                                             &icepapCreateControllerArg3,
                                                             &icepapCreateControllerArg4};

static const iocshFuncDef icepapCreateAxisDef = {"icepapCreateAxis", 2, icepapCreateAxisArgs};
static const iocshFuncDef icepapCreateControllerDef = {"icepapCreateController", 5, icepapCreateControllerArgs};

static void icepapCreateControllerCallFunc(const iocshArgBuf *args)
{
  icepapCreateController(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].dval);
}

static void icepapCreateAxisCallFunc(const iocshArgBuf *args)
{
  icepapCreateAxis(args[0].sval, args[1].ival);
}

static void icepapRegistrar(void)
{
  iocshRegister(&icepapCreateControllerDef, icepapCreateControllerCallFunc);
  iocshRegister(&icepapCreateAxisDef, icepapCreateAxisCallFunc);
}

extern "C" {
epicsExportRegistrar(icepapRegistrar);
}
