#!../../bin/linux-x86_64/icepap

#- You may have to change icepap to something else
#- everywhere it appears in this file

< envPaths

cd "${TOP}"

## Register all support components
dbLoadDatabase "dbd/icepap.dbd"
icepap_registerRecordDeviceDriver pdbbase

## Load record instances
#dbLoadRecords("db/icepap.db","user=user")

drvAsynIPPortConfigure("testRemote","172.24.65.242:5000",0,0,0)

#asynSetTraceFile("testRemote", 0, "debug.txt")
#asynSetTraceMask("testRemote", 0, 0x2F)
#asynSetTraceIOMask("testRemote", 0, 2)
#asynSetTraceInfoMask("testRemote", 0, 15)

asynOctetSetOutputEos("testRemote",0,"\r")
asynOctetSetInputEos("testRemote",0,"\n")

icepapCreateController("motionPort", "testRemote", 100, 100, 10)

icepapCreateAxis("motionPort",1)
icepapCreateAxis("motionPort",2)
# icepapCreateAxis("motionPort",3)

dbLoadTemplate("$(TOP)/iocBoot/$(IOC)/motor.substitutions.icepap")

cd "${TOP}/iocBoot/${IOC}"

dbLoadRecords("things.db","PORT=motionPort,ADDR=0")

iocInit

## Start any sequence programs
#seq sncxxx,"user=user"
