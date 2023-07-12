TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = icepap
# icepap.dbd will be created and installed
DBD += icepap.dbd


# icepap.dbd will be made up from these files:
icepap_DBD += base.dbd

icepap_DBD += motorSupport.dbd
icepap_DBD += asyn.dbd
icepap_DBD += drvAsynIPPort.dbd
icepap_DBD += icepapSup.dbd

# Add all the support libraries needed by this IOC
#icepap_LIBS += xxx
icepap_LIBS += motor
icepap_LIBS += asyn
icepap_LIBS += icepap
#icepap_LIBS += seq

# icepap_registerRecordDeviceDriver.cpp derives from icepap.dbd
icepap_SRCS += icepap_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
icepap_SRCS_DEFAULT += icepapMain.cpp
icepap_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#icepap_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
icepap_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

