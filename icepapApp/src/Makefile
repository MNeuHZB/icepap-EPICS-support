TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# build a support library

LIBRARY_IOC += icepap

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxxRecord
# install icepap.dbd into <top>/dbd
DBD += icepapSup.dbd
icepap_DBD += base.dbd
icepap_DBD += asyn.dbd
icepap_DBD += calcSupport.dbd
icepap_DBD += busySupport.dbd
icepap_DBD += motorSupport.dbd

INC += icepap_controller.h

LIB_LIBS += motor
LIB_LIBS += asyn
LIB_LIBS += $(EPICS_BASE_IOC_LIBS)

# specify all source files to be compiled and added to the library
icepap_SRCS += icepap_controller.cpp

icepap_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

