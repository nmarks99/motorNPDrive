# Makefile
TOP=../..

include $(TOP)/configure/CONFIG

# Define IOC library name
LIBRARY_IOC += NPDrive

# install NPDriveSupport.dbd into <top>/dbd
DBD += NPDriveSupport.dbd
NPDriveSupport_DBD += NPDrive.dbd

# source files to be compiled and added to the library
NPDrive_SRCS += np_drive.cpp

# Libraries needed for NPDrive
NPDrive_LIBS += motor asyn
NPDrive_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
