# ../../bin/${EPICS_HOST_ARCH}/NPDriveExample st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocNPDriveExampleLinux.dbd")
iocNPDriveExampleLinux_registerRecordDeviceDriver(pdbbase)

< settings.iocsh

# Create the controller object
epicsEnvSet("PORT_NAME", "NPDrive_ETH") 
drvAsynIPPortConfigure("$(PORT_NAME)", "127.0.0.1:23")
NPDriveMotorCreateController("NPDrive1", "$(PORT_NAME)", 2, 250, 1000)

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
