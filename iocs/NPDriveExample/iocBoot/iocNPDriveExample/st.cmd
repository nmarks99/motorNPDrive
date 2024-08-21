# ../../bin/${EPICS_HOST_ARCH}/NPDriveExample st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocNPDriveExampleLinux.dbd")
iocNPDriveExampleLinux_registerRecordDeviceDriver(pdbbase)

< settings.iocsh

# Create the controller object
epicsEnvSet("PORT_NAME", "NPDrive_ETH") 
drvAsynIPPortConfigure("$(PORT_NAME)", "164.54.115.52:6002")

dbLoadRecords("$(ASYN)/db/asynRecord.db", "P=$(PREFIX), R=asyn_1, PORT=$(PORT_NAME), ADDR=0, OMAX=$(OUT_BUFF=1000), IMAX=$(IN_BUFF=1000)")

NPDriveMotorCreateController("NPDrive1", "$(PORT_NAME)", 2, 250, 250)

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
