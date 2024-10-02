# ../../bin/${EPICS_HOST_ARCH}/NPDriveExample st.cmd
< envPaths
epicsEnvSet("IOCSH_PS1", "$(IOC)>")
epicsEnvSet("PREFIX", "NPDriveExample:")

dbLoadDatabase("../../dbd/iocNPDriveExampleLinux.dbd")
iocNPDriveExampleLinux_registerRecordDeviceDriver(pdbbase)

< NPDrive.cmd

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
