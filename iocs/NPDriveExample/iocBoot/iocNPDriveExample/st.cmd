# ../../bin/${EPICS_HOST_ARCH}/NPDriveExample st.cmd
< envPaths

dbLoadDatabase("../../dbd/iocNPDriveExampleLinux.dbd")
iocNPDriveExampleLinux_registerRecordDeviceDriver(pdbbase)

< settings.iocsh

< NPDrive.iocsh

###############################################################################
iocInit
###############################################################################

# print the time our boot was finished
date
