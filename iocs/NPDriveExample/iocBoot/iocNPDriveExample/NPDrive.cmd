# Connect to controller
iocshLoad("$(MOTOR_NPDRIVE)/iocsh/NPDrive.iocsh", "INSTANCE=NPDrive1,IP_ADDR=164.54.115.52")

# Load motor records and PVs for axis specific settings
dbLoadTemplate("NPDrive.substitutions", "P=$(PREFIX)")

# extra PVs for controller level settings
dbLoadRecords("$(MOTOR_NPDRIVE)/db/NPDrive.db", "P=$(PREFIX),PORT=NPDrive1,ADDR=0")
