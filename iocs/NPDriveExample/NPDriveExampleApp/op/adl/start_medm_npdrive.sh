#!/usr/bin/env bash
export EPICS_DISPLAY_PATH=${EPICS_DISPLAY_PATH}:/APSshare/epics/synApps_6_3/support/motor-R7-3-1/motorApp/op/adl
medm -macro "P=NPDriveExample:, M=m1" -x NPDrive_hold_pos.adl


