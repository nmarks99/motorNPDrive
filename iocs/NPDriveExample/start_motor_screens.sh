#!/usr/bin/env bash

export CAQTDM_DISPLAY_PATH=$CAQTDM_DISPLAY_PATH:/APSshare/epics/synApps_6_3/support/motor-R7-3-1/motorApp/op/ui
export CAQTDM_DISPLAY_PATH=$CAQTDM_DISPLAY_PATH:/APSshare/epics/synApps_6_3/support/motor-R7-3-1/motorApp/op/ui/autoconvert
/APSshare/bin/caQtDM -macro P=NPDriveExample:,M1=m1,M2=m2,M3=m3,M4=m4,M5=m5,M6=m6,M7=m7,M8=m8 /APSshare/epics/synApps_6_3/support/motor-R7-3-1/motorApp/op/ui/autoconvert/topMotors8.ui
