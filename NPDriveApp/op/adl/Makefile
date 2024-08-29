all:
	export CAQTDM_DISPLAY_PATH=$(CAQTDM_DISPLAY_PATH):/APSshare/uisys/sr/id
	/APSshare/bin/adl2ui NPDrive_hold_pos.adl
	/APSshare/bin/adl2ui NPDrive_open_loop.adl
	mv NPDrive_hold_pos.ui ../ui/autoconvert/
	mv NPDrive_open_loop.ui ../ui/autoconvert/
clean:
	rm ../ui/autoconvert/NPDrive_hold_pos.ui
	rm ../ui/autoconvert/NPDrive_open_loop.ui
