## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,er4ft linker.cmd package/cfg/sbl_per4ft.oer4ft

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/sbl_per4ft.xdl
	$(SED) 's"^\"\(package/cfg/sbl_per4ftcfg.cmd\)\"$""\"C:/ti/mmwave_sdk_03_05_00_04/packages/ti/utils/sbl/mss_configPkg_xwr68xx/\1\""' package/cfg/sbl_per4ft.xdl > $@
	-$(SETDATE) -r:max package/cfg/sbl_per4ft.h compiler.opt compiler.opt.defs
