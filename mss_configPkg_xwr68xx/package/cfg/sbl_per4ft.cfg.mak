# invoke SourceDir generated makefile for sbl.per4ft
sbl.per4ft: .libraries,sbl.per4ft
.libraries,sbl.per4ft: package/cfg/sbl_per4ft.xdl
	$(MAKE) -f package/cfg/sbl_per4ft.src/makefile.libs

clean::
	$(MAKE) -f package/cfg/sbl_per4ft.src/makefile.libs clean

