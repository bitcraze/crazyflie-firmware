firmdir = ..
modinc = $(firmdir)/src/modules/interface
modsrc = $(firmdir)/src/modules/src

ifeq ($(CF_PYTHON),)
	CF_PYTHON = python3
endif

swig: setup.py cffirmware_wrap.c $(modsrc)/*.c
	$(CF_PYTHON) setup.py build_ext --inplace

cffirmware_wrap.c: cffirmware.i $(modinc)/*.h
	swig -python -I$(modinc) cffirmware.i

clean:
	rm -f cffirmware.py _cffirmware.so *.pyc cffirmware_wrap.c
	rm -rf build
