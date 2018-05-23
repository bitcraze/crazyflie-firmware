-Wall -Wmissing-braces -fno-strict-aliasing -std=gnu11
# Fail on warnings
CFLAGS += -Os -g3 -Werror
# Prevent promoting floats to doubles
CFLAGS += -Wdouble-promotion

OUTDIR = ../../generated-test/module-test/tdoa3

$(OUTDIR)/tester.so : $(OUTDIR)/lpsTdoaTagEngine.o $(OUTDIR)/outlierFilter.o $(OUTDIR)/mocks.o
	cc -shared -o $(OUTDIR)/tester.so $(OUTDIR)/lpsTdoaTagEngine.o $(OUTDIR)/outlierFilter.o $(OUTDIR)/mocks.o

$(OUTDIR)/lpsTdoaTagEngine.o : ../../src/deck/drivers/src/lpsTdoaTagEngine.c
	cc -c -fPIC $(CFLAGS) -I ../../src/modules/interface -I ../../src/deck/drivers/interface -I ../../src/hal/interface -I ../../vendor/libdw1000/inc -I mocks -o $(OUTDIR)/lpsTdoaTagEngine.o ../../src/deck/drivers/src/lpsTdoaTagEngine.c

$(OUTDIR)/outlierFilter.o : ../../src/deck/drivers/src/outlierFilter.c
	cc -c -fPIC $(CFLAGS) -I ../../src/modules/interface -I ../../src/deck/drivers/interface -I ../../src/hal/interface -I ../../vendor/libdw1000/inc -I mocks -o $(OUTDIR)/outlierFilter.o ../../src/deck/drivers/src/outlierFilter.c

$(OUTDIR)/mocks.o : mocks/mocks.c
	cc -c -fPIC $(CFLAGS) -I ../../src/modules/interface -I ../../src/deck/drivers/interface -I ../../src/hal/interface -I ../../vendor/libdw1000/inc -I mocks -o $(OUTDIR)/mocks.o mocks/mocks.c

clean :
	rm $(OUTDIR)/*