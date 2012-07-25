DIRS = kernel libc canny netperf/src softfloat sysprogs tests
# the sets of directories to do various things in
BUILDDIRS = $(DIRS:%=build-%)
CLEANDIRS = $(DIRS:%=clean-%)

all: $(BUILDDIRS)
$(DIRS): $(BUILDDIRS)
$(BUILDDIRS):
	$(MAKE) -C $(@:build-%=%)

# set dependencies
build-canny: build-libc build-softfloat
build-sysprogs: build-libc build-softfloat
build-tests: build-libc build-softfloat
build-netperf/src: build-libc build-softfloat
build-libc: build-softfloat
build-kernel: build-libc

test: $(TESTDIRS) all
$(TESTDIRS): 
	$(MAKE) -C $(@:test-%=%) test

clean: $(CLEANDIRS)
$(CLEANDIRS): 
	$(MAKE) -C $(@:clean-%=%) clean


.PHONY: subdirs $(DIRS)
.PHONY: subdirs $(BUILDDIRS)
.PHONY: subdirs $(CLEANDIRS)
.PHONY: all clean
