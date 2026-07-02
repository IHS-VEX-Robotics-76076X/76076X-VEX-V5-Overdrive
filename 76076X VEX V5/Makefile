################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
ifndef HOST_BUILD
EXTRA_CFLAGS=-isystem./include/stubs
EXTRA_CXXFLAGS=-isystem./include/stubs
else
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=
endif
EXTRA_LDFLAGS=

# Add stubs directory for missing C++ standard library headers
EXTRA_INCDIR:=$(INCDIR)/stubs

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# If building for host (make HOST_BUILD=1), add preprocessor define
ifdef HOST_BUILD
EXTRA_CFLAGS+=-DHOST_BUILD
EXTRA_CXXFLAGS+=-DHOST_BUILD
endif

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########

ifdef HOST_BUILD
# Host builds compile against the mock API in include/host/ and run on the
# developer's machine, so they can't go through common.mk's device-build
# rules below (quick/all) - those pass ARM-only linker flags
# (--gc-keep-exported, -Tfirmware/v5.ld, etc.) to whatever linker HOST_CXX
# resolves to, which isn't arm-none-eabi-ld and doesn't understand them.
# `make HOST_BUILD=1` builds and runs the host test suite directly instead;
# add more .cpp files to HOST_TEST_SRC if you add more host tests.
HOST_CXX ?= g++
HOST_TEST_SRC := src/chassis.cpp src/util.cpp tests/test_chassis.cpp
HOST_TEST_BIN := $(BINDIR)/host_test

.DEFAULT_GOAL=host-test

.PHONY: host-test clean
host-test: $(HOST_TEST_BIN)
	$(HOST_TEST_BIN)

$(HOST_TEST_BIN): $(HOST_TEST_SRC)
	@mkdir -p $(dir $(HOST_TEST_BIN))
	$(HOST_CXX) -std=gnu++20 -pthread -Wall -DHOST_BUILD -I$(INCDIR) -o $@ $(HOST_TEST_SRC)

clean:
	rm -f $(HOST_TEST_BIN)
else
-include ./common.mk
endif
