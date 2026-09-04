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
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=
EXTRA_LDFLAGS=

# NOTE: there used to be an include/stubs/ directory here, added to the device
# build with -isystem to paper over an ARM toolchain that was missing its
# standard library headers. It has been deleted, because it broke the build
# rather than fixing it.
#
# The problem: -isystem put those hand-written headers AHEAD of the real ones,
# so <cstdint> resolved to a stub that declared uint8_t in the GLOBAL
# namespace only. Any PROS header using std::uint8_t then failed - which is
# most of them, starting with pros/adi.hpp. Worse, the real <tuple> and
# <typeinfo> also broke, because they need std::intmax_t and correct
# fixed-width types to work at all.
#
# The actual fix for a toolchain missing headers is to install a complete
# toolchain (the PROS CLI ships one), not to hand-write fake standard headers.
# If you hit "stdint.h: No such file or directory", see the README.

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
# `make HOST_BUILD=1` builds and runs every host test binary below instead.
#
# To add a new host test: drop tests/test_<name>.cpp somewhere, add a
# CHASSIS_TEST_BIN-style variable + build rule for it below (copy an
# existing pair and rename), and add its binary to HOST_TEST_BINS.
HOST_CXX ?= g++
HOST_CXXFLAGS := -std=gnu++20 -pthread -Wall -DHOST_BUILD -I$(INCDIR)

CHASSIS_TEST_SRC := src/chassis.cpp src/util.cpp tests/test_chassis.cpp
CHASSIS_TEST_BIN := $(BINDIR)/host_test_chassis

UTIL_TEST_SRC := src/util.cpp tests/test_util.cpp
UTIL_TEST_BIN := $(BINDIR)/host_test_util

PID_TEST_SRC := tests/test_pid.cpp
PID_TEST_BIN := $(BINDIR)/host_test_pid

HOST_TEST_BINS := $(CHASSIS_TEST_BIN) $(UTIL_TEST_BIN) $(PID_TEST_BIN)

.DEFAULT_GOAL=host-test

.PHONY: host-test clean
host-test: $(HOST_TEST_BINS)
	@for bin in $(HOST_TEST_BINS); do $$bin || exit 1; done

$(CHASSIS_TEST_BIN): $(CHASSIS_TEST_SRC)
	@mkdir -p $(dir $(CHASSIS_TEST_BIN))
	$(HOST_CXX) $(HOST_CXXFLAGS) -o $@ $(CHASSIS_TEST_SRC)

$(UTIL_TEST_BIN): $(UTIL_TEST_SRC)
	@mkdir -p $(dir $(UTIL_TEST_BIN))
	$(HOST_CXX) $(HOST_CXXFLAGS) -o $@ $(UTIL_TEST_SRC)

$(PID_TEST_BIN): $(PID_TEST_SRC)
	@mkdir -p $(dir $(PID_TEST_BIN))
	$(HOST_CXX) $(HOST_CXXFLAGS) -o $@ $(PID_TEST_SRC)

clean:
	rm -f $(HOST_TEST_BINS)
else
-include ./common.mk
endif
