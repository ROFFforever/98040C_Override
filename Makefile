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

# common.mk defaults to gnu++26, which this project's arm-none-eabi-g++
# (14.3.1) doesn't recognize yet (that alias was added in GCC 15). Pin to
# gnu++23, the newest standard this toolchain actually supports.
CXX_STANDARD:=gnu++23

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:=

COLD_DIRS := commandScheduler util Commands/TeleopCommands Commands/Tuning Controllers Autons Telemetry
COLD_FILES := Subsystems/Lift.cpp Subsystems/Motors.cpp Subsystems/piston.cpp Subsystems/drivetrain.cpp Commands/LiftMoveToCommand.cpp Commands/Rotate.cpp Commands/Swing.cpp Commands/tank_motion_profile.cpp
EXCLUDE_SRCDIRS += $(foreach d,$(COLD_DIRS),$(SRCDIR)/$(d)) $(foreach f,$(COLD_FILES),$(SRCDIR)/$(f))
COLD_LIBAR := $(FWDIR)/Override98040C.a
COLD_CXXSRC := $(foreach d,$(COLD_DIRS),$(wildcard $(SRCDIR)/$(d)/*.cpp)) $(foreach f,$(COLD_FILES),$(SRCDIR)/$(f))
COLD_OBJS := $(addprefix $(BINDIR)/,$(patsubst $(SRCDIR)/%,%.o,$(COLD_CXXSRC)))

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
-include ./common.mk

$(COLD_LIBAR): $(COLD_OBJS)
	$Dmkdir -p $(FWDIR)
	-$Drm -f $@
	$(call test_output_2,Creating $@ ,$(AR) rcs $@ $^, $(DONE_STRING))

.PHONY: refresh-cold
refresh-cold:
	-$Drm -f $(COLD_LIBAR)
	+$(MAKE) $(COLD_LIBAR)
