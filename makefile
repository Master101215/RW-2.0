# VEXcode makefile 2019_03_26_01

# show compiler output
VERBOSE = 0

# include toolchain options
include vex/mkenv.mk

# location of the project source cpp and c files
SRC_C  = $(wildcard src/*.cpp) 
SRC_C += $(wildcard src/*.c)
SRC_C += $(wildcard src/*/*.cpp) 
SRC_C += $(wildcard src/*/*.c)

# mcl and pure-pursuit live in their own top-level folders
SRC_C += $(wildcard mcl/src/*.cpp)
SRC_C += $(wildcard mcl/src/*.c)
SRC_C += $(wildcard pure-pursuit/src/*.cpp)
SRC_C += $(wildcard pure-pursuit/src/*.c)

OBJ = $(addprefix $(BUILD)/, $(addsuffix .o, $(basename $(SRC_C))) )

# location of include files that c and cpp files depend on
SRC_H  = $(wildcard include/*.h)
SRC_H += $(wildcard mcl/include/*.h)
SRC_H += $(wildcard pure-pursuit/include/*.h)

# additional dependancies
SRC_A  = makefile

# project header file locations
INC_F  = include
INC_F += mcl/include
INC_F += pure-pursuit/include

# build targets
all: $(BUILD)/$(PROJECT).bin

# include build rules
include vex/mkrules.mk
