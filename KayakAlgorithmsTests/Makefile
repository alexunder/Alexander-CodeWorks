# CXX ?= g++
# CXX ?= gcc
#CXX = aarch64-poky-linux-g++
#CC = aarch64-poky-linux-gcc

APP = Kayak-test
SRCS = main.cpp \
      RoboticsDynamics.cpp \
      Algorithm.cpp \
      KukaAlgorithms.cpp \
	  hls_helper.cpp \
      Quaternion.cpp \
      Utils/UtilFunc.cpp \
      Utils/GeometryUtils.cpp \
      Utils/Frame.cpp \
      Utils/Rotation.cpp

# Options for development
INCLUDE =  -I./eigen-3.3.7
OBJS =  $(SRCS:.cpp=.o)

#CFLAGS = -march=armv8-a -mtune=cortex-a57.cortex-a53  --sysroot=/home/alexander/works/renesas/linux/sdk/sysroots/
#CFLAGS_XENOMAI = -I../xenomai/include/cobalt -I../xenomai/include -D_GNU_SOURCE -D_REENTRANT -fasynchronous-unwind-tables -D__COBALT__ -I../xenomai/include/alchemy -std=c++11 -g -rdynamic
#CFLAGS = -g -O3 -Wall -Wl,--no-as-needed -Wl,@../xenomai/lib/modechk.wrappers -lalchemy -lcopperplate ../xenomai/lib/xenomai/bootstrap.o -Wl,--wrap=main -Wl,--dynamic-list=../xenomai/lib/dynlist.ld -L../xenomai/lib -lcobalt -lmodechk -lpthread -lrt -lm -lstdc++ -g -rdynamic
CFLAGS = -std=c++11 -g -Wall -rdynamic
LIBS = -lm

all: $(APP)

clean:
	-rm -f *~ *bak *.o Utils/*.o core.* Makefile.depend $(APP)

$(APP): $(OBJS)
	$(CXX) $(CFLAGS) -o $@ $(OBJS) $(LIBS)

#.C.o:
#	$(CXX) $< -c -o $@

.cpp.o:
	$(CXX) $(CFLAGS_XENOMAI) $(INCLUDE) $< -c -o $@
