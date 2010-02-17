#stuff we need to compile
TRACKER_BIN = tracker
TESTER_BIN = tester
BLOB_BIN = blobtrack
DESCRIBE_BIN = describe

LIBOT_STATIC = libot.a
LIBOT_SHARED = libot.so

OBJ = otDataStream.o otModule.o otPipeline.o otParser.o otFactory.o otProperty.o \
      otDaemon.o otLog.o otCameraModule.o otImageDisplayModule.o otSmoothModule.o \
	  otImageFilterModule.o otInvertModule.o otVideoModule.o otBackgroundSubtractModule.o \
	  otGrayScaleModule.o otThresholdModule.o otAmplifyModule.o

#where the source is, and where to put the object files
SRC_DIR = src
MOD_DIR = ${SRC_DIR}/modules
BIN_DIR = bin

# compiler flags
CFLAGS ?= -O0 -g -Wall # -fPIC << needed for shared
LIBS   ?=

OPENCV_CFLAGS ?= `pkg-config --cflags opencv`
OPENCV_LIBS   ?= `pkg-config --libs opencv`

ALL_CFLAGS = ${CFLAGS} ${OPENCV_CFLAGS}
ALL_LIBS   = ${LIBS} ${OPENCV_LIBS}

BIN = $(addprefix ${BIN_DIR}/, ${OBJ})

#rules for building targets
all: static
	${CXX} ${ALL_LIBS} ${ALL_CFLAGS} -o ${TRACKER_BIN} src/tracker.cpp ${LIBOT_STATIC}
	${CXX} ${ALL_LIBS} ${ALL_CFLAGS} -o ${TESTER_BIN} src/tester.cpp ${LIBOT_STATIC}
	${CXX} ${ALL_LIBS} ${ALL_CFLAGS} -o ${BLOB_BIN} src/blobtracker.cpp
	${CXX} ${ALL_LIBS} ${ALL_CFLAGS} -o ${DESCRIBE_BIN} src/describe.cpp ${LIBOT_STATIC}

static: ${BIN}
	${AR} rcs ${LIBOT_STATIC} ${BIN}

#shared: ${BIN}
#	${CXX} -shared -Wl,-soname,${LIBOT_SHARED} -o ${LIBOT_SHARED} ${BIN}

#how to build stuff in SRC_DIR
${BIN_DIR}/%.o : ${SRC_DIR}/%.cpp
	${CXX} ${ALL_CFLAGS} -c $< -o $@

#how to build stuff in MOD_DIR
${BIN_DIR}/%.o : ${MOD_DIR}/%.cpp
	${CXX} ${ALL_CFLAGS} -c $< -o $@

clean:
	rm ${LIBOT_STATIC} ${LIBOT_SHARED} 2>/dev/null &>/dev/null
	rm ${TESTER_BIN} ${TRACKER_BIN} ${DESCRIBE_BIN} ${BLOB_BIN} 2>/dev/null &>/dev/null
	rm -r ${BIN_DIR}/*.o *.dSYM build 2>/dev/null &>/dev/null
