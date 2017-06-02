CXX      := $(shell which g++)

CXXSTD   := -std=c++14
OPTS     := -g -Wall -m64 $(CXXSTD)

CXXFLAGS := $(OPTS) -c -pthread -fPIC -O3
LDFLAGS  := $(OPTS) -shared

PYCONFIG := $(shell which python3-config)

PY3CFLAGS    := $(shell $(PYCONFIG) --cflags)
PY3LDFLAGS   := -lboost_python-py35 $(shell $(PYCONFIG) --libs)

SRCS     := $(wildcard *.cpp)

.PHONY: all

all: libcavehull.so

libcavehull.so:   $(SRCS:%.cpp=%.o)
	$(CXX) $^ $(LDFLAGS) $(PY3LDFLAGS) -o $@

%.o:    %.cpp
	$(CXX) $< $(CXXFLAGS) $(PY3CFLAGS) -o $@

clean:
	rm -rf *.o *.so

