PREFIX = /usr/local

VERSION_MAJOR = 5
VERSION_MINOR = 4

CXXFLAGS = -O2 -Wall -Wextra -pedantic -std=c++14 -fvisibility=hidden -fPIC -fno-rtti $(shell pkg-config --cflags libosrm) $(shell pkg-config --cflags python3)
LDFLAGS  = -shared -Wl,-soname,libosrmc.so.$(VERSION_MAJOR)
LDLIBS   = -lstdc++ $(shell pkg-config --libs libosrm)
