
SRCFILES := $(wildcard src/*.cc)
OBJFILES := $(SRCFILES:%.cc=%.o)
DEPFILES := $(OBJFILES:%.o=%.d)
TARGET = las2heightmap
CLEANFILES = $(DEPFILES) $(OBJFILES) $(TARGET)
CXXFLAGS ?= -O3 -g -Wall -Wextra $(shell libpng-config --cflags) $(shell pkg-config --cflags pdal)
LIBS ?= `libpng-config --ldflags` `pkg-config --libs pdal`
CXXFLAGS += -std=c++17


# User configuration
-include config.mk

all: $(TARGET)

$(TARGET): $(OBJFILES)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) -o $@ $(OBJFILES) $(LIBS)

-include $(DEPFILES)

%.o: %.cc Makefile
	$(CXX) $(CXXFLAGS) -MMD -MP -MT "$*.d" -c -o $@ $<

# Clean
clean:
	$(RM) $(CLEANFILES)

.PHONY: clean

