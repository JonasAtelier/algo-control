CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

LIB     = libadrc.a
OBJ     = src/adrc.o

all: $(LIB) motor test_adrc

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/adrc.o: src/adrc.c include/adrc.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

motor: examples/motor.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_adrc: test/test_adrc.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_adrc cpp-smoke
	./test_adrc

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: motor
	./motor

clean:
	rm -f $(OBJ) $(LIB) motor test_adrc test_cpp_linkage

.PHONY: all check cpp-smoke run clean
