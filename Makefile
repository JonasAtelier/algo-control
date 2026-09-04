CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

LIB     = liblqr.a
OBJ     = src/lqr.o

all: $(LIB) balance test_lqr

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/lqr.o: src/lqr.c include/lqr.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

balance: examples/balance.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_lqr: test/test_lqr.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_lqr cpp-smoke
	./test_lqr

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: balance
	./balance

clean:
	rm -f $(OBJ) $(LIB) balance test_lqr test_cpp_linkage

.PHONY: all check cpp-smoke run clean
