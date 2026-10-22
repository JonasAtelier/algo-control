CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -pedantic
CPPFLAGS += -Iinclude
LDLIBS   = -lm

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

LIB     = libimp.a
OBJ     = src/imp.o

all: $(LIB) push test_imp

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/imp.o: src/imp.c include/imp.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

push: examples/push.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

test_imp: test/test_imp.c $(LIB)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@

check: test_imp cpp-smoke
	./test_imp

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: push
	./push

clean:
	rm -f $(OBJ) $(LIB) push test_imp test_cpp_linkage

.PHONY: all check cpp-smoke run clean
