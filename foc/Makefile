CC       ?= cc
CXX      ?= c++
AR       ?= ar
CFLAGS   ?= -std=c99 -O2 -Wall -Wextra -Werror -pedantic
CXXFLAGS ?= -std=c++11 -O2 -Wall -Wextra -Werror -pedantic
LDLIBS    = -lm

# upid ships as plain C source with no build system, so there is nothing to
# link against - its one .c file is compiled straight into whatever needs it.
# Clone it next to the algo-control checkout (make check in the repo root
# fetches it), or say: make UPID_DIR=/path/to/upid
UPID_DIR ?= ../../upid
UPID_SRC  = $(UPID_DIR)/src/upid.c

CPPFLAGS += -Iinclude -I$(UPID_DIR)/include

CXX_AVAILABLE := $(shell command -v $(firstword $(CXX)) 2>/dev/null)

LIB = libfoc.a
OBJ = src/foc.o src/foc_loop.o

all: upid-check $(LIB) test_foc align

upid-check:
	@test -f $(UPID_SRC) || { \
	  echo "upid not found at $(UPID_DIR)"; \
	  echo "clone https://github.com/JonasAtelier/upid beside algo-control,"; \
	  echo "or build with: make UPID_DIR=/path/to/upid"; \
	  exit 1; }

$(LIB): $(OBJ)
	$(AR) rcs $@ $^

src/foc.o: src/foc.c include/foc.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

src/foc_loop.o: src/foc_loop.c include/foc.h
	$(CC) $(CFLAGS) $(CPPFLAGS) -c $< -o $@

test_foc: test/test_foc.c $(LIB) $(UPID_SRC)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(UPID_SRC) $(LIB) $(LDLIBS) -o $@

align: examples/align.c $(LIB) $(UPID_SRC)
	$(CC) $(CFLAGS) $(CPPFLAGS) $< $(UPID_SRC) $(LIB) $(LDLIBS) -o $@

check: upid-check test_foc cpp-smoke
	./test_foc

ifneq ($(CXX_AVAILABLE),)
cpp-smoke: test_cpp_linkage
	./test_cpp_linkage

test_cpp_linkage: test/test_cpp_linkage.cpp $(LIB) $(UPID_SRC)
	$(CXX) $(CXXFLAGS) $(CPPFLAGS) $< $(UPID_SRC) $(LIB) $(LDLIBS) -o $@
else
cpp-smoke:
	@echo "SKIP: no C++ compiler"
endif

run: align
	./align

clean:
	rm -f $(OBJ) $(LIB) test_foc test_cpp_linkage align

.PHONY: all check cpp-smoke run clean upid-check
