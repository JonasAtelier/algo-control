CC      ?= cc
AR      ?= ar
CFLAGS  ?= -std=c99 -O2 -Wall -Wextra -pedantic
CPPFLAGS += -Iinclude
LDLIBS  = -lm

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

check: test_imp
	./test_imp

run: push
	./push

clean:
	rm -f $(OBJ) $(LIB) push test_imp

.PHONY: all check run clean
