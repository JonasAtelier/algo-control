# Runs each library's own Makefile in turn, stopping at the first failure.
# Every library still builds and tests on its own; this only saves the cd.
LIBS   := $(patsubst %/Makefile,%,$(wildcard */Makefile))
NOMAKE := $(filter-out $(LIBS),$(patsubst %/README.md,%,\
            $(wildcard */README.md)))

# foc needs upid, which lives in its own repo. Unless you name a checkout with
# make check UPID_DIR=/abs/path/to/upid, it is cloned once into .deps/ at the
# commit foc was tested against.
UPID_URL ?= https://github.com/JonasAtelier/upid.git
UPID_REV  = 4d96e5058268b18321e6141a240d43ad65b70d80
UPID_DIR ?= $(CURDIR)/.deps/upid
export UPID_DIR

check: $(UPID_DIR)/src/upid.c

check clean:
	@for d in $(NOMAKE); do echo "SKIP: $$d (no Makefile)"; done
	@for d in $(LIBS); do $(MAKE) -C $$d $@ || exit 1; done

# --no-checkout: upid.c only appears once the pinned commit is checked out, so
# an interrupted fetch is finished on the next run, never mistaken for done.
$(UPID_DIR)/src/upid.c:
	test -d $(UPID_DIR) || git clone -q --no-checkout $(UPID_URL) $(UPID_DIR)
	git -C $(UPID_DIR) checkout -q $(UPID_REV)

.PHONY: check clean
