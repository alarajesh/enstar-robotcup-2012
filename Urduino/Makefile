# You have to set:
# _ UPATH: For your urbi install bin (for umake)


UPATH = /usr/local/urbi

BIN_SHARED = Urduino

BIN_SRC = $(wildcard *.cc)

EXTRA_LDFLAGS = \

EXTRA_CPPFLAGS = \
	-posix -Wall -m32 \
	-I. \
	-I../common
UMAKE_ENV = \
	EXTRA_LDFLAGS="$(EXTRA_LDFLAGS)"  \
	EXTRA_CPPFLAGS="$(EXTRA_CPPFLAGS) -O2 -w"

#################
## MAKE TARGET ##
#################

all: $(BIN_SHARED)
	@echo "Compilation done."

$(BIN_SHARED):$(BIN_SRC)
	$(UMAKE_ENV) $(UPATH)/bin/umake-shared -o $@ $^

clean:
	@echo "Cleaning"
	@rm -rf *.so *.la
	@rm -rf .libs
	@rm -rf *~
