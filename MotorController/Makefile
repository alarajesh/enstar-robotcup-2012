BIN_SHARED=MotorController

BIN_SRC=MotorController.cc

UPATH=/usr/local/urbi

all: $(BIN_SHARED)
	@echo "Compilation done."

EXTRA_LDFLAGS = \
  -L/usr/local/lib/ \
  -lopencv_core -lopencv_highgui #-lopencv_video


EXTRA_CPPFLAGS =				\
  -m32 		\
  -I. \
  -I$(UPATH)/include	\

UMAKE_ENV =					\
  EXTRA_LDFLAGS="$(EXTRA_LDFLAGS)" 		\
  EXTRA_CPPFLAGS="$(EXTRA_CPPFLAGS) -O2 -w"

$(BIN_SHARED): $(BIN_SRC)
	$(UMAKE_ENV) umake-shared -o $@ $(BIN_SRC)

clean:
	@echo "Cleaning"
	@rm -rf $(BIN_SHARED).so _ubuild-$(BIN_SHARED).so $(BIN_SHARED).la
	@rm -rf .libs
	@rm -rf *~

