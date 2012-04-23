######################
## COMPILATION MODE ##
######################

on_test = yes
show_approx_contours = no
show_segm_coutours = yes

TESTSCRIPT = testscript.u

#####################
## VAR DECLARATION ##
#####################

RM = rm -fr

ifeq ($(on_test), yes)
	ON_TEST = -DON_TEST
endif

ifeq ($(show_approx_contours), yes)
	SHOW_APPROX_CONTOURS = -DSHOW_APPROX_CONTOURS
endif

ifeq ($(show_segm_coutours), yes)
	SHOW_SEGM_CONTOURS = -DSHOW_SEGM_CONTOURS
endif

BIN_SHARED=CDDetector

BIN_SRC=CDDetector.cc

EXTRA_LDFLAGS = \
				-L/usr/local/lib \
				-lopencv_core \
				-lopencv_highgui

EXTRA_CPPFLAGS = \
				 -m32 \
				 -I. \
				 -I$(UPATH)/include \
				 $(ON_TEST) \
				 $(SHOW_APPROX_CONTOURS) \
				 $(SHOW_SEGM_CONTOURS)

UMAKE_ENV = \
	EXTRA_LDFLAGS="$(EXTRA_LDFLAGS)" \
	EXTRA_CPPFLAGS="$(EXTRA_CPPFLAGS) -O2 -w"

#################
## MAKE TARGET ##
#################

all: $(BIN_SHARED)
	@echo "Compilation done."

$(BIN_SHARED): $(BIN_SRC)
	$(UMAKE_ENV) umake-shared -o $@ $(BIN_SRC)

run: $(BIN_SHARED)
	rlwrap urbi-launch --start -- --port 54000 -H 0.0.0.0 -i -f $(TESTSCRIPT)

gdb: $(BIN_SHARED)
	gdb --args urbi-launch --start -- --port 54000 -H 0.0.0.0 -i -f $(TESTSCRIPT)

clean:
	@echo "Cleaning"
	@$(RM) *.so *.la
	@$(RM) .libs
	@$(RM) *~

