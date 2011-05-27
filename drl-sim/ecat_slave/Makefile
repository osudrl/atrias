# Devin Koepl

# Include public settings
include makefile.uk.liboptions

##################################################################

ECAT_NAME=ecat_slave
FAKE_MEDULLA_NAME=fake_medulla

export ECAT_NAME
export FAKE_MEDULLA_NAME

INSTALL_DIR=.

LIBS = -lrt \
       -lkpaecs_core_uspace  \
       -lodloader \
       -lkpaecs_osal \
       -lecstest_foe_pal

LDIRS  = 	-L lib/

ECAT_OBJ_FILES = $(ECAT_NAME).o
FAKE_MEDULLA_OBJ_FILES = $(FAKE_MEDULLA_NAME).o

IDIRS = -I . \
				-I include/ \
				-I `rospack find atrias`/include \
				-I `rospack find drl_library`/include

##############################################################################

all: $(ECAT_OBJ_FILES) $(FAKE_MEDULLA_OBJ_FILES)
	$(CXX) -o $(ECAT_NAME) $(ECAT_OBJ_FILES) $(LDIRS) $(LIBS)
	$(CXX) -o $(FAKE_MEDULLA_NAME) $(FAKE_MEDULLA_OBJ_FILES) $(LDIRS) $(LIBS)

clean:
	$(RM) *.o *.a *.d $(ECAT_NAME) $(FAKE_MEDULLA_NAME)

%.o : %.cpp
	$(CXX) -c -MD $(CCFLAGS) $(IDIRS) $<  -o $@

%.o : %.c
	$(CC)  -c -MD $(CCFLAGS) $(IDIRS) $<  -o $@
