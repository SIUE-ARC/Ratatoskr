# This is a general use makefile for librobotcontrol projects written in C.
# Just change the target name to match your main source code filename.

MACHINE := not_blue

TARGET = test_vl53

# compiler and linker binaries

ifdef ($(MACHINE),blue)
CC  :=  gcc
LINKER	:= gcc
endif


ifeq ($(MACHINE),not_blue)
CC  := arm-linux-gnueabihf-g++
LINKER	:=  arm-linux-gnueabihf-g++
endif


# compiler and linker flags

ifeq ($(MACHINE),blue)
WFLAGS		:= -Wall -Wextra -Werror=float-equal -Wuninitialized -Wunused-variable -Wdouble-promotion
CFLAGS		:= -g -c -Wall -std=gnu++11
LDFLAGS		:= -pthread -lm -lrt -W1 -V -l:librobotcontrol.so.1
SOURCES		:= $(wildcard *.c)
INCLUDES	:= $(wildcard *.h)
OBJECTS		:= $(SOURCES:$%.c=$%.o)
prefix		:= /usr/local
RM		:= rm -f
INSTALL		:= install -m 4755
INSTALLDIR	:= install -d -m 755
SYMLINK		:= ln -s -f
SYMLINKDIR	:= /etc/robotcontrol
SYMLINKNAME	:= link_to_startup_program
endif


ifeq ($(MACHINE),not_blue)
INCDIR     	:=  ../librobotcontrol-master/library/include
WFLAGS		:= -Wall -Wextra -Werror=float-equal -Wuninitialized -Wunused-variable -Wdouble-promotion
CFLAGS		:= -g -c -Wall -I $(INCDIR) -std=gnu++11
LDFLAGS		:= -L ../usr_lib -pthread -lm -lrt -l:librobotcontrol.so.1
SOURCES		:= $(wildcard *.c)
INCLUDES	:= $(wildcard *.h)
OBJECTS		:= $(SOURCES:$%.c=$%.o)
prefix		:= /usr/local
RM		:= rm -f
INSTALL		:= install -m 4755
INSTALLDIR	:= install -d -m 755
SYMLINK		:= ln -s -f
SYMLINKDIR	:= /etc/robotcontrol
SYMLINKNAME	:= link_to_startup_program
endif


# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) -o $@ $(OBJECTS) $(LDFLAGS)
	@echo "Made: $@"


# compiling command
$(OBJECTS): %.o : %.c $(INCLUDES)
	@$(CC) $(CFLAGS) $(WFLAGS) $(DEBUGFLAG) $< -o $@
	@echo "Compiled: $@"

all:	$(TARGET)

debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo " "
	@echo "$(TARGET) Make Debug Complete"
	@echo " "

install:
	@$(MAKE) --no-print-directory
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/bin
	@echo "$(TARGET) Install Complete"

clean:
	@$(RM) $(OBJECTS)
	@$(RM) $(TARGET)
	@echo "$(TARGET) Clean Complete"

uninstall:
	@$(RM) $(DESTDIR)$(prefix)/bin/$(TARGET)
	@echo "$(TARGET) Uninstall Complete"

runonboot:
	@$(MAKE) install --no-print-directory
	@$(SYMLINK) $(DESTDIR)$(prefix)/bin/$(TARGET) $(SYMLINKDIR)/$(SYMLINKNAME)
	@echo "$(TARGET) Set to Run on Boot"

