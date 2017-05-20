################################################################################
#
# flipflip's Arduino Uno Stuff for avr-gcc and avr-libc
#
# Copyright (c) 2017 Philippe Kehl (flipflip at oinkzwurgl dot org)
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
################################################################################

################################################################################
# configuration

# verbose build?
VERBOSE     ?= 0

# debugging and programming UART TX0/RX0
PROGPORT    ?= /dev/ttyACM0
DEBUGPORT   ?= $(PROGPORT)
PROGSPEED   ?= 115200
DEBUGSPEED  ?= $(PROGSPEED)

# avrdude settings programming mode (Arduino, AVR ISP, USBtinyISP, etc.)
AVRDUDEARGS ?= -c arduino -P $(DEBUGPORT) -b $(PROGSPEED) -D -V
#AVRDUDEARGS ?= -P /dev/ttyUSB0 -b 19200 -c avrisp -V
#AVRDUDEARGS ?= -c usbtiny -y -V -B 1

# version of the printf() to include (min, norm, float)
PRINTF      ?= float

# version of the scanf() to include (min, norm, float)
SCANF       ?= float

# output directory
OBJDIR      ?= obj

# project files
PROJFILES   ?= $(wildcard *.c)

################################################################################
# toolchain

PERL        := perl
SED         := sed
SHELL       := bash
TOUCH       := touch
RM          := rm
MKDIR       := mkdir
HEAD        := head
AWK         := gawk
TEE         := tee
DOXYGEN     := doxygen
CAT         := cat
TAIL        := tail
TAR         := tar
LS          := ls
DATE        := date
FMT         := fmt
SVN         := svn
CHMOD       := chmod
STRINGS     := strings

# toolchain
AVRPREFIX   := avr-
STRIP       := $(AVRPREFIX)strip
STRINGS     := $(AVRPREFIX)strings
CC          := $(AVRPREFIX)gcc
AS          := $(AVRPREFIX)as
LD          := $(AVRPREFIX)ld
NM          := $(AVRPREFIX)nm
OBJCOPY     := $(AVRPREFIX)objcopy
OBJDUMP     := $(AVRPREFIX)objdump
SIZE        := $(AVRPREFIX)size
GDB         := $(AVRPREFIX)gdb

AVRDUDE     := avrdude

LC_ALL      := C

# target platform
ATMEGANR    := 328p
MCU         := atmega$(ATMEGANR)
F_CPU       := 16000000UL

################################################################################

# verbosity helpers
ifeq ($(VERBOSE),1)
V =
V1 =
V2 =
V12 =
OBJCOPY += -v
RM += -v
else
V = @
V1 = > /dev/null
V2 = 2> /dev/null
V12 = 2>&1 > /dev/null
endif

# disable fancy stuff for dumb terminals (e.g. Emacs compilation window)
fancyterm := true
ifeq ($(TERM),dumb)
fancyterm := false
endif
ifeq ($(TERM),)
fancyterm := false
endif
ifeq ($(fancyterm),true)
HLR="[31m"
HLG="[32m"
HLY="[33m"
HLV="[35m"
HLM="[1\;36m"
HLO="[m"
else
HLR=
HLG=
HLY=
HLV=
HLM=
HLO=
endif

###############################################################################

# preprocessor defines
DEFS        += -DF_CPU=$(F_CPU)
DEFS        += -DATOM_FLIPFLIP
DEFS        += -DATOM_USE_SLEEP
DEFS        += -DATOM_STACK_CHECKING

# compiler flags
CFLAGS      += -mmcu=$(MCU) -g3 -O1
CFLAGS      += -pipe -std=gnu99 #--param max-inline-insns-single=500
CFLAGS      += -Wall -Wpadded -Wextra -Wstrict-prototypes # -Werror -Wpedantic
CFLAGS      += -fno-common -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums
CFLAGS      += -Wunused -Wunused-parameter -Wwrite-strings # -Wold-style-definitions
CFLAGS      += -Wnested-externs -Wformat=2 -Wunused-result # -Wredundant-decls
CFLAGS      += -ffunction-sections -Wjump-misses-init -Wlogical-op
CFLAGS      += -fno-strict-aliasing -fdata-sections -Wunused-variable
CFLAGS      += -Wmissing-prototypes -Werror-implicit-function-declaration
CFLAGS      += -Wpointer-arith -Wchar-subscripts -Wcomment -Wimplicit-int -Wmain -Wparentheses
CFLAGS      += -Wsequence-point -Wreturn-type -Wswitch -Wtrigraphs -Wuninitialized -Wunknown-pragmas
CFLAGS      += -Wfloat-equal -Wundef -Wshadow -Wsign-compare -Waggregate-return
CFLAGS      += -Wmissing-declarations -Wformat -Wmissing-format-attribute -Wno-deprecated-declarations
CFLAGS      += -Wnested-externs -Winline -Wlong-long -Wcast-align
#CFLAGS      += -Wunreachable-code -Wbad-function-cast -Wpacked
#CFLAGS      += -Wa,--gstabs
#CFLAGS      += -ffunction-sections -fdata-sections

# linker flags
LDFLAGS     += -mmcu=$(MCU) -Wl,-Map=$(OBJDIR)/$(PROJECT).map,--cref -Wl,-gc-sections
#LDFLAGS     += -Wl,--defsym=__stack=sMainInitStack
LDLIBS      += -lm
#LDLIBS      += -nostdlib -lm -lc -lgcc -lc

# librarires linker flags
ifeq ($(PRINTF),min)
#$(info lo-fi printf())
LDLIBS      += -Wl,-u,vfprintf -lprintf_min
else ifeq ($(PRINTF),norm)
#$(info normal printf())
LDLIBS      +=
else ifeq ($(PRINTF),float)
#$(info hi-fi printf() (incl. float conversion))
LDLIBS      += -Wl,-u,vfprintf -lprintf_flt -lm
else
$(error Wrong value for PRINTF variable!)
endif

ifeq ($(SCANF),min)
#$(info lo-fi scanf())
LDLIBS      += -Wl,-u,vfscanf -lscanf_min
else ifeq ($(SCANF),norm)
#$(info  normal scanf())
LDLIBS      +=
else ifeq ($(SCANF),float)
#$(info hi-fi scanf() (incl. float conversion))
LDLIBS      += -Wl,-u,vfscanf -lscanf_flt
else
$(error Wrong value for SCANF variable!)
endif

################################################################################

default:
	@echo "Make what? Try 'make help'."

# source code files to compile
SRCFILES    += $(PROJFILES)

# include directories and flags
SRCDIRS     += $(sort $(dir $(SRCFILES))) $(OBJDIR)
INCFLAGS    += $(strip $(foreach dir, $(SRCDIRS), -I$(dir)))

# object files to generate (compile, assemble)
OFILES      +=

# makes compile rule for .c files
define avrMakeCompileRuleC
#$ (info avrMakeCompileRuleC $(1) --> $(OBJDIR)/$(subst /,__,$(subst $(FFDIR)/,,$(patsubst %.c,%.o,$(1)))))
OFILES += $(OBJDIR)/$(subst /,__,$(subst ..,,$(subst $(FFDIR)/,,$(patsubst %.c,%.o,$(1)))))
$(OBJDIR)/$(subst /,__,$(subst ../,__,$(subst $(FFDIR)/,,$(patsubst %.c,%.o,$(1))))): $(1) $(MAKEFILE_LIST)
	@echo "$(HLY)C $$< $(HLR)$$@$(HLO)"
	$(V)$(CC) -c -o $$@ $$(CFLAGS) $(DEFS) $(INCFLAGS) $$< -MD -MF $$(@:%.o=%.d) -MT $$@ -Wa,-adhlns=$$(@:.o=.lst)
endef

# makes compile rule for .S files
define avrMakeCompileRuleS
#$ (info avrMakeCompileRuleC $(1) --> $(OBJDIR)/$(subst /,__,$(subst $(FFDIR)/,,$(patsubst %.S,%.o,$(1)))))
OFILES += $(OBJDIR)/$(subst /,__,$(subst $(FFDIR)/,,$(patsubst %.S,%.o,$(1))))
$(OBJDIR)/$(subst /,__,$(subst $(FFDIR)/,,$(patsubst %.S,%.o,$(1)))): $(1) $(MAKEFILE_LIST)
	@echo "$(HLY)A $$< $(HLR)$$@$(HLO)"
	$(V)$(CC) -c -o $$@ $$(CFLAGS) $(DEFS) $(INCFLAGS) $$< -MD -MF $$(@:%.o=%.d) -MT $$@ -Wa,-adhlns=$$(@:.o=.lst)
endef

# makes compile rule for .s files
define avrMakeCompileRules
#$ (info avrMakeCompileRuleC $(1) --> $(OBJDIR)/$(subst /,__,$(subst $(FFDIR)/,,$(patsubst %.s,%.o,$(1)))))
OFILES += $(OBJDIR)/$(subst /,__,$(subst $(FFDIR)/,,$(patsubst %.s,%.o,$(1))))
$(OBJDIR)/$(subst /,__,$(subst $(FFDIR)/,,$(patsubst %.s,%.o,$(1)))): $(1) $(MAKEFILE_LIST)
	@echo "$(HLY)A $$< $(HLR)$$@$(HLO)"
	$(V)$(CC) -c -o $$@ -x assembler-with-cpp $$(CFLAGS) $(DEFS) $(INCFLAGS) $$< -MD -MF $$(@:%.o=%.d) -MT $$@ -Wa,-adhlns=$$(@:.o=.lst)
endef

# create compile rules and populate $(OFILES) list
$(foreach sfile, $(filter %.s,$(SRCFILES)), $(eval $(call avrMakeCompileRules,$(sfile)))) # watch the spaces!
$(foreach Sfile, $(filter %.S,$(SRCFILES)), $(eval $(call avrMakeCompileRuleS,$(Sfile)))) # watch the spaces!
$(foreach cfile, $(filter %.c,$(SRCFILES)), $(eval $(call avrMakeCompileRuleC,$(cfile)))) # watch the spaces!

# dependency files
DFILES := $(patsubst %.o,$(OBJDIR)/%.d,$(notdir $(OFILES)))

# load available dependency files
ifneq ($(MAKECMDGOALS),debugmf)
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),distclean)
-include $(DFILES)
endif
endif
endif

################################################################################

# make output directory
$(OBJDIR)/.dummy:
	@echo "$(HLG)* $(OBJDIR)/$(HLO)"
	$(V)$(MKDIR) -p $(OBJDIR)
	$(V)$(TOUCH) $@

################################################################################

# image file derivatives to generate
IMGFILESEXT += .elf .lss .lst .sym .hex .srec .bin .strings
IMGFILESEXT += _eeprom.hex _eeprom.srec _eeprom.bin
IMGFILESEXT += .size
IMGFILESEXT += #.cof

IMGNAME := img
IMGDERIVS   := $(addprefix $(OBJDIR)/$(IMGNAME), $(IMGFILESEXT))

# image
.PHONY: img
img: $(OFILES) $(OBJDIR)/.dummy $(OBJDIR)/macros.txt $(IMGDERIVS)

# always compile main.c on all changes (for the version info)
$(filter %src__main.o, $(OFILES)): $(filter-out %src__main.o, $(OFILES))

# we generate .o files from .c and .S files
$(OFILES): $(OBJDIR)/.dummy Makefile

# generate list of predefined macros
$(OBJDIR)/macros.txt: $(OBJDIR)/$(IMGNAME).elf # $(MAKEFILE_LIST)
	@echo "$(HLV)G $@ $(HLO)"
	$(V)echo | $(CC) $(DEFS) $(CFLAGS) -dM -E - > $@

# link into .elf file
$(OBJDIR)/$(IMGNAME).elf: $(OFILES)
	@echo "$(HLM)L $(@)$(HLO) $(HLR)$(HLO)"
	$(V)$(CC) -o $@ $(LDFLAGS) $(OFILES) $(LDLIBS)

# .size from .elf (section sizes)
SIZE += --mcu=$(MCU)
$(OBJDIR)/$(IMGNAME).size: $(OBJDIR)/$(IMGNAME).elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(SIZE) -Ax $(OBJDIR)/$(IMGNAME).elf > $@
	$(V)$(SIZE) -Bd $(OBJDIR)/$(IMGNAME).elf >> $@
	$(V)$(SIZE) -Bx $(OBJDIR)/$(IMGNAME).elf >> $@
	$(V)$(SIZE) -AC $(OBJDIR)/$(IMGNAME).elf >> $@
	$(V)$(SIZE) -Bd $(OFILES) >> $@
	$(V)$(SIZE) -AC $(OBJDIR)/$(IMGNAME).elf

# .lss from .elf (extended listing)
%.lss: %.elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(OBJDUMP) -t -h $< > $@

# .lst from .elf (disassembly)
%.lst: %.elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(OBJDUMP) -h -S $< > $@

# .sym from .elf (symbol table)
%.sym: %.elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(NM) -n $< > $@

# .hex/.srec/.bin from .elf
OBJCOPYIMG := $(OBJCOPY) # -j .text -j .data
%.hex: %.elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(OBJCOPYIMG) -O ihex $< $@
%.srec: %.elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(OBJCOPYIMG) -O srec $< $@
	$(V)$(CHMOD) -x $@
%.bin: %.elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(OBJCOPYIMG) -O binary $< $@
	$(V)$(CHMOD) -x $@

# .strings list of strings in binary
%.strings: %.bin
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(STRINGS) $< > $@

OBJCOPYEEP := $(OBJCOPY) -j .eeprom --set-section-flags=.eeprom="alloc,load" \
	--change-section-lma .eeprom=0

# _eeprom.hex from .elf (eeprom contents)
%_eeprom.hex: %.elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(OBJCOPYEEP) -O ihex $< $@ $(V2)

# _eeprom.srec from .elf (eeprom contents)
%_eeprom.srec: %.elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(OBJCOPYEEP) -O srec $< $@ $(V2)
	$(V)$(CHMOD) -x $@

# _eeprom.bin from .elf (eeprom contents)
%_eeprom.bin: %.elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(OBJCOPYEEP) -O binary $< $@ $(V2)
	$(V)$(CHMOD) -x $@

# .cof from .elf
%.cof: %.elf
	@echo "$(HLV)G $@$(HLO)"
	$(V)$(OBJCOPY) --debugging --change-section-address .data-0x800000 --change-section-address .bss-0x800000 --change-section-address .noinit-0x800000 --change-section-address .eeprom-0x810000 -O coff-avr $< $@

################################################################################

# program/flash
.PHONY: prog
prog: img
	@echo "$(HLY)P $(OBJDIR)/$(IMGNAME).hex$(HLO)"
	$(V)$(AVRDUDE) $(AVRDUDEARGS) -p m$(ATMEGANR) -U flash:w:$(OBJDIR)/$(IMGNAME).hex
#-U eeprom:w:$(OBJDIR)/$(IMGNAME).eep

# run serial debug tool
.PHONY: debug
debug: debug.pl $(DEBUGPORT)
	@echo "$(HLG)R $^ $(DEBUGSPEED)$(HLO)"
	$(V)$(PERL) $^ $(DEBUGSPEED)

# show AVR chip fuses and stuff
.PHONY: chipinfo
chipinfo:
	$(V)$(AVRDUDE) $(AVRDUDEARGS) -p m$(ATMEGANR) -v

# docu
.PHONY: doc
doc: $(OBJDIR)/.dummy $(SRCFILES) Doxyfile
	$(V)$(RM) -rf $(OBJDIR)/html
	$(V)$(MKDIR) $(OBJDIR)/html
	@echo "$(HLG)R Doxygen$(HLR) $(OBJDIR)/html/index.html$(HLO)"
	$(V)$(DOXYGEN) Doxyfile | $(TEE) $(OBJDIR)/doxygen.log $(V1)
	$(V)$(SED) -i -r -e 's@$(PWD)/@@g' -e '/^$$/d' $(OBJDIR)/doxygen_warnings.log $(OBJDIR)/doxygen.log
	$(V)if [ -s $(OBJDIR)/doxygen_warnings.log ]; then \
		$(CAT) $(OBJDIR)/doxygen_warnings.log; \
	fi

# cleanup
.PHONY: clean
clean:
	@if [ -d "$(OBJDIR)" ]; then echo "$(HLM)* removing $(OBJDIR) dir$(HLO)"; fi
ifneq ($(OBJDIR),)
	$(V)if [ -d "$(OBJDIR)" ]; then $(RM) -rf $(OBJDIR); fi
else
	@echo "$(HLR)ERROR: no OBJDIR!!!$(HLO)"
endif

.PHONY: distclean
distclean:
	@echo "$(HLM)* removing obj* *~$(HLO)"
	$(V)$(RM) -rf obj*
	$(V)$(RM) -f *~

################################################################################

.PHONY: help
help:
	@echo "Usage: make <thing> [VERBOSE=1]"
	@echo
	@echo "Where possible <thing>s to make are:"
	@echo
	@echo "img   -- compile and link"
	@echo "prog  -- program the Arduino"
	@echo "debug -- show debug output"
	@echo "doc   -- generate documentation"
	@echo "clean -- clean up (remove generated files)"
	@echo
	@echo "Add VERBOSE=1 for verbose output. See top of Makefile for more settings."
	@echo
	@echo "E.g.:"
	@echo
	@echo "  make img && make prog && make debug"
	@echo

###############################################################################
# eof
