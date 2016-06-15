#
# Copyright (c) 2016, Intel Corporation
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

#
# This makefile declare the global variables which will be used to make output more verbose or
# more quiet. Options V=1 or C=0 can be used on the command line to change default beahvior
#
# Rules/commands inside all makfiles can be written to show more or less logs depending on
# variable VERBOSE, which is set to '1' or set to '' depending on V=1 or V=0. For recipes,
# the variable $(AT) will expend to '@' or '', enabling or not the echoing of the command
#
# For instance, usual rule is:
#   my_target:
#           @echo $(ANSI_RED)"Always printed"$(ANSI_OFF)
#           $(AT)build_cmd $@
#
# To print additional message each time a target is built:
#   my_target:
#           @[ "$(VERBOSE)" ] && echo "Extra info about $@" || true
#           $(AT)build_cmd $@
#
# To print additional message everytime the makefile is parsed:
#   ifneq ($(VERBOSE),)
#   $(info Extra stuff to print: $(STUFF))
#   endif
#
# To modify options used for a specific command
#   FLAGS += $(if $(VERBOSE),--verbose-options,--quiet-options)
#

# Use V=1 to switch from silent to verbose
V ?= 0

# Generic variables for most rules
export VERBOSE   := $(filter 1, $(V))
export AT        := $(if $(VERBOSE),,@)

# Additional specific variables are defined to handle frequent cases:
# Options for make, always exported
export MAKEFLAGS += $(if $(VERBOSE),,--no-print-directory)

# Avoid printing stats on stderr, "status=none" would be better, but is not recognized by all dd
export SILENT_DD := $(if $(VERBOSE),,2> /dev/null)

# Redirect stdout to /dev/null, working with most commands if no switch are available
export DEV_NULL  := $(if $(VERBOSE),,> /dev/null)

# Use C=0 to disable ANSI colors
C ?= 1
ifeq ($(C),1)
# Generating ESC char with printf will ensure it will be recognized by different tools
  ESC_CHAR := $(shell printf '\033')
  export ANSI_RED  := '$(ESC_CHAR)[31m'
  export ANSI_CYAN := '$(ESC_CHAR)[36m'
  export ANSI_OFF  := '$(ESC_CHAR)[0m'
else
  export ANSI_RED  :=
  export ANSI_CYAN :=
  export ANSI_OFF  :=
endif
