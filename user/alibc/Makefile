CFILES := $(shell find . -name "*.[Sc]")
OFILES := $(patsubst %, $(CWD)/obj/%.o, $(CFILES))

# User controllable C compiler command.
CC := gcc

# User controllable C flags.
CFLAGS := -g -O0 -pipe -Iinclude

all: $(OFILES)

$(CWD)/obj/%.S.o: %.S
	mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@

$(CWD)/obj/%.c.o: %.c
	mkdir -p $(@D)
	$(CC) $(CFLAGS) -c $< -o $@
