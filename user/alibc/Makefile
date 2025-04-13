CFILES := $(shell find . -name "*.[Sc]")
OFILES := $(patsubst %, $(CWD)/obj/%.o, $(CFILES))

all: $(OFILES)

$(CWD)/obj/%.S.o: %.S
	mkdir -p $(@D)
	gcc $(CFLAGS) -c $< -o $@

$(CWD)/obj/%.c.o: %.c
	mkdir -p $(@D)
	gcc $(CFLAGS) -c $< -o $@
