CC := clang
DEBUG_CFLAGS := $(CFLAGS) -I./include/
RELEASE_CFLAGS := $(CFLAGS) -DNDEBUG

SRCS := plreadln.c plreadln_wordmk.c plreadln_intellisense.c
OBJS := $(SRCS:%.c=build/%.o)

.PHONY: all lib test clean

all: CFLAGS := $(DEBUG_CFLAGS)
all: $(OBJS)

lib: all
	ar rv libplreadln.a $(OBJS)

test: CFLAGS := $(DEBUG_CFLAGS)
test: $(OBJS)
	$(CC) $(DEBUG_CFLAGS) example/echo.c -o echo.out $(OBJS)

build/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -rf build libplreadln.a echo.out
