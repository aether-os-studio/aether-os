all:
	gcc $(CFLAGS) -g -O0 -c init.c -o init.o
	ld init.o $(shell find $(CWD)/obj/ -name "*.o") -o initd.exec	
