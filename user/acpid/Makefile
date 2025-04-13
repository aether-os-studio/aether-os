all:
	gcc $(CFLAGS) -g -O0 -c acpid.c -o acpid.o
	ld acpid.o $(shell find $(CWD)/obj/ -name "*.o") -o acpid.exec	
