all:
	gcc $(CFLAGS) -c acpid.c -o acpid.o
	ld acpid.o $(shell find $(CWD)/obj/ -name "*.o") -o acpid.exec	
