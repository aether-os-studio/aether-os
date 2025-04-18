all:
	gcc $(CFLAGS) -c shell.c -o shell.o
	ld shell.o $(shell find $(CWD)/obj/ -name "*.o") -o shell.exec	
