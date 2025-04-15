all:
	gcc $(CFLAGS) -c fsd.c -o fsd.o
	ld fsd.o $(shell find $(CWD)/obj/ -name "*.o") -o fsd.exec	
