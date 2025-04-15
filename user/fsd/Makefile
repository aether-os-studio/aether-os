all:
	gcc $(CFLAGS) -c fsd.c -o fsd.o
	gcc $(CFLAGS) -c partition.c -o partition.o
	ld fsd.o partition.o $(shell find $(CWD)/obj/ -name "*.o") -o fsd.exec	
