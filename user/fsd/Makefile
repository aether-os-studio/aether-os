all:
	gcc $(CFLAGS) -c fsd.c -o fsd.o
	gcc $(CFLAGS) -c cache.c -o cache.o
	ld fsd.o cache.o $(shell find $(CWD)/obj/ -name "*.o") -o fsd.exec	
