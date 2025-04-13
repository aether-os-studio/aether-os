all:
	gcc $(CFLAGS) -g -O0 -c pcid.c -o pcid.o
	ld pcid.o $(shell find $(CWD)/obj/ -name "*.o") -o pcid.exec	
