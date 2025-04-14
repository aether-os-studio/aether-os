all:
	gcc $(CFLAGS) -c pcid.c -o pcid.o
	gcc $(CFLAGS) -c scanner.c -o scanner.o
	ld pcid.o scanner.o $(shell find $(CWD)/obj/ -name "*.o") -o pcid.exec	
