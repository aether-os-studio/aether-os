all:
	gcc -static -nostdinc -c init.c -o init.o -I$(CWD)/alibc/include
	ld init.o $(shell find $(CWD)/obj/ -name "*.o") -o initd.exec	
