all:
	gcc -static -nostdinc -mno-red-zone -g -O0 -c init.c -o init.o -I$(CWD)/alibc/include
	ld init.o $(shell find $(CWD)/obj/ -name "*.o") -o initd.exec	
