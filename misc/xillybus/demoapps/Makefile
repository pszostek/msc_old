CFLAGS=-g -Wall -O3

APPLICATIONS=memwrite memread streamread streamwrite

all: 	$(APPLICATIONS)

%: 	%.c
	gcc  $(CFLAGS) $@.c -o $@

clean:
	rm -f *~ $(APPLICATIONS)
