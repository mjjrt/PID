CC =gcc

OBJ =main.o
SRC =main.c
OUT =PIDmain

CFLAGS =-Wall -Werror -pedantic -ggdb
LIBS =-lm

all: $(OBJ)
	$(CC) $(CFLAGS) $(LIBS) $(OBJ) -o $(OUT)

$(OBJ): $(SRC)
	$(CC) $(CFLAGS) -c $(SRC)

clean:
	rm -rf *.o *.exe *.out main $(OUT) 