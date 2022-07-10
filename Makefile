CC =gcc

OBJ =main.o
SRC =main.c
OUT =PIDmain

CFLAGS =-std=c11 -Wall -Werror -pedantic
#LIBS =-lm

all: $(OBJ)
	$(CC) $(CFLAGS) $(OBJ) -o $(OUT) -lm

$(OBJ): $(SRC)
	$(CC) $(CFLAGS) -c $(SRC) -lm

clean:
	rm -rf *.o *.exe *.out main $(OUT) 