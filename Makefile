SRC=billiard_geom.cpp
AUX=billiard_geom.h
DST=billiard
OPTS=-Wall -Wextra -pedantic
LIBS=-lglut -lGLU -lGL -lm -ggdb 
CC=g++


pacman: $(SRC) $(AUX)
	@$(CC) $(OPTS) $(LIBS) $(SRC) -o $(DST)
	@echo Compilation complete!

clean: $(DST)
	$(RM) $(DST)
