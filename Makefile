SRC=billiards.cpp
ESRC=billiard_geom.cpp
AUX=billiard_geom.h
DST=billiard
DSTT=billiard_test
OPTS=-Wall -Wextra -pedantic
LIBS=-lglut -lGLU -lGL -lm -ggdb 
CC=g++


billiard: $(SRC) $(ESRC) $(AUX)
	@$(CC) -DNO_TEST $(LIBS) $(SRC) $(ESRC) -o $(DST)
	@echo Compilation complete!

test: $(ESRC) $(AUX)
	@$(CC) $(OPTS) $(LIBS) $(ESRC) -o $(DSTT)
	@echo Compilation for testing complete!

clean: 
	@if [ -e $(DST) ] ; \
	then \
     	$(RM) $(DST) ; \
	fi;

	@if [ -e $(DSTT) ] ; \
	then \
     	$(RM) $(DSTT) ; \
	fi;
