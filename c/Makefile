# Sean Smith and Jason Ruan

# C port of Sparkfun 9DOF c++ library


all: compile link run

compile:
	gcc SFE_LS9DSO.c -c SFE_LS9DSO.o -lmraa

link:
	gcc example.c SFE_LS9DSO.o -o example

clean:
	-rm example

run:
	./example