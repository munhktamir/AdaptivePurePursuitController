#test_uthread: test_uthread.c uthread.o uthread.h dstructures.o dstructures.h 
#	gcc -ggdb -c test_uthread.c uthread.c dstructures.c
#	gcc -ggdb -o test_uthread.out test_uthread.o uthread.o dstructures.o

test:
	gcc -ggdb -Wall -I . -c utils/Utils.c motion/MotionState.c motion/test_MotionState.c
	gcc -ggdb -o test_MotionState.out MotionState.o Utils.o

clean:
	del *.o
	del *.out