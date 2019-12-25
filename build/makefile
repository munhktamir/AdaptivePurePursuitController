tests: clean
	gcc -ggdb -Wall -c ../utils/Utils.c
	gcc -ggdb -Wall -c ../motion/MotionState.c ../motion/MotionSegment.c ../motion/MotionProfileGoal.c ../motion/MotionProfile.c \
	                   ../motion/MotionProfileGenerator.c ../motion/SetpointGenerator.c ../motion/ProfileFollower.c
	gcc -ggdb -Wall -c ../tests/test_Runner.c
	gcc -ggdb test_Runner.o Utils.o MotionState.o MotionSegment.o MotionProfileGoal.o MotionProfile.o MotionProfileGenerator.o \
	          SetpointGenerator.o ProfileFollower.o -lcheck -lm -lpthread -lrt -o mytests.out

clean:
	rm *.o
	rm *.out
