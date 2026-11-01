robot: robot.cpp TCS34725.cpp
	g++ $^ -o $@ -llgpio -li2c

clean:
	rm -f robot *.o
