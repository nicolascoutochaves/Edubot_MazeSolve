all: clean Main

clean:
	rm -f Main

Main: Main.cpp
	g++ -o Main Main.cpp -O3 -Wall -Wextra -Wpedantic	
