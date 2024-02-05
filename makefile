all: clean Main

clean:
	rm -f Main

Main: Main.cpp
	g++ -ofast Main Main.cpp -Wall -Wextra -Wpedantic	
