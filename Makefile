test:test.cpp
	g++ -o test test.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`
