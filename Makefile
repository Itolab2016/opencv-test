test:test.cpp Makefile
	g++ -o test test.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

usbcam:usbcam.cpp Makefile
	g++ -o usbcam usbcam.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`
