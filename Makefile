test:test.cpp Makefile
	g++ -o test test.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

usbcam:usbcam.cpp Makefile
	g++ -o usbcam usbcam.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

surftest:surf.cpp Makefile
	g++ -o surftest surf.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

surfmovie:surfmovie.cpp Makefile
	g++ -o surfmovie surfmovie.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

siftmovie:siftmovie.cpp Makefile
	g++ -o siftmovie siftmovie.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

chess:chess.cpp
	g++ -o chess chess.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

sfmtest:sfmtest.cpp Makefile
	g++ -o sfmtest sfmtest.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`
