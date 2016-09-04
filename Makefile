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

chess:chess.cpp Makefile
	g++ -o chess chess.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

sfmtest:sfmtest.cpp Makefile
	g++ -o sfmtest sfmtest.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

opticlflow:opticalflow.cpp Makefile
	g++ -o opticalflow opticalflow.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

recover3d:recover3d.cpp Makefile
	g++ -o recover3d recover3d.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

featuredetect:featuredetect.cpp Makefile
	g++ -o featuredetect featuredetect.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`

viztest:viztest.cpp Makefile
	g++ -o viztest viztest.cpp `pkg-config --cflags opencv` `pkg-config --libs opencv`
