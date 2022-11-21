src_vis = src/visualizer.cpp src/serial.cpp src/glad.c
src_calib = src/calibration.cpp src/serial.cpp
ldflags = -lglfw -ldl -lGL -lGLEW 

all:
	g++ -o visualizer -Iinclude ${src_vis} ${ldflags} 

calib:
	g++ -o calibration -Iinclude ${src_calib}

clean:
	rm -f visualizer calibration 
