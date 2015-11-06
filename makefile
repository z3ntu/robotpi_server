make prog: robotpi_socket.c
	gcc robotpi_socket.c -o robotpi_socket -lwiringPi -lbcm2835 -lpthread
