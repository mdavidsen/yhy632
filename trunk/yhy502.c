#include <termios.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/signal.h>
#include <sys/types.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/termios.h> /* POSIX terminal control definitions */
#include <sys/socket.h> /* POSIX terminal control definitions */
#include <netdb.h> 
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <sys/ioctl.h>



int readport(int fd, char *result) {
	int iIn = read(fd, result, 1024);
	result[iIn-1] = 0x00;
	if (iIn < 0) {
		if (errno == EAGAIN) {
			printf("SERIAL EAGAIN ERROR\n");
			return 0;
		} else {
			printf("SERIAL read error %d %s\n", errno, strerror(errno));
			return 0;
		}
	}                    
	return 1;
}

int initport(int fd) {
	struct termios options;
	// Get the current options for the port...
	tcgetattr(fd, &options);
	// Set the baud rates to 19200...
	cfsetispeed(&options, B19200);
	//cfsetospeed(&options, B9600);
	// Enable the receiver and set local mode...
	options.c_cflag |= (CLOCAL | CREAD);

    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  
	// Set the new options for the port...
	tcsetattr(fd, TCSANOW, &options);
    
    fcntl(fd, F_SETFL, O_SYNC); //blocking read
	return 1;
}

#define YHY502_HEADER_FIRST 0xAA
#define YHY502_HEADER_SECOND 0xBB

#define YHY502_CMD_READ_CARD_SERIAL 0x20
#define YHY502_CMD_READ_MODULE_SERIAL 0x02

#define YHY502_TIMEOUT 1 //seconds

#define YHY502_CMD_ERROR -3 
#define YHY502_READ_ERROR -2 
#define YHY502_CHECKSUM_ERROR -1

int do_write(int fd, const void * ptr, int n) {
    //~ printf("write %d bytes to %d: ", n, fd);
    //~ int i;
    //~ for (i = 0; i < n; i++) {
        //~ printf("%x ",((unsigned char *)ptr)[i]);
    //~ }
    //~ printf("\n");
    return write(fd, ptr, n);
}

int yhy502_synchronous_read(int fd, void * ptr, int n, int timeout_seconds  ) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);

    int n_read = 0;
    int retval;
    
    struct timeval timeout;
    
    timeout.tv_sec = timeout_seconds; 
    timeout.tv_usec = 0;
    
    while (n_read <  n) {
        retval = select(fd + 1,&rfds, NULL, NULL, &timeout);
        //~ printf("retval: %d\n", retval);
        if (retval) {
            if (FD_ISSET(fd,&rfds)) {            
                n_read+=  read(fd, ptr + n_read, n - n_read);
                
            }
        } else {
            return -1;
        }
    }
//~ 
//~ 
    printf("read %d/%d bytes from %d: ", n_read, n, fd);
    
    int i;
    for (i = 0; i < n_read; i++) {
        printf("%x ",((unsigned char *)ptr)[i]);
    }
    printf("\n");
    
    return n_read;
}


int yhy502_send_command(int fd, unsigned char command, char * data, int data_len) {
    unsigned char length = 2 + data_len;
    unsigned char header0 = YHY502_HEADER_FIRST;
    unsigned char header1 = YHY502_HEADER_SECOND;
    
    unsigned char checksum = 0;
    
    do_write(fd, &header0, 1);
    do_write(fd, &header1, 1);
    
    do_write(fd, &length, 1);  checksum ^= length; 
    do_write(fd, &command, 1); checksum ^= command;
    
    int i ;
    for (i = 0; i < data_len; i++) {
        do_write(fd, data + i, 1); 
        checksum ^= data[i];
    }   
    do_write(fd, &checksum, 1); 
    fdatasync(fd);
}

// returns count bytes recieved, -1 otherwise
int yhy502_recieve_data(int fd, unsigned char * status, char * data) {
    unsigned char prev_byte = 0;
    unsigned char current_byte = 0;
    
    while (1) {
        if (yhy502_synchronous_read(fd, &current_byte, 1, YHY502_TIMEOUT) >= 0)  {
            if (( prev_byte  == YHY502_HEADER_FIRST ) && ( current_byte  == YHY502_HEADER_SECOND )) {
                // header found, stop
                break;
            }
        
            prev_byte = current_byte;
        } else {
            printf("failed to get header!\n");
            return -2;
        } 
    }
    
    unsigned char length;
    unsigned char checksum = 0;
    unsigned char original_checksum;
    
    if ( yhy502_synchronous_read(fd, &length, 1, YHY502_TIMEOUT) < 1) return -2;
    checksum^= length;
    
    if ( yhy502_synchronous_read(fd, status, 1, YHY502_TIMEOUT) < 1) return -2;
    checksum ^= (*status);
    
    int i;
    //reading data

    if ( yhy502_synchronous_read(fd, data, length - 2, YHY502_TIMEOUT) < length - 2) return -2;

    for (i=0; i < length -2; i++  ) {
        checksum ^= data[i];        
    }
    
    yhy502_synchronous_read(fd, &original_checksum, 1, YHY502_TIMEOUT);
    
    //~ printf("checksum %d should be %d\n", original_checksum, checksum);
    if (original_checksum == checksum) {
        return length - 2;
    } else {
        return -1;
    }
        
    
}
// returns count bytes recieved, -1 otherwise, -2 if error is occured
int yhy502_send_recieve(int fd, unsigned char command, char * request, int request_len, char * response) {
    
    yhy502_send_command(fd, command, request, request_len);
    
    unsigned char status;
    int count_recieved = yhy502_recieve_data(fd, &status, response);
    
    if (count_recieved > -1) {
        if (status != command ) {
            return -3;
        }
    }
    
    return count_recieved;
    
}

int main() {
    char devicename[] = "/dev/ttyUSB0";
    int fd = 0;
    fd = open(devicename, O_RDWR | O_NOCTTY    );
    int set_bits = 4;
    ioctl(fd, TIOCMSET, &set_bits);
    
    initport(fd);
    printf("YHY502 reader started\n");
    
    //~ yhy502_send_command(fd, YHY502_CMD_READ_CARD_SERIAL, 0, 0);
    
    unsigned char response_buffer[255];
    int count_recieved; 
    
    count_recieved = yhy502_send_recieve(fd, YHY502_CMD_READ_CARD_SERIAL, 0,0, response_buffer);
    printf("recieved %d: ", count_recieved);
    int i;
    for (i = 0; i < count_recieved; i++) {
        printf("%x ",response_buffer[i]);
    }
    printf("\n");
   
}


