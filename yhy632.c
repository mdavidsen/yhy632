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

FILE * logfd;


int readport(int fd, char *result) {
	int iIn = read(fd, result, 1024);
	result[iIn-1] = 0x00;
	if (iIn < 0) {
		if (errno == EAGAIN) {
			fprintf(logfd, "SERIAL EAGAIN ERROR\n");
			return 0;
		} else {
			fprintf(logfd, "SERIAL read error %d %s\n", errno, strerror(errno));
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
	cfsetispeed(&options, B115200);
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

#define YHY632_HEADER_FIRST 0xAA
#define YHY632_HEADER_SECOND 0xBB

#define YHY632_TIMEOUT 3 //seconds

#define YHY632_CMD_ERROR -3 
#define YHY632_READ_ERROR -2 
#define YHY632_CHECKSUM_ERROR -1


#define YHY632_CMD_SET_BAUDRATE  0x0101
#define YHY632_CMD_SET_NODE_NUMBER  0x0102
#define YHY632_CMD_READ_NODE_NUMBER  0x0103
#define YHY632_CMD_READ_FW_VERSION  0x0104
#define YHY632_CMD_BEEP  0x0106
#define YHY632_CMD_LED  0x0107
#define YHY632_CMD_WORKING_STATUS  0x0108  // #not used?         # data = 0x41    

#define YHY632_CMD_ANTENNA_POWER  0x010C
#define YHY632_CMD_RFU  0x0108

#define YHY632_CMD_MIFARE_REQUEST  0x0201// #  request a type of card 
                            // 0x52: request all Type A card In field, 
                            // 0x26: request idle card

#define YHY632_TYPE_MIFARE_UL  0x0044
#define YHY632_TYPE_MIFARE_1K  0x0004
#define YHY632_TYPE_MIFARE_4K  0x0002
#define YHY632_TYPE_MIFARE_DESFIRE  0x0344
#define YHY632_TYPE_MIFARE_PRO  0x0008


#define YHY632_CMD_MIFARE_ANTICOLISION  0x0202 // 0x04 -> <NUL> (00)      [4cd90080]-cardnumber
#define YHY632_CMD_MIFARE_SELECT  0x0203 //#  [4cd90080]  -> 0008
#define YHY632_CMD_MIFARE_HALT  0x0204

 
#define YHY632_CMD_MIFARE_AUTH2  0x0207 //# 60 [sector*4] [key]
//#Auth_mode:		Authenticate mode, 0x60: KEY A, 0x61: KEY B

#define YHY632_CMD_MIFARE_READ_BLOCK  0x0208 // # [block_number] 

#define YHY632_CMD_MIFARE_UL_SELECT 0x0212
#define YHY632_LED_GREEN 2
#define YHY632_LED_RED 1
#define YHY632_LED_BOTH 3


int verbose;

int do_write(int fd, const void * ptr, int n) {
    if (verbose) {
        fprintf(logfd, "write %d bytes to %d: ", n, fd);
        int i;
        for (i = 0; i < n; i++) {
            fprintf(logfd, "%.2x ",((unsigned char *)ptr)[i]);
        }
        fprintf(logfd, "\n");
    }
    return write(fd, ptr, n);
}

int synchronous_read(int fd, void * ptr, int n, int timeout_seconds  ) {
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
        if (retval) {
            if (FD_ISSET(fd,&rfds)) {            
                int this_read = read(fd, ptr + n_read, n - n_read);
                n_read+=  this_read;
                if (this_read == 0) {
                    break;
                }
                
            }
        } else {
            return -1;
        }
    }
//~ 
//~ 
    if (verbose) {
        fprintf(logfd, "read %d/%d bytes from %d: ", n_read, n, fd);
    
        int i;
        for (i = 0; i < n_read; i++) {
            fprintf(logfd, "%x ",((unsigned char *)ptr)[i]);
        }
        fprintf(logfd, "\n");
    }
    
    return n_read;
}

#define hibyte(x)       ( (unsigned char)(x>>8) )
#define lobyte(x)       ( (unsigned char)(x & 0xFF) )

int yhy632_send_command(int fd, unsigned short command, unsigned char * data, int data_len) {
    unsigned short length = 2 + 2 + 1 + data_len;
    unsigned char header0 = YHY632_HEADER_FIRST;
    unsigned char header1 = YHY632_HEADER_SECOND;
    unsigned char zero = 0;
    
    unsigned char checksum = 0x00;
    
    unsigned char * payload;

    do_write(fd, &header0, 1);
    do_write(fd, &header1, 1);

    do_write(fd, &length, 2); 

    do_write(fd, &zero, 1);
    do_write(fd, &zero, 1);
    checksum ^= 0;
    
    do_write(fd, &command, 2); 
    checksum ^= hibyte(command);
    checksum ^= lobyte(command);
    
    int i ;
    for (i = 0; i < data_len; i++) {
        do_write(fd, data + i, 1); 
        checksum ^= data[i];
        if (data[i] == 0xAA) {
           
            do_write(fd, &zero, 1);
        }
    }   
    
    do_write(fd, &checksum, 1); 
    fdatasync(fd);
}

// returns count bytes recieved, -1 otherwise
int yhy632_recieve_data(int fd, unsigned short expected_cmd, unsigned char * status, char * data) {
    unsigned char prev_byte = 0;
    unsigned char current_byte = 0;
    unsigned short cmd;
    
    while ( 1 ) { //expected_cmd != cmd
        while (1) {
            if (synchronous_read(fd, &current_byte, 1, YHY632_TIMEOUT) >= 0)  {
                if (( prev_byte  == YHY632_HEADER_FIRST ) && ( current_byte  == YHY632_HEADER_SECOND )) {
                    // header found, stop
                    if (verbose) fprintf(logfd, "header found stop\n");
                    break;
                }
            
                prev_byte = current_byte;
            } else {
                fprintf(logfd, "failed to get header!\n");
                return -2;
            } 
        }
        
        unsigned short length;
        unsigned short reserved;
        
        
        unsigned char checksum = 0;
        unsigned char original_checksum;
        
        if ( synchronous_read(fd, &length, 2, YHY632_TIMEOUT) < 1) return -2;        
        if (verbose) fprintf(logfd, "length: %d\n", length);
        
        if (length == 0x000a && expected_cmd == 0x0212) {
            length = 0x0d;
        }
                
        if ( synchronous_read(fd, &reserved, 2, YHY632_TIMEOUT) < 1) return -2;
        checksum ^= hibyte(reserved);
        checksum ^= lobyte(reserved);
        
        if ( synchronous_read(fd, &cmd, 2, YHY632_TIMEOUT) < 1) return -2;
        
        if (cmd != expected_cmd)  continue ;
        
        
        checksum ^= lobyte(cmd);     checksum ^= hibyte(cmd); 
        
        
        
        if ( synchronous_read(fd, status, 1, YHY632_TIMEOUT) < 1) return -2;
        
        
        
        
        checksum ^= *status;
        

        int i;
        //reading data
        
        if ( synchronous_read(fd, data, length - 6, YHY632_TIMEOUT) < length - 6) return -2;

        for (i=0; i < length -6; i++  ) {
            checksum ^= data[i];        
        }
        
        synchronous_read(fd, &original_checksum, 1, YHY632_TIMEOUT);
        
        
        if (original_checksum == checksum) {            
            return length - 6;
        } else {
            if (*status == 0) {
                fprintf(logfd, "checksum %d should be %d\n", original_checksum, checksum);
            }
            return -1;
        }
    }
        
    
}
// returns count bytes recieved, -1 otherwise, -2 if error is occured
int yhy632_send_recieve(int fd, unsigned short command, unsigned char * request, int request_len, unsigned char * response) {
    
    yhy632_send_command(fd, command, request, request_len);
    
    unsigned char status;
    int count_recieved = yhy632_recieve_data(fd, command, &status, response);
    
    if (count_recieved > -1) {
        if (status != 0 ) {
            return -3;
        }
    }
    
    return count_recieved;
    
}

// returns cardtype, fill serial
int yhy632_select(int fd,unsigned short * cardtype, unsigned char * serial) {
    unsigned char buffer;
    unsigned char response_buffer[16];
    buffer = 0x52;
    int serial_length;
    
    int bytes_recieved = yhy632_send_recieve(fd, YHY632_CMD_MIFARE_REQUEST, &buffer, 1, (unsigned char * ) cardtype);
    if (bytes_recieved > 0) {
        int serial_length = yhy632_send_recieve(fd, YHY632_CMD_MIFARE_ANTICOLISION, 0, 0, serial);
        if (serial_length > 0) {
            if (verbose) fprintf(logfd, "cardtype: %d\n", *cardtype);
            if (*cardtype == YHY632_TYPE_MIFARE_UL) {
                if (verbose) fprintf(logfd,"TYPE_UL!\n");
                
                //fprintf(logfd, "try...\n");
                
                serial_length = yhy632_send_recieve(fd, YHY632_CMD_MIFARE_UL_SELECT, &buffer, 0, serial);
             } else {
                int bytes_recieved2 = yhy632_send_recieve(fd, YHY632_CMD_MIFARE_SELECT, serial, serial_length, response_buffer);
            }
        }
        return  serial_length;
    } else {
        return bytes_recieved;
    }

    
}

char * tohex(unsigned char * buffer, int length) {
    int i;
    char * result = (char *) malloc ( length * 2  + 1);
    for (i = 0; i < length; i++) {
        sprintf(result + i * 2, "%.2X", buffer[i]);
    }
    return result;
}
int yhy632_serial_to_matrix_iii(unsigned short cardtype, unsigned char * serial, unsigned char serial_length, char * output) {
    char * hex;
    char postfix[4] = "XX";
    
    if (cardtype == YHY632_TYPE_MIFARE_UL) {
        char * left = tohex(serial, serial_length / 2);
        char * right = tohex(serial+ serial_length / 2 , serial_length /2 + 1);
        
        right = realloc(right, strlen(left) + strlen(right) + 1);
        strcat(right, left);
        hex = right;      
        strcpy(postfix,"UL");

    } else {
        hex =  tohex(serial, serial_length);

        if (cardtype == YHY632_TYPE_MIFARE_1K) {
            strcpy(postfix, "1K");
        } else if (cardtype == YHY632_TYPE_MIFARE_4K) {
            strcpy(postfix, "3K");
        } else if (cardtype == YHY632_TYPE_MIFARE_DESFIRE) {
            strcpy(postfix,"DF");
        } else if (cardtype == YHY632_TYPE_MIFARE_PRO) {
            strcpy(postfix,"PR");
        }
            
        
        
    }
    
    //~ char * result = (char *) malloc(  strlen(hex) + 6 + 1);
    strcpy(output,"Mifare[");
    strcat(output, hex);
    strcat(output, "] ");
    strcat(output, postfix);
    strcat(output, " X");
    
    
    return 0;
}

#define ADDRESS "192.168.0.1"
//~ #define ADDRESS "127.0.0.1"
#define PORT "80"
#define LOCATION 10

int get_socket(){
    struct addrinfo hints, *res;
    int sockfd;

    // first, load up address structs with getaddrinfo():

    memset(&hints, 0, sizeof (hints));
    res = (struct addrinfo *) malloc ( sizeof( struct addrinfo));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    getaddrinfo(ADDRESS, PORT, &hints, &res);
    // make a socket:

    sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol);

    // connect!

    if ( connect(sockfd, res->ai_addr, res->ai_addrlen)) {
        fprintf(logfd, "Connection error\n");
        return -1;
    }
    return sockfd;
}

int sock_fd;


void socket_connect() {
    while ( (sock_fd = get_socket()) == -1) {
        sleep(2);
        fprintf(logfd,"trying to reconnect...\n");
        fflush(logfd);
    } 
}



void yhy632_set_led(int fd, unsigned char led) {
    unsigned char response_buffer[255];
    unsigned char arg = 0;
    if (led != 0 ) {
        yhy632_send_recieve(fd, YHY632_CMD_LED , &arg, 1, response_buffer); //reset
    }   
        
    arg = led;
    yhy632_send_recieve(fd, YHY632_CMD_LED , &arg, 1, response_buffer); //set
}

void yhy632_beep(int fd, unsigned char delay) {
    unsigned char response_buffer[255];
    unsigned char arg = delay;
    yhy632_send_recieve(fd, YHY632_CMD_BEEP, &arg, 1, response_buffer); //reset
}

int main(int argc, char * argv[]) {
    unsigned char response_buffer[255];
    int count_recieved; 
    unsigned char delay = 100;
    verbose = 0;
    unsigned short cardtype;
  
    char * devicename = "/dev/ttyS0";
    char server_send_buffer[1024] = "";
    char server_recv_buffer[1024] = "";

    int gpio = open("/dev/gpio2", O_WRONLY);
    
    write(gpio, "LED SWITCH 1\n", 13);//set gpio to output (what ioctl??)
    
 
    //~ logfd = fopen("server.log","w");
    logfd = stdout;
    
    
    
    int argi;
    for (argi = 1; argi < argc; argi++) {
        if (strcmp("-v", argv[argi]) == 0 ) {
            verbose = 1;
        } else if (argv[argi][0] != '-') {
            devicename = argv[argi];            
        }
    }
    
     
    int fd = 0;
    fd = open(devicename, O_RDWR | O_NOCTTY    );
    int set_bits = 4;
    ioctl(fd, TIOCMSET, &set_bits);
    
    initport(fd);
    fprintf(logfd, "yhy632 reader started\n");
    yhy632_beep(fd, 50);
    yhy632_set_led(fd, YHY632_LED_RED);
    
    unsigned char version[16];
    count_recieved = yhy632_send_recieve(fd, YHY632_CMD_READ_FW_VERSION, 0, 0, version);
    fprintf(logfd, "version: %.*s\n", count_recieved, version);

    socket_connect();
    
    unsigned char serial[16];
    char matrix_serial[64];
    unsigned char prev_serial[16] = "";
    unsigned char prev_serial_len = 0;
    unsigned char prev_cardtype = 0;
    
    while (1) {
        count_recieved = yhy632_select(fd, &cardtype, serial);
        if (count_recieved > 0) {
            if ( (  memcmp(prev_serial, serial, count_recieved ) != 0 )  || (prev_serial_len != count_recieved) || (prev_cardtype != cardtype) ) {
             
                yhy632_serial_to_matrix_iii(cardtype, serial, count_recieved,matrix_serial);
                sprintf(server_send_buffer,"A %d %s\n",LOCATION,matrix_serial);
                
                
                if (verbose) {
                     fprintf(logfd, "cardtype: %d\n", cardtype);
                     fprintf(logfd, "count_recieved: %d\n", count_recieved);
                     fprintf(logfd, "serial:%s\n",matrix_serial);
                     fprintf(logfd, "server_buffer:%s\n",server_send_buffer);              
                }
                
                
                while ( send(sock_fd,server_send_buffer, strlen(server_send_buffer),MSG_NOSIGNAL ) < 0) {
                    fprintf(logfd, "error while sending\n");
                    socket_connect();
                };
                
                
                
                if (synchronous_read(sock_fd, server_recv_buffer, 8, 1) > 0) {
                
                    
                    if (strcmp(server_recv_buffer,"ACCEPTED")==0) {
                                    fprintf(logfd,"Access granted to %s \n",matrix_serial);fflush(logfd);
                                    // open the lock
                                    ioctl(gpio, 1, 2);
                                    usleep(1E3*50);
                                    ioctl(gpio, 0, 2);
                                    
                                    
                                    yhy632_set_led(fd, YHY632_LED_GREEN);
                                    yhy632_beep(fd, 100);
                                    yhy632_set_led(fd, YHY632_LED_RED);
                                    //~ usleep(1E3*500);
                                    
                                    
                                    
                    } else {
                        fprintf(logfd,"Access denied to %s, answer was %s \n",matrix_serial,server_recv_buffer );fflush(logfd);
                        
                        int i;
                        for (i = 0; i< 3; i++ ) {
                            
                            yhy632_set_led(fd, 0);
                            yhy632_beep(fd, 10);
                            yhy632_set_led(fd, YHY632_LED_RED);
                            //~ usleep(1E3*20);
                            
                        }
                    }
                }
                 
                prev_serial_len = count_recieved;
                prev_cardtype = cardtype;
                memcpy(prev_serial, serial, count_recieved);
                
                
            }
                                    
            
        } else {
            //~ printf("select error\n");
            prev_cardtype = 0;
        }
        
    }
   
   
}


