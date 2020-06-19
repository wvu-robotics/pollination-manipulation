#include <manipulation_pollinator/maestro.hpp>

// initializes maestro board of choosing
int maestro::init(const char * device){
	fd = open(device, O_RDWR | O_NOCTTY);
	struct termios options;
 	tcgetattr(fd, &options);
 	options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
 	options.c_oflag &= ~(ONLCR | OCRNL);
 	options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
 	tcsetattr(fd, TCSANOW, &options);
	if (fd == -1){
		perror(device);
  		return -1;
	}

	//having issues with config function

	return 0;
}


//takes in a file called maestro.conf and
//configures the board to that configuration
// file
int maestro::config(){
	printf("Configuring Maestro...\n");
	char* const argv[]= {"./Config/UscCmd", "--configure", "Config/maestro.conf",NULL};
	if(execv("./Config/UscCmd",argv) == -1){
		perror("failed to configure\n");
		return -1;
	}
	printf("Configured Device\n");
	return 0;
}

//returns error code if communication unsuccessful
//shifts high byte to the left (256*response)
int maestro::getError(){
  unsigned char command[] = {0xA1};
  if(write(fd, command, sizeof(command)) == -1){
		perror("error writing");
  	return -1;
  }
  unsigned char response[2];
  if(read(fd,response,2) != 2){
    perror("error reading");
    return -1;
  }
	return response[0] + 256*response[1];
}

//gets current position of each motor
int maestro::getPosition(unsigned char channel){
  unsigned char command[] = {0x90, channel};
  if(write(fd, command, sizeof(command)) == -1){
    perror("error writing");
    return -1;
  }

  unsigned char response[2];
  if(read(fd,response,2) != 2){
    perror("error reading");
    return -1;
  }
   return response[0] + 256*response[1];
}

//sets values for pulse width to change position
//target is whatever pwm number is in microseconds times 4
//ex. 1500 µs (1500×4 =6000
int maestro::setTarget(unsigned char channel, unsigned short target){
  unsigned char command[] = {0x84, channel, target & 0x7F, target >> 7 & 0x7F};
  if (write(fd, command, sizeof(command)) == -1){
		perror("error writing");
  	return -1;
 	}
 	return 0;
}

//sets up to 12 targets, first byte is target number, second is the lowest channel in use, remainder are input as a vector
int maestro::setMultipleTargets(unsigned char target_num, unsigned char low_channel, unsigned short targets[12]){
	unsigned char command[] = {0x9F, target_num, low_channel, targets[0] & 0x7F, targets[0] >> 7 & 0x7F, targets[1] & 0x7F, targets[1]>>7 & 0x7F, targets[2] & 0x7F, targets[2]>>7 & 0x7F, targets[3] & 0x7F, targets[3]>>7 & 0x07F,targets[4] & 0x07F, targets[4]>>7 & 0x7F,targets[5] & 0x7F, targets[5]>>7 & 0x7F,targets[6] & 0x07F, targets[6]>>7 & 0x07F,targets[7] & 0x07F, targets[7]>>7 & 0x7F,targets[8] & 0x7F, targets[8]>>7 & 0x07F, targets[9] & 0x07F, targets[9]>>7 & 0x07F, targets[10] & 0x7F, targets[10]>>7 & 0x7F, targets[11] & 0x7F, targets[11]>>7 & 0x7F};
	if (write(fd, command, sizeof(command)) == -1){
		perror("error writing");
  	return -1;
 	}
 	return 0;
}

// Sets the speed of the Maestro channel.
// See the "Serial Servo Commands" section of the user's guide.
// Corresponds to speed* 0.25microseconds/10millisecond in the PWM duty cycle.
// A speed of zero makes the PWM signal change immediately.
int maestro::setSpeed(unsigned char channel, unsigned short speed){
   unsigned char command[] = {0x87, channel, speed & 0x7F, speed >> 7 & 0x7F};
	 printf("whatever you want");
   if(write(fd, command, sizeof(command)) == -1){
   perror("error writing");
   return -1;
   }
	return 0;
}

//set 3 targets on the maestro
//written by mo so probably shit
int maestro::setPosition(unsigned char channel, unsigned short position1, unsigned short position2, unsigned short position3){
    unsigned char command[] = {0x9F,(unsigned char)3,channel,position1 & 0x7F, position1 >> 7 & 0x7F,position2 & 0x7F, position2 >> 7 & 0x7F,position3 & 0x7F, position3 >> 7 & 0x7F};
   if(write(fd, command, sizeof(command)) == -1){
       perror("error writing");
       return -1;
   }
	return 0;
}


//sets acceleration of motor on desired maestro channel
// ramp>>7 = lower byte, else ramp = higher byte
// write (const char* s, streamsize n ); inserts the first no chars of the array pointed by s
int maestro::setAcceleration(unsigned char channel, unsigned short ramp){
	unsigned char command[] = {0x89, channel, ramp & 0x7F , ramp >> 7 & 0x7F};
	if (write(fd, command, sizeof(command))==-1){
		perror("error writing");
		return -1;
	}
	return 0;
}

maestro::maestro(){//idk why this exists
	printf("Initializing Maestro... \n");
}

maestro::~maestro(){
	close(fd);
	printf("Closed. \n");
}


//sets pwm cycle as specified on time and period in units of 1/48 us,
//both on-time and period encoded w/ 7 bits per byte, same as setTarget
//periods 1020 (47.1 kHz), 4080 (11.7 kHz), and 16320 (2.9 kHz) provide the best possible res with 100% and 0% duty cycle
/*int setPWM(int fd, unsigned short ontime, unsigned short period){
	unsigned char command[]={0x8A, ontime & 0x07F, ontime >> 7 & 0x07F, period & 0x07F, period >> 7 & 0x07F};
	if (write(fd, command, sizeof(command)) == -1){
		perror("error writing");
  	return -1;
 	}
 	return 0;
} */
