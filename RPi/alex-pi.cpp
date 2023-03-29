#include <stdio.h>
#include <pthread.h>
#include <semaphore.h>
#include <unistd.h>
#include <stdint.h>
#include "packet.h"
#include "serial.h"
#include "serialize.h"
#include "constants.h"
#include <termios.h>

#define PORT_NAME			"/dev/ttyACM0"
#define BAUD_RATE			B9600

// Initialise current states
int current_speed = 100;
int current_angle = 90;

int exitFlag=0;
sem_t _xmitSema;

static struct termios old, new1;

void initTermios(int echo) {
	tcgetattr(0, &old);
	new1 = old;
	new1.c_lflag &= ~ICANON;
	new1.c_lflag &= echo ? ECHO : ~ECHO;
	tcsetattr(0, TCSANOW, &new1);
}

void resetTermios(void) {
	tcsetattr(0, TCSANOW, &old); // restore old settings

}

void handleError(TResult error)
{
	switch(error)
	{
		case PACKET_BAD:
			printf("ERROR: Bad Magic Number\n");
			break;

		case PACKET_CHECKSUM_BAD:
			printf("ERROR: Bad checksum\n");
			break;

		default:
			printf("ERROR: UNKNOWN ERROR\n");
	}
}

void handleStatus(TPacket *packet)
{
	printf("\n ------- ALEX STATUS REPORT ------- \n\n");
	printf("Left Forward Ticks:\t\t%d\n", packet->params[0]);
	printf("Right Forward Ticks:\t\t%d\n", packet->params[1]);
	printf("Left Reverse Ticks:\t\t%d\n", packet->params[2]);
	printf("Right Reverse Ticks:\t\t%d\n", packet->params[3]);
	printf("Left Forward Ticks Turns:\t%d\n", packet->params[4]);
	printf("Right Forward Ticks Turns:\t%d\n", packet->params[5]);
	printf("Left Reverse Ticks Turns:\t%d\n", packet->params[6]);
	printf("Right Reverse Ticks Turns:\t%d\n", packet->params[7]);
	printf("Forward Distance:\t\t%d\n", packet->params[8]);
	printf("Reverse Distance:\t\t%d\n", packet->params[9]);
	printf("\n---------------------------------------\n\n");
}

void handleResponse(TPacket *packet)
{
	// The response code is stored in command
	switch(packet->command)
	{
		case RESP_OK:
			printf("Command OK\n");
		break;

		case RESP_STATUS:
			handleStatus(packet);
		break;

		default:
			printf("Arduino is confused\n");
	}
}

void handleErrorResponse(TPacket *packet)
{
	// The error code is returned in command
	switch(packet->command)
	{
		case RESP_BAD_PACKET:
			printf("Arduino received bad magic number\n");
		break;

		case RESP_BAD_CHECKSUM:
			printf("Arduino received bad checksum\n");
		break;

		case RESP_BAD_COMMAND:
			printf("Arduino received bad command\n");
		break;

		case RESP_BAD_RESPONSE:
			printf("Arduino received unexpected response\n");
		break;

		default:
			printf("Arduino reports a weird error\n");
	}
}

void handleMessage(TPacket *packet)
{
	printf("Message from Alex: %s\n", packet->data);
}

void handlePacket(TPacket *packet)
{
	switch(packet->packetType)
	{
		case PACKET_TYPE_COMMAND:
				// Only we send command packets, so ignore
			break;

		case PACKET_TYPE_RESPONSE:
				handleResponse(packet);
			break;

		case PACKET_TYPE_ERROR:
				handleErrorResponse(packet);
			break;

		case PACKET_TYPE_MESSAGE:
				handleMessage(packet);
			break;
	}
}

void sendPacket(TPacket *packet)
{
	char buffer[PACKET_SIZE];
	int len = serialize(buffer, packet, sizeof(TPacket));

	serialWrite(buffer, len);
}

void *receiveThread(void *p)
{
	char buffer[PACKET_SIZE];
	int len;
	TPacket packet;
	TResult result;
	int counter=0;

	while(1)
	{
		len = serialRead(buffer);
		counter+=len;
		if(len > 0)
		{
			result = deserialize(buffer, len, &packet);

			if(result == PACKET_OK)
			{
				counter=0;
				handlePacket(&packet);
			}
			else 
				if(result != PACKET_INCOMPLETE)
				{
					printf("PACKET ERROR\n");
					handleError(result);
				}
		}
	}
}

void getParams(TPacket *commandPacket, int* command_parameter)
{
	// printf("Enter distance/angle in cm/degrees (e.g. 50) and power in %% (e.g. 75) separated by space.\n");
	// printf("E.g. 50 75 means go at 50 cm at 75%% power for forward/backward, or 50 degrees left or right turn at 75%%  power\n");
	commandPacket->params[0] = *command_parameter;
	commandPacket->params[1] = 80;
}

void sendCommand(char command)
{
	switch(command)
	{
		case '1':
			current_speed = 100;
			return;
		
		case '2':
			current_speed = 80;
			return;

		case '3':
			current_speed = 60;
			return;

		case '4':
			current_speed = 40;
			return;

		default:
			break;
	}
	
	TPacket commandPacket;

	commandPacket.packetType = PACKET_TYPE_COMMAND;

	switch(command)
	{
		// movement commands
		case 'w':
			//getParams(&commandPacket, curr_speed);
			commandPacket.command = COMMAND_FORWARD;
			command.params[0] = current_speed;
			sendPacket(&commandPacket);
			break;

		case 's':
			// getParams(&commandPacket, curr_speed);
			commandPacket.command = COMMAND_REVERSE;
			command.params[0] = current_speed;
			sendPacket(&commandPacket);
			break;

		// rotation commands
		case 'a':
			// getParams(&commandPacket, curr_angle);
			commandPacket.command = COMMAND_TURN_LEFT;
			commandPacket.params[0] = 90;
			commandPacket.params[1] = 90;
			sendPacket(&commandPacket);
			break;

		case 'd':
			// getParams(&commandPacket, curr_angle);
			commandPacket.command = COMMAND_TURN_RIGHT;
			commandPacket.params[0] = 90;
			commandPacket.params[1] = 90;
			sendPacket(&commandPacket);
			break;

		case 'q':
			// getParams(&commandPacket, curr_angle);
			commandPacket.command = COMMAND_TURN_LEFT;
			commandPacket.params[0] = 60;
			commandPacket.params[1] = 95;
			sendPacket(&commandPacket);
			break;

		case 'e':
			// getParams(&commandPacket, curr_angle);
			commandPacket.command = COMMAND_TURN_RIGHT;
			commandPacket.params[0] = 60;
			commandPacket.params[1] = 95;
			sendPacket(&commandPacket);
			break;

		case 'z':
			// getParams(&commandPacket, curr_angle);
			commandPacket.command = COMMAND_TURN_LEFT;
			commandPacket.params[0] = 30;
			commandPacket.params[1] = 100;
			sendPacket(&commandPacket);
			break;

		case 'c':
			// getParams(&commandPacket, curr_angle);
			commandPacket.command = COMMAND_TURN_RIGHT;
			commandPacket.params[0] = 30;
			commandPacket.params[1] = 100;
			sendPacket(&commandPacket);
			break;

		// misc commands
		case 32:
			commandPacket.command = COMMAND_STOP;
			sendPacket(&commandPacket);
			break;

		case 'C':
			commandPacket.command = COMMAND_CLEAR_STATS;
			commandPacket.params[0] = 0;
			sendPacket(&commandPacket);
			break;

		case 'g':
		case 'G':
			commandPacket.command = COMMAND_GET_STATS;
			sendPacket(&commandPacket);
			break;

		case 27:
			exitFlag=1;
			break;

		default:
			printf("Bad command\n");

	}
}

int main()
{
	// Connect to the Arduino
	startSerial(PORT_NAME, BAUD_RATE, 8, 'N', 1, 5);

	// Sleep for two seconds
	printf("WAITING TWO SECONDS FOR ARDUINO TO REBOOT\n");
	sleep(2);
	printf("DONE\n");

	// Spawn receiver thread
	pthread_t recv;

	pthread_create(&recv, NULL, receiveThread, NULL);

	// Send a hello packet
	TPacket helloPacket;

	helloPacket.packetType = PACKET_TYPE_HELLO;
	sendPacket(&helloPacket);

	printf("Command (w=forward, s=reverse, a=turn left 90deg, d=turn right 90deg, space=stop, esc=quit, ignore the rest: ...c=clear stats, g=get stats)\n");

	

	while(!exitFlag)
	{
		
		char ch;

		// make terminal auto submit
		initTermios(0);
		read(0, &ch, 1);		

		sendCommand(ch);
		resetTermios();
	}

	printf("Closing connection to Arduino.\n");
	endSerial();
}
