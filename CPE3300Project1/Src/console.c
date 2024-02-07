/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : console.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 02/04/24
 * @brief          : API to retrieve input from USART console
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "console.h"
#include "network.h"

/**
 * console_help:
 * Provides user with detailed help in using the console
 * parameters: none
 * returns: none
 */
static void console_help() {
	printf("To send a sequence of characters:\n");
	printf("Send (message to be sent)\n");
}

/**
 * console_scan:
 * Retrieves user input and executes specified command
 * parameters: none
 * returns: none
 */
void console_scan(void) {
	char command[50];
	unsigned char message[100];
	int ret_val;
	char userinput[150];

	printf("> ");

	fgets(userinput, 149, stdin);

	ret_val = sscanf(userinput, "%s %s", command, message);

	if (!strcmp(command, "help")) {
		console_help();
	}
	else if (!strcmp(command, "send")) {
		// TODO: send message
		tx_string(message);
	}
	else {
		printf("Unsupported command: %s\n", command);
	}
}

