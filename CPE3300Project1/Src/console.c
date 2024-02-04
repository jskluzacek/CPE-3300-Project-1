/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : console.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 02/04/24
 * @brief          : Methods to retrieve input from console
 *****************************************************************************/

#include <stdio.h>
#include <string.h>
#include "console.h"

/**
 * console_help:
 * Provides user with detailed help in using the console
 * parameters: none
 * returns: none
 */
static void console_help() {
	printf("To use Read Memory Word:\n");
	printf("Type: rmw (address)\n");
	printf("To use Write Memory Word:\n");
	printf("Type: wmw (address) (value)\n");
	printf("To use Dump Memory:\n");
	printf("Type: dm (address) (number of bytes)\n");
}

/**
 * console_scan:
 * Retrieves user input and executes specified command
 * parameters: none
 * returns: none
 */
void console_scan(void) {
	char command[50];
	int arg2;
	int arg1;
	int ret_val;
	char userinput[100];

	printf("> ");

	fgets(userinput, 99, stdin);

	ret_val = sscanf(userinput, "%s %i %i", command, &arg1, &arg2);

	if (!strcmp(command, "help")) {
		console_help();
	}
	else if (!strcmp(command, "send")) {
		// TODO: send arg1
	}
	else {
		printf("Unsupported command: %s\n", command);
	}
}

