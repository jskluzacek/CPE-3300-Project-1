/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : console.c
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 02/27/24
 * @brief          : API to retrieve input from USART console
 *****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "console.h"
#include "network.h"

/**
 * console_help:
 * Provides user with detailed help in using the console
 * parameters: none
 * returns: none
 */
static void console_help() {
	printf("To send a message:\n");
	printf("send (src_adr) (message)\n");
	printf("*src_adr in DECIMAL form...\n");
	printf("To receive messages:\n");
	printf("receive");
}

/**
 * console_print_char:
 * Prints a given character to the console
 * parameters: character to be printed
 * returns: none
 */
void console_print_char(char ch) {
	printf("%c", ch);
}

/**
 * console_print_str:
 * Prints a given string to the console
 * parameters: string to be printed
 * returns: none
 */
void console_print_str(const char str[]) {
	printf("%s\n", str);
}

/**
 * console_scan:
 * Retrieves user input and executes specified command
 * parameters: none
 * returns: none
 */
void console_scan(void) {
	char command[COMMAND_LEN];
	unsigned char message[MESSAGE_LEN + 1];
	char dest_addr[3 + 1];
	int ret_val;
	char userinput[COMMAND_LEN + MESSAGE_LEN + 1];

	printf("> ");

	fgets(userinput, COMMAND_LEN + MESSAGE_LEN - 1 + 1, stdin);

	ret_val = sscanf(userinput, "%s %s %s", command, dest_addr, message);

	if (!strcmp(command, "help")) {
		console_help();
	}
	else if (!strcmp(command, "send")) {
		tx_message(message, atoi(dest_addr));
	}
	else if (!strcmp(command, "receive")) {
		rx_messages();
	}
	else {
		printf("Unsupported command: %s\n", command);
	}
}

