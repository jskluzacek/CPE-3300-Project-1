/******************************************************************************
 * @assignment	   : CPE3300 Project 1
 * @file           : console.h
 * @author         : Team Edward - Kenny Gifford and Jerico Skluzacek
 * @date		   : 02/04/24
 * @brief          : Header for console API
 *****************************************************************************/

#ifndef _console_h
#define _console_h

// Constants
#define MESSAGE_LEN 255
#define COMMAND_LEN	50

// prototypes
void console_scan(void);
void console_print_char(char);
void console_print_str(const char str[]);

#endif // console.h
