/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include <string.h>
#include <stdlib.h>

#include "stm32f4xx_ll_gpio.h"

#include "app.h"

#include "cmd_line_task.h"
#include "cmd_line.h"
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
struct _cmd_line_typedef
{
                uint16_t    buffer_size;
                char*       p_buffer;

    volatile    uint16_t    write_index;
    volatile    char        RX_char;
};
typedef struct _cmd_line_typedef cmd_line_typedef;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static bool is_warned_user = false;

static const char * ErrorCode[6] =
{
    "OK\n",
    "CMDLINE_BAD_CMD\n",
    "CMDLINE_TOO_MANY_ARGS\n",
    "CMDLINE_TOO_FEW_ARGS\n",
    "CMDLINE_INVALID_ARG\n",
    "CMDLINE_INVALID_CMD\n",
};

/*
const char SPLASH[][65] = 
{
{"\r\n"},
{".........................................................\r\n"},
{".........................................................\r\n"},
{"..    ____                       _     _               ..\r\n"},
{"..   / ___| _ __   __ _  ___ ___| |   (_)_ __  _ __    ..\r\n"},
{"..   \\___ \\| '_ \\ / _` |/ __/ _ \\ |   | | '_ \\| '_ \\   ..\r\n"},
{"..    ___) | |_) | (_| | (_|  __/ |___| | | | | | | |  ..\r\n"},
{"..   |____/| .__/ \\__,_|\\___\\___|_____|_|_| |_|_| |_|  ..\r\n"},
{"..         |_|    _   _ _____ _____                    ..\r\n"},
{"..               | | | | ____| ____|                   ..\r\n"},
{"..               | |_| |  _| |  _|                     ..\r\n"},
{"..               |  _  | |___| |___                    ..\r\n"},
{"..               |_| |_|_____|_____|                   ..\r\n"},
{"..            __     _____   ___   ___                 ..\r\n"},
{"..            \\ \\   / / _ \\ / _ \\ / _ \\                ..\r\n"},
{"..             \\ \\ / / | | | | | | | | |               ..\r\n"},
{"..              \\ V /| |_| | |_| | |_| |               ..\r\n"},
{"..               \\_/  \\___(_)___(_)___/                ..\r\n"},
{".........................................................\r\n"},
{".........................................................\r\n"},                                                   
};
*/

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
//static void         CMD_send_splash(uart_stdio_typedef* p_uart);
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
uart_stdio_typedef  RS232_UART;
char                g_RS232_UART_TX_buffer[512];
char                g_RS232_UART_RX_buffer[64];

cmd_line_typedef    CMD_line;
char                g_CMD_line_buffer[64];
uint8_t             g_CMD_line_return = CMDLINE_OK;
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: CMD Line Task Init :::::::: */
void CMD_Line_Task_Init()
{
    UART_Init(  &RS232_UART, RS232_UART_HANDLE, RS232_UART_IRQ,
                g_RS232_UART_TX_buffer, g_RS232_UART_RX_buffer,
                sizeof(g_RS232_UART_TX_buffer), sizeof(g_RS232_UART_RX_buffer));
    
    CMD_line.p_buffer       = g_CMD_line_buffer;
    CMD_line.buffer_size    = 64;
    CMD_line.write_index 	= 0;

    if(CMD_line.buffer_size != 0)
    {
        memset((void *)CMD_line.p_buffer, 0, sizeof(CMD_line.p_buffer));
    }

    UART_Send_String(&RS232_UART, "GPP FIRMWARE V1.0.0\n");
    UART_Send_String(&RS232_UART, "> ");
    //CMD_send_splash(&RS232_UART);
}

/* :::::::::: CMD Line Task ::::::::::::: */
void CMD_Line_Task(void*)
{
    uint8_t time_out;

    for(time_out = 50; (!RX_BUFFER_EMPTY(&RS232_UART)) && (time_out != 0); time_out--)
    {
        CMD_line.RX_char = UART_Get_Char(&RS232_UART);

        if (g_CMD_line_return == CMDLINE_IS_PROCESSING)
        {
            UART_Send_String(&RS232_UART, "COMMAND IS BEING PROCESSED, PLEASE WAIT\n");
            UART_Send_String(&RS232_UART, "> ");
            is_warned_user = true;
        }
        
        if(((CMD_line.RX_char == 8) || (CMD_line.RX_char == 127)))
        {
            if (CMD_line.write_index == 0)
                break;

            CMD_line.write_index--;
            UART_Send_Char(&RS232_UART, CMD_line.RX_char);
            break;
        }

        UART_Send_Char(&RS232_UART, CMD_line.RX_char);

        if((CMD_line.RX_char == '\r') || (CMD_line.RX_char == '\n'))
        {
            if(CMD_line.write_index > 0)
            {
                // Add a NUL char at the end of the CMD
                CMD_line.p_buffer[CMD_line.write_index] = 0;

                g_CMD_line_return = CmdLineProcess(CMD_line.p_buffer);

                if (g_CMD_line_return == CMDLINE_IS_PROCESSING)
                {
                    return;
                }

                CMD_line.write_index = 0;

                UART_Send_String(&RS232_UART, "> ");
                UART_Printf(&RS232_UART, ErrorCode[g_CMD_line_return]);
                UART_Send_String(&RS232_UART, "> ");
            }
            else
            {
                UART_Send_String(&RS232_UART, "> ");
            }
        }
        else
        {
            CMD_line.p_buffer[CMD_line.write_index] = CMD_line.RX_char;
            CMD_line.write_index++;

            if (CMD_line.write_index > CMD_line.buffer_size)
            {
                // SDKLFJSDFKS
                // > CMD too long!
                // > 
                UART_Send_String(&RS232_UART, "\n> CMD too long!\n> ");
                //CMD_line.write_index = CMD_line.read_index;
                CMD_line.write_index    = 0;
            }
        }
    }

    if (g_CMD_line_return == CMDLINE_IS_PROCESSING)
    {
        g_CMD_line_return = CmdLineProcess(CMD_line.p_buffer);

        if (g_CMD_line_return == CMDLINE_IS_PROCESSING)
        {
            return;
        }

        CMD_line.write_index = 0;

        UART_Send_String(&RS232_UART, "> ");
        UART_Printf(&RS232_UART, ErrorCode[g_CMD_line_return]);
        UART_Send_String(&RS232_UART, "> ");

        if (is_warned_user == true)
        {
            UART_Send_String(&RS232_UART, "FINISH PROCESS COMMAND, PLEASE CONTINUE\n");
            UART_Send_String(&RS232_UART, "> ");
            is_warned_user = false;
        }
        
    }
}

/* :::::::::: IRQ Handler ::::::::::::: */
void RS232_IRQHandler(void)
{
    if(LL_USART_IsActiveFlag_TXE(RS232_UART.handle) == true)
    {
        if(TX_BUFFER_EMPTY(&RS232_UART))
        {
            // Buffer empty, so disable interrupts
            LL_USART_DisableIT_TXE(RS232_UART.handle);
        }
        else
        {
            // There is more data in the output buffer. Send the next byte
            UART_Prime_Transmit(&RS232_UART);
        }
    }

    if(LL_USART_IsActiveFlag_RXNE(RS232_UART.handle) == true)
    {
        RS232_UART.RX_irq_char = LL_USART_ReceiveData8(RS232_UART.handle);

        // NOTE: On win 10, default PUTTY when hit enter only send back '\r',
        // while on default HERCULES when hit enter send '\r\n' in that order.
        // The code bellow is modified so that it can work on PUTTY and HERCULES.
        if((!RX_BUFFER_FULL(&RS232_UART)) && (RS232_UART.RX_irq_char != '\n'))
        {
            if (RS232_UART.RX_irq_char == '\r')
            {
                RS232_UART.p_RX_buffer[RS232_UART.RX_write_index] = '\n';
                ADVANCE_RX_WRITE_INDEX(&RS232_UART);
            }
            else
            {
                RS232_UART.p_RX_buffer[RS232_UART.RX_write_index] = RS232_UART.RX_irq_char;
                ADVANCE_RX_WRITE_INDEX(&RS232_UART);
            }
        }
    }
}

/*
static void CMD_send_splash(uart_stdio_typedef* p_uart)
{
    for(uint8_t i = 0 ; i < 21 ; i++)
    {
		UART_Send_String(p_uart, &SPLASH[i][0]);
	}
	UART_Send_String(p_uart, "> ");
}
*/

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
