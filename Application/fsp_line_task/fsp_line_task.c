/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Include~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
#include "fsp_line_task.h"
#include <string.h>
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Enum ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Struct ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Class ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~Private Types ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
struct _fsp_line_typedef
{
				uint16_t 	buffer_size;
				char 		*p_buffer;

	volatile 	uint16_t 	write_index;
	volatile 	char 		RX_char;
};
typedef struct _fsp_line_typedef fsp_line_typedef;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
static const char * ErrorCode[7] =
{
    "FSP_PKT_RECEIVED_OK\n",
    "FSP_PKT_NOT_READY\n",
    "FSP_PKT_INVALID\n",
    "FSP_PKT_WRONG_ADR\n",
    "FSP_PKT_ERROR\n",
    "FSP_PKT_CRC_FAIL\n",
    "FSP_PKT_WRONG_LENGTH\n"
};

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
uart_stdio_typedef 	GPC_UART;
char 				g_GPC_UART_TX_buffer[GPP_TX_SIZE];
char 				g_GPC_UART_RX_buffer[GPP_RX_SIZE];

fsp_packet_t 		s_FSP_TX_Packet;
fsp_packet_t 		s_FSP_RX_Packet;
FSP_Payload 		*ps_FSP_TX = (FSP_Payload*) (&s_FSP_TX_Packet.payload);
FSP_Payload 		*ps_FSP_RX = (FSP_Payload*) (&s_FSP_RX_Packet.payload);

fsp_line_typedef 	FSP_line;
char 				g_FSP_line_buffer[FSP_BUF_LEN];

uint8_t				g_FSP_line_return = 1;

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Public Function ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* :::::::::: FSP Line Task Init :::::::: */
void FSP_Line_Task_Init()
{
	UART_Init(	&GPC_UART, GPC_UART_HANDLE, GPC_UART_IRQ, g_GPC_UART_TX_buffer,
				g_GPC_UART_RX_buffer, GPP_TX_SIZE, GPP_RX_SIZE);

	FSP_line.p_buffer 		= g_FSP_line_buffer;
	FSP_line.buffer_size 	= FSP_BUF_LEN;
	FSP_line.write_index 	= 0;

	if (FSP_line.buffer_size != 0)
	{
		memset((void*) FSP_line.p_buffer, 0, sizeof(FSP_line.p_buffer));
	}

	fsp_init(FSP_ADR_GPP);
}

/* :::::::::: FSP Line Task ::::::::::::: */
void FSP_Line_Task(void*)
{
	uint8_t FSP_return, time_out = 0;

	for (time_out = 50; (!RX_BUFFER_EMPTY(&GPC_UART)) && (time_out != 0); time_out--)
	{
		if (g_FSP_line_return == 0)
		{
			break;
		}
		
		FSP_line.RX_char = UART_Get_Char(&GPC_UART);

		if (FSP_line.RX_char == FSP_PKT_SOD)
		{
			FSP_line.write_index = 0;
		}
		else if (FSP_line.RX_char == FSP_PKT_EOF)
		{
			FSP_return = frame_decode((uint8_t*) FSP_line.p_buffer, FSP_line.write_index, &s_FSP_RX_Packet);

			UART_Printf(&RS232_UART, "%s> ", ErrorCode[FSP_return]);

			if (FSP_return == FSP_PKT_READY)
			{
				g_FSP_line_return = FSP_Line_Process();
			}

			if (g_FSP_line_return == 0)
			{
				return;
			}
			
			FSP_line.write_index = 0;
		} 
		else 
		{
			FSP_line.p_buffer[FSP_line.write_index] = FSP_line.RX_char;
			FSP_line.write_index++;

			if (FSP_line.write_index > FSP_line.buffer_size)
				FSP_line.write_index = 0;

		}
	}

	if (g_FSP_line_return == 0)
    {
        g_FSP_line_return = FSP_Line_Process();

        if (g_FSP_line_return == 0)
        {
            return;
        }

        FSP_line.write_index = 0;
    }
}

/* :::::::::: IRQ Handler ::::::::::::: */
void GPC_UART_IRQHandler(void) 
{
	if (LL_USART_IsActiveFlag_TXE(GPC_UART.handle) == true)
	{
		if (TX_BUFFER_EMPTY(&GPC_UART))
		{
			// Buffer empty, so disable interrupts
			LL_USART_DisableIT_TXE(GPC_UART.handle);
		}
		else
		{
			// There is more data in the output buffer. Send the next byte
			UART_Prime_Transmit(&GPC_UART);
		}
	}

	if (LL_USART_IsActiveFlag_RXNE(GPC_UART.handle) == true)
	{
		GPC_UART.RX_irq_char = LL_USART_ReceiveData8(GPC_UART.handle);

		if (!RX_BUFFER_FULL(&GPC_UART))
		{
			GPC_UART.p_RX_buffer[GPC_UART.RX_write_index] = GPC_UART.RX_irq_char;
			ADVANCE_RX_WRITE_INDEX(&GPC_UART);
		}
	}
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Private Prototype ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ End of the program ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
