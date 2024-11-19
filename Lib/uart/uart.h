#ifndef UARTSTDIO_H
#define UARTSTDIO_H

#include "stm32f4xx_ll_usart.h"

//#include "app.h"

typedef struct _uart_stdio_typedef
{
    USART_TypeDef*          handle;
    IRQn_Type		        irqn;

    uint16_t                TX_size;
    uint16_t                RX_size;

                char*       p_TX_buffer;
    volatile    uint16_t    TX_write_index;
    volatile    uint16_t    TX_read_index;

                char*       p_RX_buffer;
    volatile    uint16_t    RX_write_index;
    volatile    uint16_t    RX_read_index;
    volatile    char        RX_irq_char;
}uart_stdio_typedef;

void        UART_Init( uart_stdio_typedef* p_uart, USART_TypeDef* _handle,
                        IRQn_Type _irqn, char* _p_TX_buffer, char* _p_RX_buffer,
                        uint16_t _TX_size, uint16_t _RX_size);

void        UART_Send_Char(uart_stdio_typedef* p_uart, const char Char);
void        UART_Send_String(uart_stdio_typedef* p_uart, const char *pcBuf);
uint16_t    UART_Write(uart_stdio_typedef* p_uart, const char *pcBuf, uint16_t ui16Len);
void        UART_Printf(uart_stdio_typedef* p_uart, const char *pc_string, ...);
uint16_t    UART_FSP(uart_stdio_typedef* p_uart, const char *pcBuf, uint16_t ui16Len);
char        UART_Get_Char(uart_stdio_typedef* p_uart);

uint8_t     UART_is_buffer_full(volatile uint16_t *pui16Read,
                volatile uint16_t *pui16Write, uint16_t ui16Size);

uint8_t     UART_is_buffer_empty(volatile uint16_t *pui16Read,
                volatile uint16_t *pui16Write);

uint16_t    UART_get_buffer_count(volatile uint16_t *pui16Read,
                volatile uint16_t *pui16Write, uint16_t ui16Size);

uint16_t    UART_advance_buffer_index(volatile uint16_t* pui16Index, uint16_t ui16Size);

void        UART_Prime_Transmit(uart_stdio_typedef* p_uart);

//*****************************************************************************
//
// Macros to determine number of free and used bytes in the transmit buffer.
//
//*****************************************************************************
#define TX_BUFFER_SIZE(p_uart)          ((p_uart)->TX_size)

#define TX_BUFFER_USED(p_uart)          (UART_get_buffer_count(&(p_uart)->TX_read_index,  \
                                                        &(p_uart)->TX_write_index, \
                                                        (p_uart)->TX_size))

#define TX_BUFFER_FREE(p_uart)          (TX_BUFFER_SIZE(p_uart) - TX_BUFFER_USED(p_uart))

#define TX_BUFFER_EMPTY(p_uart)         (UART_is_buffer_empty(&(p_uart)->TX_read_index,   \
                                                       &(p_uart)->TX_write_index))

#define TX_BUFFER_FULL(p_uart)          (UART_is_buffer_full(&(p_uart)->TX_read_index,  \
                                                      &(p_uart)->TX_write_index, \
                                                      (p_uart)->TX_size))

#define ADVANCE_TX_WRITE_INDEX(p_uart)  (UART_advance_buffer_index(&(p_uart)->TX_write_index, \
                                                            (p_uart)->TX_size))

#define ADVANCE_TX_READ_INDEX(p_uart)   (UART_advance_buffer_index(&(p_uart)->TX_read_index, \
                                                            (p_uart)->TX_size))

//*****************************************************************************
//
// Macros to determine number of free and used bytes in the receive buffer.
//
//*****************************************************************************
#define RX_BUFFER_SIZE(p_uart)          ((p_uart)->RX_size)

#define RX_BUFFER_USED(p_uart)          (UART_get_buffer_count(&(p_uart)->RX_read_index,  \
                                                        &(p_uart)->RX_write_index, \
                                                        (p_uart)->RX_size))

#define RX_BUFFER_FREE(p_uart)          (RX_BUFFER_SIZE(p_uart) - RX_BUFFER_USED(p_uart))

#define RX_BUFFER_EMPTY(p_uart)         (UART_is_buffer_empty(&(p_uart)->RX_read_index,   \
                                                       &(p_uart)->RX_write_index))

#define RX_BUFFER_FULL(p_uart)          (UART_is_buffer_full(&(p_uart)->RX_read_index,  \
                                                      &(p_uart)->RX_write_index, \
                                                      (p_uart)->RX_size))

#define ADVANCE_RX_WRITE_INDEX(p_uart)  (UART_advance_buffer_index(&(p_uart)->RX_write_index, \
                                                            (p_uart)->RX_size))

#define ADVANCE_RX_READ_INDEX(p_uart)   (UART_advance_buffer_index(&(p_uart)->RX_read_index, \
                                                            (p_uart)->RX_size))

#endif // UARTSTDIO_H
