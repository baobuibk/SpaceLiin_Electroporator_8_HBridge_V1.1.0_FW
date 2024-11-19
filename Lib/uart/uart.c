/*********************
 *      INCLUDES
 *********************/
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>

#include "stm32f405xx.h"

#include "uart.h"

//*****************************************************************************
//
// A mapping from an integer between 0 and 15 to its ASCII character
// equivalent.
//
//*****************************************************************************
static const char * const HEX_reference = "0123456789abcdef";

/*********************
 *   GLOBAL FUNCTION
 *********************/

/**
 * The function `UARTConfig` initializes a UART configuration with a specified receive buffer size and
 * sets up interrupt-driven reception.
 * 
 * @param uart_p The `uart_p` parameter is a pointer to a structure of type `UART_HandleTypeDef`, which
 * contains configuration settings for a UART (Universal Asynchronous Receiver-Transmitter) peripheral.
 * @param rx_buffer_size The `rx_buffer_size` parameter specifies the size of the receive buffer used
 * for storing incoming data in the UARTConfig function. This buffer will be initialized using the
 * QUEUE_Init function to manage incoming data efficiently.
 */
void UART_Init( uart_stdio_typedef* p_uart, USART_TypeDef* _handle,
                IRQn_Type _irqn, char* _p_TX_buffer, char* _p_RX_buffer,
                uint16_t _TX_size, uint16_t _RX_size)
{
    p_uart->handle  = _handle;
    p_uart->irqn    = _irqn;
    p_uart->TX_size = _TX_size;
    p_uart->RX_size = _RX_size;

    p_uart->p_TX_buffer = _p_TX_buffer;
    p_uart->p_RX_buffer = _p_RX_buffer;

    p_uart->TX_write_index  = 0;
    p_uart->TX_read_index   = 0;
    p_uart->RX_write_index  = 0;
    p_uart->RX_read_index   = 0;

    if(TX_BUFFER_SIZE(p_uart) != 0)
    {
        //p_uart->p_TX_buffer = (uint8_t *)malloc(TX_BUFFER_SIZE(p_uart) * sizeof(uint8_t));
        memset((void *)p_uart->p_TX_buffer, 0, sizeof(p_uart->p_TX_buffer));
    }

    if(RX_BUFFER_SIZE(p_uart) != 0)
    {
        //p_uart->p_RX_buffer = (uint8_t *)malloc(RX_BUFFER_SIZE(p_uart) * sizeof(uint8_t));
        memset((void *)p_uart->p_RX_buffer, 0, sizeof(p_uart->p_RX_buffer));
    }

    // Disable the TX IT
    LL_USART_DisableIT_TXE(p_uart->handle);

    // Start the RX IT, it will only run when there is a data send to the MCU.
    LL_USART_EnableIT_RXNE(p_uart->handle);
}

//*****************************************************************************
//
//! Send a char to the UART.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//
//*****************************************************************************
void UART_Send_Char(uart_stdio_typedef* p_uart, const char Char)
{
	UART_Write(p_uart, &Char, 1);
}

//*****************************************************************************
//
//! Send the string to the UART.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//
//*****************************************************************************
void UART_Send_String(uart_stdio_typedef* p_uart, const char *pcBuf)
{
	UART_Write(p_uart, pcBuf, strlen(pcBuf));
}

//*****************************************************************************
//
//! Writes a string of characters to the UART output.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//! \param ui16Len is the length of the string to transmit.
//!
//! This function will transmit the string to the UART output.  The number of
//! characters transmitted is determined by the \e ui16Len parameter.  This
//! function does no interpretation or translation of any characters.  Since
//! the output is sent to a UART, any LF (/n) characters encountered will be
//! replaced with a CRLF pair.
//!
//! Besides using the \e ui16Len parameter to stop transmitting the string, if
//! a null character (0) is encountered, then no more characters will be
//! transmitted and the function will return.
//!
//! In non-buffered mode, this function is blocking and will not return until
//! all the characters have been written to the output FIFO.  In buffered mode,
//! the characters are written to the UART transmit buffer and the call returns
//! immediately.  If insufficient space remains in the transmit buffer,
//! additional characters are discarded.
//!
//! \return Returns the count of characters written.
//
//*****************************************************************************
uint16_t UART_Write(uart_stdio_typedef* p_uart, const char *pcBuf, uint16_t ui16Len)
{

    uint8_t uIdx;

    //
    // Check for valid arguments.
    //
    //
    // Send the characters
    //
    for(uIdx = 0; uIdx < ui16Len; uIdx++)
    {
        //
        // If the character to the UART is \n, then add a \r before it so that
        // \n is translated to \n\r in the output. This is for the enter key.
        //
        if(pcBuf[uIdx] == '\n')
        {
            if(!TX_BUFFER_FULL(p_uart))
            {
                p_uart->p_TX_buffer[p_uart->TX_write_index] = '\r';
                ADVANCE_TX_WRITE_INDEX(p_uart);
            }
            else
            {
                //
                // Buffer is full - discard remaining characters and return.
                //
                break;
            }
        }

        if(!TX_BUFFER_FULL(p_uart))
        {
        	p_uart->p_TX_buffer[p_uart->TX_write_index] = pcBuf[uIdx];
            ADVANCE_TX_WRITE_INDEX(p_uart);
        }
        else
        {
            //
            // Buffer is full - discard remaining characters and return.
            //
            p_uart->p_TX_buffer[p_uart->TX_write_index] = '\r';
            break;
        }
    }

    //
    // If the usart txe irq is disable, this mean an usart phase is finished
    // we need to enable the txe irq and kick start the transmit process.
    //
    if (LL_USART_IsEnabledIT_TXE(p_uart->handle) == false)
    {
        // NOTE: Turn on TXE after prime transmit,
        // if turn on TXE b4 prime transmit create a
        // bug where the index = 2 char don't get
        // send.

        //LL_USART_EnableIT_TXE(p_uart->handle);
        UART_Prime_Transmit(p_uart);
        LL_USART_EnableIT_TXE(p_uart->handle);
    }

    //
    // Return the number of characters written.
    //
    return(uIdx);
}

//*****************************************************************************
//
//! A simple UART based vprintf function supporting \%c, \%d, \%p, \%s, \%u,
//! \%x, and \%X.
//!
//! \param pc_string is the format string.
//! \param vaArgP is a variable argument list pointer whose content will depend
//! upon the format string passed in \e pc_string.
//!
//! This function is very similar to the C library <tt>vprintf()</tt> function.
//! All of its output will be sent to the UART.  Only the following formatting
//! characters are supported:
//!
//! - \%c to print a character
//! - \%d or \%i to print a decimal value
//! - \%s to print a string
//! - \%u to print an unsigned decimal value
//! - \%x to print a hexadecimal value using lower case letters
//! - \%X to print a hexadecimal value using lower case letters (not upper case
//! letters as would typically be used)
//! - \%p to print a pointer as a hexadecimal value
//! - \%\% to print out a \% character
//!
//! For \%s, \%d, \%i, \%u, \%p, \%x, and \%X, an optional number may reside
//! between the \% and the format character, which specifies the minimum number
//! of characters to use for that value; if preceded by a 0 then the extra
//! characters will be filled with zeros instead of spaces.  For example,
//! ``\%8d'' will use eight characters to print the decimal value with spaces
//! added to reach eight; ``\%08d'' will use eight characters as well but will
//! add zeroes instead of spaces.
//!
//! The type of the arguments in the variable arguments list must match the
//! requirements of the format string.  For example, if an integer was passed
//! where a string was expected, an error of some kind will most likely occur.
//!
//! \return None.
//
//*****************************************************************************
void UART_Printf(uart_stdio_typedef* p_uart, const char *pc_string, ...)
{
    uint16_t ui16Idx, ui16Value, ui16Pos, ui16Count, ui16Base, ui16Neg;
    char *pcStr, pcBuf[16], cFill;

    //
    // Check the arguments.
    //

    va_list vaArgP;

    va_start(vaArgP, pc_string);

    //
    // Loop while there are more characters in the string.
    //
    while(*pc_string)
    {
        //
        // Find the first non-% character, or the end of the string.
        //
        for(ui16Idx = 0;
            (pc_string[ui16Idx] != '%') && (pc_string[ui16Idx] != '\0');
            ui16Idx++)
        {
        }

        //
        // Write this portion of the string.
        //
        UART_Write(p_uart, pc_string, ui16Idx);

        //
        // Skip the portion of the string that was written.
        //
        pc_string += ui16Idx;

        //
        // See if the next character is a %.
        //
        if(*pc_string == '%')
        {
            //
            // Skip the %.
            //
            pc_string++;

            //
            // Set the digit count to zero, and the fill character to space
            // (in other words, to the defaults).
            //
            ui16Count = 0;
            cFill = ' ';

            //
            // It may be necessary to get back here to process more characters.
            // Goto's aren't pretty, but effective.  I feel extremely dirty for
            // using not one but two of the beasts.
            //
again:

            //
            // Determine how to handle the next character.
            //
            switch(*pc_string++)
            {
                //
                // Handle the digit characters.
                //
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                {
                    //
                    // If this is a zero, and it is the first digit, then the
                    // fill character is a zero instead of a space.
                    //
                    if((pc_string[-1] == '0') && (ui16Count == 0))
                    {
                        cFill = '0';
                    }

                    //
                    // Update the digit count.
                    //
                    ui16Count *= 10;
                    ui16Count += pc_string[-1] - '0';

                    //
                    // Get the next character.
                    //
                    goto again;
                }

                //
                // Handle the %c command.
                //
                case 'c':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui16Value = va_arg(vaArgP, int);

                    //
                    // Print out the character.
                    //
                    UART_Write(p_uart, (char *)&ui16Value, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %d and %i commands.
                //
                case 'd':
                case 'i':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui16Value = va_arg(vaArgP, int);

                    //
                    // Reset the buffer position.
                    //
                    ui16Pos = 0;

                    //
                    // If the value is negative, make it positive and indicate
                    // that a minus sign is needed.
                    //
                    if((int16_t)ui16Value < 0)
                    {
                        //
                        // Make the value positive.
                        //
                        ui16Value = -(int16_t)ui16Value;

                        //
                        // Indicate that the value is negative.
                        //
                        ui16Neg = 1;
                    }
                    else
                    {
                        //
                        // Indicate that the value is positive so that a minus
                        // sign isn't inserted.
                        //
                        ui16Neg = 0;
                    }

                    //
                    // Set the base to 10.
                    //
                    ui16Base = 10;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %s command.
                //
                case 's':
                {
                    //
                    // Get the string pointer from the varargs.
                    //
                    pcStr = va_arg(vaArgP, char *);

                    //
                    // Determine the length of the string.
                    //
                    for(ui16Idx = 0; pcStr[ui16Idx] != '\0'; ui16Idx++)
                    {
                    }

                    //
                    // Write the string.
                    //
                    UART_Write(p_uart, pcStr, ui16Idx);

                    //
                    // Write any required padding spaces
                    //
                    if(ui16Count > ui16Idx)
                    {
                        ui16Count -= ui16Idx;
                        while(ui16Count--)
                        {
                            UART_Write(p_uart, " ", 1);
                        }
                    }

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %u command.
                //
                case 'u':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui16Value = va_arg(vaArgP, int);

                    //
                    // Reset the buffer position.
                    //
                    ui16Pos = 0;

                    //
                    // Set the base to 10.
                    //
                    ui16Base = 10;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ui16Neg = 0;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %x and %X commands.  Note that they are treated
                // identically; in other words, %X will use lower case letters
                // for a-f instead of the upper case letters it should use.  We
                // also alias %p to %x.
                //
                case 'x':
                case 'X':
                case 'p':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ui16Value = va_arg(vaArgP, int);

                    //
                    // Reset the buffer position.
                    //
                    ui16Pos = 0;

                    //
                    // Set the base to 16.
                    //
                    ui16Base = 16;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ui16Neg = 0;

                    //
                    // Determine the number of digits in the string version of
                    // the value.
                    //
convert:
                    for(ui16Idx = 1;
                        (((ui16Idx * ui16Base) <= ui16Value) &&
                         (((ui16Idx * ui16Base) / ui16Base) == ui16Idx));
                        ui16Idx *= ui16Base, ui16Count--)
                    {
                    }

                    //
                    // If the value is negative, reduce the count of padding
                    // characters needed.
                    //
                    if(ui16Neg)
                    {
                        ui16Count--;
                    }

                    //
                    // If the value is negative and the value is padded with
                    // zeros, then place the minus sign before the padding.
                    //
                    if(ui16Neg && (cFill == '0'))
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ui16Pos++] = '-';

                        //
                        // The minus sign has been placed, so turn off the
                        // negative flag.
                        //
                        ui16Neg = 0;
                    }

                    //
                    // Provide additional padding at the beginning of the
                    // string conversion if needed.
                    //
                    if((ui16Count > 1) && (ui16Count < 16))
                    {
                        for(ui16Count--; ui16Count; ui16Count--)
                        {
                            pcBuf[ui16Pos++] = cFill;
                        }
                    }

                    //
                    // If the value is negative, then place the minus sign
                    // before the number.
                    //
                    if(ui16Neg)
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ui16Pos++] = '-';
                    }

                    //
                    // Convert the value into a string.
                    //
                    for(; ui16Idx; ui16Idx /= ui16Base)
                    {
                        pcBuf[ui16Pos++] =
                            HEX_reference[(ui16Value / ui16Idx) % ui16Base];
                    }

                    //
                    // Write the string.
                    //
                    UART_Write(p_uart, pcBuf, ui16Pos);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %% command.
                //
                case '%':
                {
                    //
                    // Simply write a single %.
                    //
                    UART_Write(p_uart, pc_string - 1, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle all other commands.
                //
                default:
                {
                    //
                    // Indicate an error.
                    //
                    UART_Write(p_uart, "ERROR", 5);

                    //
                    // This command has been handled.
                    //
                    break;
                }
            }
        }
    }
    va_end(vaArgP);
}

//*****************************************************************************
//
//! Read a single character from the UART, blocking if necessary.
//!
//! This function will receive a single character from the UART and store it at
//! the supplied address.
//!
//! In both buffered and unbuffered modes, this function will block until a
//! character is received.  If non-blocking operation is required in buffered
//! mode, a call to UARTRxAvail() may be made to determine whether any
//! characters are currently available for reading.
//!
//! \return Returns the character read.
//
//*****************************************************************************
char UART_Get_Char(uart_stdio_typedef* p_uart)
{

    char return_char;

    if (RX_BUFFER_EMPTY(p_uart))
    {
        return 0;
    }
    
    //
    // Read a character from the buffer.
    //
    return_char = p_uart->p_RX_buffer[p_uart->RX_read_index];
    ADVANCE_RX_READ_INDEX(p_uart);

    //
    // Return the character to the caller.
    //
    return(return_char);

}

//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is full or not.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is used to determine whether or not a given ring buffer is
//! full.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b 1 if the buffer is full or \b 0 otherwise.
//
//*****************************************************************************

uint8_t UART_is_buffer_full(volatile uint16_t *pui16Read,
             volatile uint16_t *pui16Write, uint16_t ui16Size)
{
    uint16_t ui16Write;
    uint16_t ui16Read;

    ui16Write = *pui16Write;
    ui16Read = *pui16Read;

    return((((ui16Write + 1) % ui16Size) == ui16Read) ? 1 : 0);
}


//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is empty or not.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//!
//! This function is used to determine whether or not a given ring buffer is
//! empty.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b 1 if the buffer is empty or \b 0 otherwise.
//
//*****************************************************************************

uint8_t UART_is_buffer_empty(volatile uint16_t *pui16Read,
              volatile uint16_t *pui16Write)
{
    uint16_t ui16Write;
    uint16_t ui16Read;

    ui16Write = *pui16Write;
    ui16Read = *pui16Read;

    return((ui16Read == ui16Write) ? 1 : 0);
}


//*****************************************************************************
//
//! Determines the number of bytes of data contained in a ring buffer.
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is used to determine how many bytes of data a given ring
//! buffer currently contains.  The structure of the code is specifically to
//! ensure that we do not see warnings from the compiler related to the order
//! of volatile accesses being undefined.
//!
//! \return Returns the number of bytes of data currently in the buffer.
//
//*****************************************************************************

uint16_t UART_get_buffer_count(volatile uint16_t *pui16Read,
               volatile uint16_t *pui16Write, uint16_t ui16Size)
{
    uint16_t ui16Write;
    uint16_t ui16Read;

    ui16Write = *pui16Write;
    ui16Read = *pui16Read;

    return((ui16Write >= ui16Read) ? (ui16Write - ui16Read) :
           (ui16Size - (ui16Read - ui16Write)));
}

//*****************************************************************************
//
//! Adding +1 to the index
//!
//! \param pui16Read points to the read index for the buffer.
//! \param pui16Write points to the write index for the buffer.
//! \param ui16Size is the size of the buffer in bytes.
//!
//! This function is use to advance the index by 1, if the index
//! already hit the uart size then it will reset back to 0.
//!
//! \return Returns the number of bytes of data currently in the buffer.
//
//*****************************************************************************

uint16_t UART_advance_buffer_index(volatile uint16_t* pui16Index, uint16_t ui16Size)
{
    *pui16Index = (*pui16Index + 1) % ui16Size;

    return(*pui16Index);
}


//*****************************************************************************
//
// Take as many bytes from the transmit buffer as we have space for and move
// them into the UART transmit FIFO.
//
//*****************************************************************************
void UART_Prime_Transmit(uart_stdio_typedef* p_uart)
{
    //
    // Do we have any data to transmit?
    //
    if(!TX_BUFFER_EMPTY(p_uart))
    {
        NVIC_DisableIRQ(p_uart->irqn);

        LL_USART_TransmitData8(p_uart->handle, p_uart->p_TX_buffer[p_uart->TX_read_index]);
        ADVANCE_TX_READ_INDEX(p_uart);
        
        NVIC_EnableIRQ(p_uart->irqn);
    }
}


uint16_t UART_FSP(uart_stdio_typedef* p_uart, const char *pcBuf, uint16_t ui16Len)
{

    uint8_t uIdx;

    //
    // Check for valid arguments.
    //
    //
    // Send the characters
    //
    for(uIdx = 0; uIdx < ui16Len; uIdx++)
    {
        if(!TX_BUFFER_FULL(p_uart))
        {
        	p_uart->p_TX_buffer[p_uart->TX_write_index] = pcBuf[uIdx];
            ADVANCE_TX_WRITE_INDEX(p_uart);
        }
        else
        {
            //
            // Buffer is full - discard remaining characters and return.
            //
            p_uart->p_TX_buffer[p_uart->TX_write_index] = '\r';
            break;
        }
    }

    //
    // If the usart txe irq is disable, this mean an usart phase is finished
    // we need to enable the txe irq and kick start the transmit process.
    //
    if (LL_USART_IsEnabledIT_TXE(p_uart->handle) == false)
    {
        // NOTE: Turn on TXE after prime transmit,
        // if turn on TXE b4 prime transmit create a
        // bug where the index = 2 char don't get
        // send.

        //LL_USART_EnableIT_TXE(p_uart->handle);
        UART_Prime_Transmit(p_uart);
        LL_USART_EnableIT_TXE(p_uart->handle);
    }

    //
    // Return the number of characters written.
    //
    return(uIdx);
}
