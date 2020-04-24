/*
 *
 * [File Name]     hello_world.c
 * [Platform]      DE0-nano
 * [Project]       NiosII & FreeRTOS
 * [Version]       1.00
 * [Author]        ea.rincon11
 * [Date]          24/07/2017
 * [Language]      'C'
 * [History]       1.20 - Edited from original file included in FreeRTOS V9.0.0 Nios Example
 */
//=====================================================================================================================
// Standard C/C++ Includes
//=====================================================================================================================
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

//=====================================================================================================================
// Intel FPGA HAL Includes
//=====================================================================================================================
#include <altera_avalon_sgdma.h>
#include <altera_avalon_sgdma_descriptor.h>
#include <altera_avalon_sgdma_regs.h>
#include "sys/alt_stdio.h"
#include "sys/alt_irq.h"

//=====================================================================================================================
// Scheduler Includes
//=====================================================================================================================
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "partest.h"
#include "semphr.h"

//=====================================================================================================================
// Constants
//=====================================================================================================================
/* The rate at which the Print out message controlled by the 'Task1' */
#define mainTASK1_PERIOD	( 500 )

/* The rate at which the Print out message controlled by the 'Task2' */
#define mainTASK2_PERIOD	( 1000 )

/* Priority definitions for the tasks in the demo application. */
#define mainTASK1_PRIORITY		( tskIDLE_PRIORITY + 1 )

/* Priority definitions for the tasks in the demo application. */
#define mainTASK2_PRIORITY	( tskIDLE_PRIORITY + 2 )

//=====================================================================================================================
// Function Prototypes
//=====================================================================================================================
static void prvPrintTask1( void *pvParameters );		// Functions exemplo de tasks
static void prvPrintTask2( void *pvParameters );		// Functions exemplo de tasks
BaseType_t 	xNetworkInterfaceInitialise( void );		// Function para inicializar o hardware MAC
static void prvEMACHandlerTask( void *pvParameters );	// A deferred interrupt handler task that processes
void 		rx_ethernet_isr( void *context );			// Rotina de interrupt da porta ethernet

//=====================================================================================================================
// Create sgdma transmit and receive devices
//=====================================================================================================================
alt_sgdma_dev * sgdma_tx_dev;
alt_sgdma_dev * sgdma_rx_dev;

//=====================================================================================================================
// Allocate descriptors in the descriptor_memory (onchip memory)
//=====================================================================================================================
alt_sgdma_descriptor tx_descriptor		__attribute__ (( section ( ".descriptor_memory" )));
alt_sgdma_descriptor tx_descriptor_end	__attribute__ (( section ( ".descriptor_memory" )));

alt_sgdma_descriptor rx_descriptor  	__attribute__ (( section ( ".descriptor_memory" )));
alt_sgdma_descriptor rx_descriptor_end  __attribute__ (( section ( ".descriptor_memory" )));

//=====================================================================================================================
// Global Variables
//=====================================================================================================================
unsigned int text_length;

// Create a transmit frame
unsigned char tx_frame[1024] = {
	0x00,0x00, 						// for 32-bit alignment
	0xFF,0xFF,0xFF,0xFF,0xFF,0xFF, 	// destination address (broadcast)
	0x01,0x60,0x6E,0x11,0x02,0x0F, 	// source address
	0x00,0x2E, 						// length or type of the payload data
	'\0' 							// payload data (ended with termination character)
};

// Create a receive frame
unsigned char rx_frame[2048] = { 0 };

//The semaphore used to wake the deferred interrupt handler task when an Rx interrupt is received.
SemaphoreHandle_t xEMACRxEventSemaphore = NULL;


//=====================================================================================================================
// Main Function
//=====================================================================================================================
int main()
{
	// Configure any hardware required for this demo.
	vParTestInitialise();

	// Initialize the MAC hardware
	xNetworkInterfaceInitialise();

	printf("Hello from Nios II!\n");

	// prvPrintTask1 uses sprintf so requires more stack.
	xTaskCreate( prvPrintTask1, "Task1", configMINIMAL_STACK_SIZE, NULL, mainTASK1_PRIORITY, NULL );

	// prvPrintTask2 uses sprintf so requires more stack.
	xTaskCreate( prvPrintTask2, "Task2", configMINIMAL_STACK_SIZE, NULL, mainTASK2_PRIORITY, NULL );

	// prvPrintTask2 uses sprintf so requires more stack.
	xTaskCreate( prvEMACHandlerTask, "interrupt", configMINIMAL_STACK_SIZE, NULL, 1, NULL );

    // Finally start the scheduler
	vTaskStartScheduler();

	// Will only reach here if there is insufficient heap available to start the scheduler.
	for( ;; );

  return 0;
}

//=====================================================================================================================
// Tasks Functions
//=====================================================================================================================
static void prvPrintTask1( void *pvParameters )
{
	FILE* stream_UART;
	stream_UART = fopen ("/dev/uart", "w");
	if (stream_UART == NULL)
	{
		printf("Nao foi criada stream UART");
	}
	for( ;; )
	{
		// Wait until it is time to run the tests again.
		vTaskDelay( mainTASK1_PERIOD / portTICK_PERIOD_MS);

        // Print out an message
        printf( "NIOS II Task1 from jtag\r\n" );
        fprintf(stream_UART, "NIOS II Task1 from uart\r\n ");

        // Control LED 1 DE0-NANO
        vParTestToggleLED(1);
	}
}

static void prvPrintTask2( void *pvParameters )
{
	for( ;; )
	{
		/* Wait until it is time to run the tests again. */
		vTaskDelay( mainTASK2_PERIOD / portTICK_PERIOD_MS);

        /* Print out an message */
        printf( "NIOS II Task2\r\n" );

        //Control LED 2 DE0-NANO
        vParTestToggleLED(2);
	}
}

//=====================================================================================================================
// Functions
//=====================================================================================================================
BaseType_t xNetworkInterfaceInitialise( void )
{

	// Configura SGDMA
	//===================================================================================================================================
	// Open the sgdma transmit device
	//----------------------------------------------------------------------------------------
	sgdma_tx_dev = alt_avalon_sgdma_open ("/dev/sgdma_tx");

	if (sgdma_tx_dev == NULL) {
		alt_printf ("Error: could not open scatter-gather dma transmit device\n");
		return pdFAIL;
	} else alt_printf ("Opened scatter-gather dma transmit device\n");

	// Open the sgdma receive device
	//----------------------------------------------------------------------------------------
	sgdma_rx_dev = alt_avalon_sgdma_open ("/dev/sgdma_rx");

	if (sgdma_rx_dev == NULL) {
		alt_printf ("Error: could not open scatter-gather dma receive device\n");
		return pdFAIL;
	} else alt_printf ("Opened scatter-gather dma receive device\n");

	// Set interrupts for the sgdma receive device
	alt_avalon_sgdma_register_callback( sgdma_rx_dev, (alt_avalon_sgdma_callback) rx_ethernet_isr, 0x00000014, NULL );

	// Create new receive sgdma descriptor
	//===================================================================================================================================
	alt_avalon_sgdma_construct_stream_to_mem_desc( &rx_descriptor, &rx_descriptor_end, (alt_u32 *)rx_frame, 0, 0 );

	// Set up of sgdma receive descriptor
	//===================================================================================================================================
	alt_avalon_sgdma_do_async_transfer( sgdma_rx_dev, &rx_descriptor );

	// Configura Triple Speed Ethernet
	//===================================================================================================================================
	// Triple-speed Ethernet MegaCore base address
	volatile int *tse = (int *) TSE_BASE;

	// Initialize the MAC address
	//----------------------------------------------------------------------------------------
	*(tse + 3) = 0x116E6001;
	*(tse + 4) = 0x00000F02;

	// Specify the addresses of the PHY devices to be accessed through MDIO interface
	//----------------------------------------------------------------------------------------
	*(tse + 0x0F) = 0x10;
	*(tse + 0x10) = 0x11;

	// Write to register 20 of the PHY chip for Ethernet port 0 to set up line loopback
	//----------------------------------------------------------------------------------------
	*(tse + 0x94) = 0x4000;

	// Write to register 16 of the PHY chip for Ethernet port 1 to enable automatic crossover for all modes
	//----------------------------------------------------------------------------------------
	*(tse + 0xB0) = *(tse + 0xB0) | 0x0060;

	// Write to register 20 of the PHY chip for Ethernet port 2 to set up delay for input/output clk
	//----------------------------------------------------------------------------------------
	*(tse + 0xB4) = *(tse + 0xB4) | 0x0082;

	// Software reset the second PHY chip and wait
	//----------------------------------------------------------------------------------------
	*(tse + 0xA0) = *(tse + 0xA0) | 0x8000;
	while ( *(tse + 0xA0) & 0x8000  );

	// Enable read and write transfers, gigabit Ethernet operation, and CRC forwarding
	//----------------------------------------------------------------------------------------
	*(tse + 2) = *(tse + 2) | 0x0000004B;


	vSemaphoreCreateBinary( xEMACRxEventSemaphore );

	return pdPASS;
}

//=====================================================================================================================
// Subroutine to read incoming Ethernet frames
//=====================================================================================================================
void rx_ethernet_isr (void *context)
{
	// Wait until receive descriptor transfer is complet
	while (alt_avalon_sgdma_check_descriptor_status(&rx_descriptor) != 0);

	xSemaphoreGiveFromISR( xEMACRxEventSemaphore, NULL );


	// Create new receive sgdma descriptor
	//===================================================================================================================================
	alt_avalon_sgdma_construct_stream_to_mem_desc( &rx_descriptor, &rx_descriptor_end, (alt_u32 *)rx_frame, 0, 0 );

	// Set up of sgdma receive descriptor
	//===================================================================================================================================
	alt_avalon_sgdma_do_async_transfer( sgdma_rx_dev, &rx_descriptor );
}

static void prvEMACHandlerTask( void *pvParameters )
{
	alt_u16 valor = 0;
	while(1)
	{
		if (xSemaphoreTake( xEMACRxEventSemaphore, portMAX_DELAY ))
		{
			// Teste para novo received frame
			alt_printf( "receive> %s \n", rx_frame+295 );

			// Reprint current input line after the output
			alt_printf( "\nsend> \n");
		}
	}


}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
