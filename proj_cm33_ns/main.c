/*******************************************************************************
* File Name       :   main.c
*
* Description     : This source file contains the main routine for non-secure
*                   application in the CM33 CPU. This example demonstrates the 
*                   UART transmit and receive operation using DMA.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2023-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "uart_dma.h"
#include "cybsp.h"

/*******************************************************************************
* Constants
*******************************************************************************/
#define RESET_VAL                         (0U)
#define SET_VAL                           (1U)

/* The timeout value in microsecond used to wait for core to be booted */
#define CM55_BOOT_WAIT_TIME_USEC          (10U)

/*Interrupt priority of UART and DMA interrupts*/
#define INT_PRIORITY                      (7U)

/* App boot address for CM55 project */
#define CM55_APP_BOOT_ADDR                (CYMEM_CM33_0_m55_nvm_START + \
                                           CYBSP_MCUBOOT_HEADER_SIZE)

/*******************************************************************************
* Global variables
*******************************************************************************/
volatile uint8_t rx_dma_error;   /* RxDma error flag */
volatile uint8_t tx_dma_error;   /* TxDma error flag */
volatile uint8_t uart_error;     /* UART error flag */
volatile uint8_t rx_dma_done;    /* RxDma done flag */

/*******************************************************************************
* Function definitions
*******************************************************************************/
/*******************************************************************************
* Function Name: handle_app_error
********************************************************************************
* Summary:
* User defined error handling function.
*******************************************************************************/
void handle_app_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);

    /* Infinite loop */
    while(true);
}

/*******************************************************************************
* Function Name: isr_uart
********************************************************************************
* Summary:
* Handles UART Rx underflow and overflow conditions. This conditions must never
* occur.
*******************************************************************************/
void isr_uart(void)
{
    uint32 intrSrcRx;
    uint32 intrSrcTx;

    /* Get RX interrupt sources */
    intrSrcRx = Cy_SCB_UART_GetRxFifoStatus(CYBSP_DEBUG_UART_HW);
    Cy_SCB_UART_ClearRxFifoStatus(CYBSP_DEBUG_UART_HW, intrSrcRx);

    /* Get TX interrupt sources */
    intrSrcTx = Cy_SCB_UART_GetTxFifoStatus(CYBSP_DEBUG_UART_HW);
    Cy_SCB_UART_ClearTxFifoStatus(CYBSP_DEBUG_UART_HW, intrSrcTx);

    /* RX overflow or RX underflow or RX overflow occured */
    uart_error = SET_VAL;
}

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
* The main function performs the following actions:
*  1. Configures RX and TX DMAs to handle UART RX+TX direction.
*  2. Configures UART component.
*  3. Turn on USER LED 1 if the UART initialization has failed.
*  4. Sends text header to the UART serial terminal.
*  5. Waits in an infinite loop (for DMA or UART error interrupt).
*
* Parameters:
* None
*
* Return:
* int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    
    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if ( CY_RSLT_SUCCESS != result )
    {
        handle_app_error();
    }

    uint8_t rx_dma_uart_buffer_a[BUFFER_SIZE];
    uint8_t rx_dma_uart_buffer_b[BUFFER_SIZE];
    cy_en_scb_uart_status_t init_status;
    cy_stc_scb_uart_context_t CYBSP_DEBUG_UART_context;

    /* flag to control which descriptor to use */
    uint32_t active_descr = DMA_DESCR0; 
    cy_stc_sysint_t CYBSP_DEBUG_UART_INT_cfg =
    {
        .intrSrc      = CYBSP_DEBUG_UART_IRQ,
        .intrPriority = INT_PRIORITY,
    };
    cy_stc_sysint_t RX_DMA_INT_cfg =
    {
        .intrSrc      = (IRQn_Type)CYBSP_UART_RX_DMA_IRQ,
        .intrPriority = INT_PRIORITY,
    };
    cy_stc_sysint_t TX_DMA_INT_cfg =
    {
        .intrSrc      = (IRQn_Type)CYBSP_UART_TX_DMA_IRQ,
        .intrPriority = INT_PRIORITY,
    };

    /* Configure DMA Rx and Tx channels for operation */
    configure_rx_dma(rx_dma_uart_buffer_a, rx_dma_uart_buffer_b,
            &RX_DMA_INT_cfg);
    configure_tx_dma(rx_dma_uart_buffer_a, &TX_DMA_INT_cfg);

   /* Initialize and enable interrupt from UART. The UART interrupt sources
    *  are enabled in the Component GUI 
    */
    Cy_SysInt_Init(&CYBSP_DEBUG_UART_INT_cfg, &isr_uart);
    NVIC_EnableIRQ(CYBSP_DEBUG_UART_INT_cfg.intrSrc);

    /* Start UART operation */
    init_status = Cy_SCB_UART_Init(CYBSP_DEBUG_UART_HW,
                                  &CYBSP_DEBUG_UART_config,
                                  &CYBSP_DEBUG_UART_context);

    if ( CY_SCB_UART_SUCCESS != init_status)
    {
        /* Turn ON USER LED 1 to indicate UART initialization has failed */
        Cy_GPIO_Set(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN);
        handle_app_error();
    }

    Cy_SCB_UART_Enable(CYBSP_DEBUG_UART_HW);

    /* Transmit header to the terminal */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    Cy_SCB_UART_PutString(CYBSP_DEBUG_UART_HW, "\x1b[2J\x1b[;H");

    /* Display header information */
    Cy_SCB_UART_PutString(CYBSP_DEBUG_UART_HW, "**************");
    Cy_SCB_UART_PutString(CYBSP_DEBUG_UART_HW, " PSOC Edge MCU: "
        "UART transmit and receive using DMA ");
    Cy_SCB_UART_PutString(CYBSP_DEBUG_UART_HW, "**************\r\n\n");
    Cy_SCB_UART_PutString(CYBSP_DEBUG_UART_HW, ">> Start typing "
        "to see the echo on the screen \r\n\n");

    /* Initialize flags */
    rx_dma_error = RESET_VAL;
    tx_dma_error = RESET_VAL;
    uart_error = RESET_VAL;
    rx_dma_done = RESET_VAL;

    /* Enable global interrupts */
    __enable_irq();

   /* Enable CM55. CM55_APP_BOOT_ADDR must be updated if CM55
    * memory layout is changed.
    */
    Cy_SysEnableCM55(MXCM55, CM55_APP_BOOT_ADDR, CM55_BOOT_WAIT_TIME_USEC);
 
    while (1)
    {
        /* Indicate status if RxDma error or TxDma error or UART error occurs */
        if (rx_dma_error || tx_dma_error || uart_error)
        {
            handle_app_error();
        }

        /* Handle RxDma complete */
        if (rx_dma_done)
        {
            /* Ping Pong between rx_dma_uart_buffer_a and rx_dma_uart_buffer_b 
             * Ping Pong buffers give firmware time to pull the data out of one
             * or the other buffer 
             */
            if (DMA_DESCR0 == active_descr)
            {
                /* Set source RX Buffer A as source for TxDMA */
                Cy_DMA_Descriptor_SetSrcAddress
                (&CYBSP_UART_TX_DMA_Descriptor_0,
                        (uint32_t *)rx_dma_uart_buffer_a);
                active_descr = DMA_DESCR1;
            }
            else
            {
                /* Set source RX Buffer B as source for TxDMA */
                Cy_DMA_Descriptor_SetSrcAddress
                (&CYBSP_UART_TX_DMA_Descriptor_0,(uint32_t *)
                        rx_dma_uart_buffer_b);
                active_descr = DMA_DESCR0;
            }

            Cy_DMA_Channel_SetDescriptor(CYBSP_UART_TX_DMA_HW,
                    CYBSP_UART_TX_DMA_CHANNEL,
                    &CYBSP_UART_TX_DMA_Descriptor_0);
            Cy_DMA_Channel_Enable(CYBSP_UART_TX_DMA_HW,
                    CYBSP_UART_TX_DMA_CHANNEL);
            rx_dma_done = RESET_VAL;
        }
    }
}


/* [] END OF FILE */