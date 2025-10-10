/******************************************************************************
* File Name        : UartDma.c
*
* Description      : This file contains all the functions and variables required 
*                    for proper operation of UART/DMA for this Code example.
*
* Related Document : See README.md
*
*******************************************************************************
* (c) 2023-2025, Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is owned by
* Infineon Technologies AG or one of its affiliates ("Infineon") and is protected
* by and subject to worldwide patent protection, worldwide copyright laws, and
* international treaty provisions. Therefore, you may use this Software only as
* provided in the license agreement accompanying the software package from which
* you obtained this Software. If no license agreement applies, then any use,
* reproduction, modification, translation, or compilation of this Software is
* prohibited without the express written permission of Infineon.
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,
* BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF THIRD-PARTY RIGHTS AND
* IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A SPECIFIC USE/PURPOSE OR
* MERCHANTABILITY. Infineon reserves the right to make changes to the Software
* without notice. You are responsible for properly designing, programming, and
* testing the functionality and safety of your intended application of the
* Software, as well as complying with any legal requirements related to its
* use. Infineon does not guarantee that the Software will be free from intrusion,
* data theft or loss, or other breaches ("Security Breaches"), and Infineon
* shall have no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any application
* where a failure of the Product or any consequences of the use thereof can
* reasonably be expected to result in personal injury.
*******************************************************************************/
#include "uart_dma.h"

/*******************************************************************************
* Global variables
*******************************************************************************/
extern uint8_t rx_dma_error;   /* RxDma error flag */
extern uint8_t tx_dma_error;   /* TxDma error flag */
extern uint8_t rx_dma_done;    /* RxDma done flag  */

/*******************************************************************************
* Function definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: configure_rx_dma
********************************************************************************
* Summary:
* Configures DMA Rx channel for operation.
*******************************************************************************/
void configure_rx_dma(uint8_t* buffer_a, uint8_t* buffer_b, cy_stc_sysint_t* 
    int_config)
{
    volatile cy_en_dma_status_t dma_init_status;

    /* Initialize descriptor 0 */
    dma_init_status = Cy_DMA_Descriptor_Init
    (&CYBSP_UART_RX_DMA_Descriptor_0,
    &CYBSP_UART_RX_DMA_Descriptor_0_config);
    if (CY_DMA_SUCCESS != dma_init_status)
    {
        handle_app_error();
    }

    /* Initialize descriptor 1 */
    dma_init_status = Cy_DMA_Descriptor_Init
    (&CYBSP_UART_RX_DMA_Descriptor_1,
    &CYBSP_UART_RX_DMA_Descriptor_1_config);

    if (CY_DMA_SUCCESS != dma_init_status)
    {
        handle_app_error();
    }

    dma_init_status = Cy_DMA_Channel_Init(CYBSP_UART_RX_DMA_HW,
            CYBSP_UART_RX_DMA_CHANNEL,
           &CYBSP_UART_RX_DMA_channelConfig);

    if (CY_DMA_SUCCESS != dma_init_status)
    {
        handle_app_error();
    }

    /* Set source and destination address for descriptor 1 */
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_UART_RX_DMA_Descriptor_0,
        (uint32_t *) &CYBSP_DEBUG_UART_HW->RX_FIFO_RD);
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_UART_RX_DMA_Descriptor_0,
        (uint32_t *) buffer_a);

    /* Set source and destination address for descriptor 2 */
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_UART_RX_DMA_Descriptor_1,
    (uint32_t *) &CYBSP_DEBUG_UART_HW->RX_FIFO_RD);
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_UART_RX_DMA_Descriptor_1,
    (uint32_t *) buffer_b);

    /* Set DMA Channel descriptor */
    Cy_DMA_Channel_SetDescriptor(CYBSP_UART_RX_DMA_HW,
            CYBSP_UART_RX_DMA_CHANNEL,
            &CYBSP_UART_RX_DMA_Descriptor_0);

    /* Initialize and enable interrupt from RxDma */
    Cy_SysInt_Init  (int_config, &rx_dma_complete);
    NVIC_EnableIRQ(int_config->intrSrc);

    /* Enable DMA interrupt source. */
    Cy_DMA_Channel_SetInterruptMask(CYBSP_UART_RX_DMA_HW,
            CYBSP_UART_RX_DMA_CHANNEL, CY_DMA_INTR_MASK);

    /* Enable channel and DMA block to start descriptor execution process */
    Cy_DMA_Channel_Enable(CYBSP_UART_RX_DMA_HW,
            CYBSP_UART_RX_DMA_CHANNEL);
    Cy_DMA_Enable(CYBSP_UART_RX_DMA_HW);
}

/*******************************************************************************
* Function Name: configure_tx_dma
********************************************************************************
* Summary:
* Configures DMA Tx channel for operation.
*******************************************************************************/
void configure_tx_dma(uint8_t* buffer_a, cy_stc_sysint_t* int_config)
{
    volatile cy_en_dma_status_t dma_init_status;

    /* Init descriptor */
    dma_init_status = Cy_DMA_Descriptor_Init(&CYBSP_UART_TX_DMA_Descriptor_0,
    &CYBSP_UART_TX_DMA_Descriptor_0_config);

    if (CY_DMA_SUCCESS != dma_init_status)
    {
        handle_app_error();
    }

    /* Init DMA Channel */
    dma_init_status = Cy_DMA_Channel_Init(CYBSP_UART_TX_DMA_HW,
            CYBSP_UART_TX_DMA_CHANNEL,
            &CYBSP_UART_TX_DMA_channelConfig);

    if (CY_DMA_SUCCESS != dma_init_status)
    {
        handle_app_error();
    }

    /* Set source and destination for descriptor 1 */
    Cy_DMA_Descriptor_SetSrcAddress(&CYBSP_UART_TX_DMA_Descriptor_0,
    (uint32_t *) buffer_a);
    Cy_DMA_Descriptor_SetDstAddress(&CYBSP_UART_TX_DMA_Descriptor_0,
    (uint32_t *) &CYBSP_DEBUG_UART_HW->TX_FIFO_WR);

   /* Set next descriptor to NULL to stop the chain execution after descriptor 1
    *  is completed.
    */
    Cy_DMA_Descriptor_SetNextDescriptor(Cy_DMA_Channel_GetCurrentDescriptor
    (CYBSP_UART_TX_DMA_HW, CYBSP_UART_TX_DMA_CHANNEL), NULL);

    /* Initialize and enable the interrupt from TxDma */
    Cy_SysInt_Init  (int_config, &tx_dma_complete);
    NVIC_EnableIRQ(int_config->intrSrc);

    /* Enable DMA interrupt source */
    Cy_DMA_Channel_SetInterruptMask(CYBSP_UART_TX_DMA_HW,
            CYBSP_UART_TX_DMA_CHANNEL, CY_DMA_INTR_MASK);

   /* Enable Data Write block but keep channel disabled to not trigger
    *  descriptor execution because TX FIFO is empty and SCB keeps active level
    *  for DMA. 
    */
    Cy_DMA_Enable(CYBSP_UART_TX_DMA_HW);
}

/*******************************************************************************
* Function Name: rx_dma_complete
********************************************************************************
* Summary:
* Handles Rx Dma descriptor completion interrupt source: triggers Tx Dma to
* transfer back data received by the Rx Dma descriptor.
*******************************************************************************/
void rx_dma_complete(void)
{
    Cy_DMA_Channel_ClearInterrupt(CYBSP_UART_RX_DMA_HW,
            CYBSP_UART_RX_DMA_CHANNEL);

    /* Check interrupt cause to capture errors. */
    if (CY_DMA_INTR_CAUSE_COMPLETION == Cy_DMA_Channel_GetStatus
        (CYBSP_UART_RX_DMA_HW, CYBSP_UART_RX_DMA_CHANNEL))
    {
        rx_dma_done = SET_BIT;
    }
    else
    {
        /* DMA error occurred while RX operations */
        rx_dma_error = SET_BIT;
    }
}

/*******************************************************************************
* Function Name: tx_dma_complete
********************************************************************************
*
* Summary:
* Handles Tx Dma descriptor completion interrupt source: only used for
* indication.
*******************************************************************************/
void tx_dma_complete(void)
{
   /* Check interrupt cause to capture errors.
    *  Note that next descriptor is NULL to stop descriptor execution 
    */
    if ((CY_DMA_INTR_CAUSE_COMPLETION    != Cy_DMA_Channel_GetStatus
        (CYBSP_UART_TX_DMA_HW, CYBSP_UART_TX_DMA_CHANNEL)) &&
        (CY_DMA_INTR_CAUSE_CURR_PTR_NULL != Cy_DMA_Channel_GetStatus
        (CYBSP_UART_TX_DMA_HW, CYBSP_UART_TX_DMA_CHANNEL)))
    {
        /* DMA error occurred while TX operations */
        tx_dma_error = SET_BIT;
    }

    Cy_DMA_Channel_ClearInterrupt(CYBSP_UART_TX_DMA_HW,
            CYBSP_UART_TX_DMA_CHANNEL);
}


/* [] END OF FILE */