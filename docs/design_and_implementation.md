[Click here](../README.md) to view the README.

## Design and implementation

The design of this application is minimalistic to get started with code examples on PSOC&trade; Edge MCU devices. All PSOC&trade; Edge E84 MCU applications have a dual-CPU three-project structure to develop code for the CM33 and CM55 cores. The CM33 core has two separate projects for the secure processing environment (SPE) and non-secure processing environment (NSPE). A project folder consists of various subfolders, each denoting a specific aspect of the project. The three project folders are as follows:

**Table 1. Application projects**

Project | Description
--------|------------------------
*proj_cm33_s* | Project for CM33 secure processing environment (SPE)
*proj_cm33_ns* | Project for CM33 non-secure processing environment (NSPE)
*proj_cm55* | CM55 project

<br>

In this code example, at device reset, the secure boot process starts from the ROM boot with the secure enclave (SE) as the root of trust (RoT). From the secure enclave, the boot flow is passed on to the system CPU subsystem where the secure CM33 application starts. After all necessary secure configurations, the flow is passed on to the non-secure CM33 application. Resource initialization for this example is performed by this CM33 non-secure project. It configures the system clocks, pins, clock to peripheral connections, and other platform resources. It then enables the CM55 core using the `Cy_SysEnableCM55()` function and the CM55 core is subsequently put to DeepSleep mode.

In the CM33 non-secure application, an UART block is configured to receive and send data to a terminal emulator. The application uses DMA to handle the data received in the UART Rx FIFO. Two DMA channels are used to handle the data in transmit and receive directions. Two SRAM buffers are alternately used on the receive side to hold the data received from the UART terminal. These buffers are called "ping-pong buffers" and are mainly used to provide the time for pulling the data out of either buffer.

The RxDma resource handles the data transfer in the receive direction. RxDma has two descriptors in the chain; these two descriptors are configured such that the source alternates between the ping-pong buffers in the receive direction.

The TxDma resource is used to handle data in the transmit direction. TxDma has only one descriptor configured to transfer a single element alternately from the ping-pong buffers. This data is finally echoed back (see **Figure 1**).

**Figure 1. Buffering**

![](../images/buffering.png)

<br>