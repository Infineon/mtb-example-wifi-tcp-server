/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for TCP Server Example in
* ModusToolbox.
*
* Related Document: See Readme.md
*
*******************************************************************************
* (c) 2019, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/

/* Header file includes */
#include "cyhal.h"
#include "cybsp.h"
#include "cybsp_wifi.h"
#include "cy_retarget_io.h"
#include "string.h"

/* FreeRTOS header file */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* lwIP header files */
#include <lwip/tcpip.h>
#include <lwip/api.h>
#include <lwipinit.h>
#include "ip4_addr.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* Wi-Fi Credentials: Modify WIFI_SSID and WIFI_PASSWORD to match your Wi-Fi network
 * Credentials.
 */
#define WIFI_SSID                         "MY-WIFI-SSID"
#define WIFI_PASSWORD                     "MY-PASSWORD"
#define MAX_CONNECTION_RETRIES            (10u)

#define TCP_SERVER_PORT                   50007

/* 32-bit task notification value for the tcp_server_task */
#define TCP_SERVER_LISTEN_OFF             (0x00lu)
#define TCP_SERVER_LISTEN_ON              (0x01lu)

#define USER_BTN1_INTR_PRIORITY           (5)

#define TCP_SERVER_TASK_STACK_SIZE        (1024*5)
#define TCP_SERVER_TASK_PRIORITY          (1)
#define TCP_SERVER_TASK_QUEUE_LEN         (10)
#define RTOS_TASK_TICKS_TO_WAIT           (100)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void isr_button_press( void *callback_arg, cyhal_gpio_event_t event);
void tcp_server_task(void *arg);

/*******************************************************************************
* Global Variables
********************************************************************************/
/* The primary WIFI driver. */
static whd_interface_t iface;

/* 32-bit task notification value to notify TCP server task to start/stop
 * listening on a TCP port.
 */
uint32_t server_state = TCP_SERVER_LISTEN_OFF;

/* TCP Sever task handle. */
TaskHandle_t tcp_server_task_handle;

/* This enables RTOS aware debugging. */
volatile int uxTopUsedPriority;

struct netif *net;

static const char *write_buffer = "Hello World!";


/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function sets up user tasks and then starts
 *  the RTOS scheduler.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int
 *
 *******************************************************************************/
int main()
{
    cy_rslt_t result ;

    /* This enables RTOS aware debugging in OpenOCD. */
    uxTopUsedPriority = configMAX_PRIORITIES - 1 ;

    /* Initialize the board support package. */
    result = cybsp_init() ;
    CY_ASSERT(result == CY_RSLT_SUCCESS) ;

    /* Initialize User button 1 and register interrupt on falling edge. */
    cyhal_gpio_init(CYBSP_USER_BTN1, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, 1);
    cyhal_gpio_register_callback(CYBSP_USER_BTN1, isr_button_press, NULL);
    cyhal_gpio_enable_event(CYBSP_USER_BTN1, CYHAL_GPIO_IRQ_FALL, USER_BTN1_INTR_PRIORITY, 1);

     /* Initialize the User LED. */
    cyhal_gpio_init((cyhal_gpio_t) CYBSP_USER_LED, CYHAL_GPIO_DIR_OUTPUT,
                    CYHAL_GPIO_DRIVE_PULLUP, CYBSP_LED_STATE_OFF);

    /* Enable global interrupts. */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port. */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX, 
                        CY_RETARGET_IO_BAUDRATE);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence to clear screen. */
    printf("\x1b[2J\x1b[;H");
    printf("===============================================================\n");
    printf("CE229153 - ModusToolbox Connectivity Example: TCP Server\n");
    printf("===============================================================\n\n");

    /* Create the tasks. */
    xTaskCreate(tcp_server_task, "Network task", TCP_SERVER_TASK_STACK_SIZE, NULL, 
               TCP_SERVER_TASK_PRIORITY, &tcp_server_task_handle);

    /* Start the FreeRTOS scheduler. */
    vTaskStartScheduler();

    /* Should never get here. */
    CY_ASSERT(0);
}

/*******************************************************************************
 * Function Name: isr_button_press
 *******************************************************************************
 *
 * Summary:
 *  GPIO interrupt service routine. This function is used to detect button
 *  presses and send task notifications to the TCP server task.
 *
 * Parameters:
 *  void *callback_arg : pointer to variable passed to the ISR
 *  cyhal_gpio_event_t event : GPIO event type
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void isr_button_press( void *callback_arg, cyhal_gpio_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(server_state == TCP_SERVER_LISTEN_OFF)
    {
        server_state = TCP_SERVER_LISTEN_ON;

        /* Notify the TCP server task to start listening on the port */
        xTaskNotifyFromISR(tcp_server_task_handle, server_state, eSetValueWithoutOverwrite, 
                           &xHigherPriorityTaskWoken);

        portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    }
}


/*******************************************************************************
 * Function Name: tcp_server_task
 *******************************************************************************
 * Summary:
 *  Task used to establish a connection to a remote TCP client.
 *
 * Parameters:
 *  void *args : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void tcp_server_task(void *arg)
{
    cy_rslt_t result;
    whd_ssid_t ssid_data;
    const char *ssid = WIFI_SSID;
    const char *key = WIFI_PASSWORD;

    struct netconn *conn = NULL, *client_conn = NULL;
    struct netbuf * data_buffer = NULL;
    char *ptr ;
    uint16_t plen, tcp_server_port = TCP_SERVER_PORT;
    err_t err;

    /* Initialize and start the tcpip_thread. */
    tcpip_init(NULL, NULL) ;
    printf("lwIP TCP/IP stack initialized\n");

    /* Initialize the Wi-Fi Driver. */
    result = cybsp_wifi_init_primary(&iface);

    if(result == CY_RSLT_SUCCESS)
    {
        printf("Wi-Fi driver initialized \n");
    }
    else
    {
        printf("Wi-Fi driver initialization failed!\n");
        CY_ASSERT(0);
    }
    
     /* Variable to track the number of connection retries to the Wi-Fi AP
      * specified by WIFI_SSID macro.
      */
    int conn_retries = 0;

    /* Join the Wi-Fi AP */
    result = WHD_PENDING;
    ssid_data.length = strlen(ssid);
    memcpy(ssid_data.value, ssid, ssid_data.length);
    while(result != CY_RSLT_SUCCESS && conn_retries < MAX_CONNECTION_RETRIES)
    {
        result = whd_wifi_join(iface, &ssid_data, WHD_SECURITY_WPA2_AES_PSK,
                               (const uint8_t *)key, strlen(key));
        conn_retries++;
    }

    if(result == CY_RSLT_SUCCESS)
    {
        printf("Successfully joined Wi-Fi network '%s'\n", ssid);
    }
    else
    {
        printf("Failed to join Wi-Fi network '%s'\n", ssid);
        CY_ASSERT(0);
    }

    /* Add the Wi-Fi interface to the lwIP stack. */
    result = add_interface_to_lwip(iface, NULL);
    if(result == CY_RSLT_SUCCESS)
    {
        printf("Wi-Fi interface added to TCP/IP stack\n");
    }
    else
    {
        printf("Failed to add Wi-Fi interface to lwIP stack!\n");
        CY_ASSERT(0);
    }

    /* Fetch the IP address assigned based on the added Wi-Fi interface. */
    net = get_lwip_interface();

    /* Wait till IP address is assigned. */
    while(net->ip_addr.u_addr.ip4.addr == 0)
    {
        vTaskDelay(RTOS_TASK_TICKS_TO_WAIT);
    }
    printf("IP Address %s assigned\n", ip4addr_ntoa(&net->ip_addr.u_addr.ip4));

    /* Create and bind to a TCP socket */
    conn = netconn_new(NETCONN_TCP) ;
    if (conn == NULL)
    {
        printf("Failed to create a TCP socket\n");
        CY_ASSERT(0);
    }

    err = netconn_bind(conn, &(net->ip_addr), tcp_server_port) ;
    if (err != ERR_OK)
    {
        netconn_delete(conn) ;
        printf("netconn_bind returned error. Error: %d", err);
        CY_ASSERT(0);
    }

    while(true)
    {
        printf("===============================================================\n");
        printf("Press User Button (SW2) to start listening.\n");
        /* Block till User Button (SW2) is pressed. */
        xTaskNotifyWait(0, 0, &server_state, portMAX_DELAY);

        /* Accept new connections */
        printf("Waiting for connections...\n");
        err = netconn_listen(conn) ;
        if (err != ERR_OK)
        {
            netconn_delete(conn);
            printf("netconn_listen returned error. Error: %d", err);
            CY_ASSERT(0);
        }

        /* Turn ON the user LED to indicate that the server is listening. */
        cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, CYBSP_LED_STATE_ON);

        if( (err = netconn_accept(conn, &client_conn)) == ERR_OK)
        {
            printf("Info: Incoming connection from client\n");

            /* Read the data from the TCP client. */
            err = netconn_recv(client_conn, &data_buffer) ;
            if (err != ERR_OK)
            {
                printf("netconn_recv returned error. Error %d", err);
                CY_ASSERT(0);
            }
            else
            {
                netbuf_data(data_buffer, (void *)&ptr, &plen) ;
                for(int i = 0; i < plen; i++)
                {
                    printf("%c", ptr[i]);
                }
                printf("\n");
            }
            
            /* Write data to the TCP client. */
            err = netconn_write(client_conn, write_buffer, strlen(write_buffer), NETCONN_NOFLAG);
            if(err != ERR_OK)
            {
                printf("netconn_write returned error. Error %d", err);
                CY_ASSERT(0);
            }

            /* Set the TCP server state as TCP_SERVER_LISTEN_OFF.
             * Turn OFF the User LED to indicate that server has stopped listening on the port.
             */
            server_state = TCP_SERVER_LISTEN_OFF;
            cyhal_gpio_write((cyhal_gpio_t) CYBSP_USER_LED, CYBSP_LED_STATE_OFF);
        }
      }
 }

/* [] END OF FILE */
