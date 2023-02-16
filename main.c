//#include <hardware/irq.h>
//#include <hardware/uart.h>
#include <pico/stdlib.h>
//#include <stdio.h>
//#include <string.h>
//#include <pico/cyw43_arch.h>
//#include "btstack_run_loop.h"

#include "btstack.h"

#define RFCOMM_SERVER_CHANNEL 1
#define HEARTBEAT_PERIOD_MS 500

#define UART_ID uart0
#define BAUD_RATE 2400
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE

#define UART_TX_PIN 0
#define UART_RX_PIN 1

static uint8_t  spp_service_buffer[150];
static btstack_packet_callback_registration_t hci_event_callback_registration;
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static btstack_timer_source_t heartbeat;
static char lineBuffer[30];

int rfcomm_channel_id;

void on_uart_rx() {
    int i = 0;
    while (uart_is_readable(UART_ID)) {

        uint8_t ch = uart_getc(UART_ID);
        lineBuffer[i] = ch;
        i++;
        if (i==sizeof(lineBuffer)){
            rfcomm_request_can_send_now_event(rfcomm_channel_id);
        }
   }
    rfcomm_request_can_send_now_event(rfcomm_channel_id);
}

static void spp_service_setup(void){

    // register for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    l2cap_init();

#ifdef ENABLE_BLE
    // Initialize LE Security Manager. Needed for cross-transport key derivation
    sm_init();
#endif

    rfcomm_init();
    rfcomm_register_service(packet_handler, RFCOMM_SERVER_CHANNEL, 0xffff);  // reserved channel, mtu limited by l2cap

    // init SDP, create record for SPP and register with SDP
    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    spp_create_sdp_record(spp_service_buffer, 0x10001, RFCOMM_SERVER_CHANNEL, "SerialBT");
    sdp_register_service(spp_service_buffer);
    printf("SDP service record size: %u\n", de_get_len(spp_service_buffer));
}

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);

    bd_addr_t event_addr;
    uint8_t   rfcomm_channel_nr;
    uint16_t  mtu;
    int i;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case HCI_EVENT_PIN_CODE_REQUEST:
                    // inform about pin code request
                    printf("Pin code request - using '0000'\n");
                    hci_event_pin_code_request_get_bd_addr(packet, event_addr);
                    gap_pin_code_response(event_addr, "0000");
                    break;

                case HCI_EVENT_USER_CONFIRMATION_REQUEST:
                    // ssp: inform about user confirmation request
                    //printf("SSP User Confirmation Request with numeric value '%06"PRIu32"'\n", little_endian_read_32(packet, 8));
                    printf("SSP User Confirmation Auto accept\n");
                    break;

                case RFCOMM_EVENT_INCOMING_CONNECTION:
                    rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
                    rfcomm_channel_nr = rfcomm_event_incoming_connection_get_server_channel(packet);
                    rfcomm_channel_id = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
                    printf("RFCOMM channel %u requested for %s\n", rfcomm_channel_nr, bd_addr_to_str(event_addr));
                    rfcomm_accept_connection(rfcomm_channel_id);
                    break;
               
                case RFCOMM_EVENT_CHANNEL_OPENED:
                    if (rfcomm_event_channel_opened_get_status(packet)) {
                        printf("RFCOMM channel open failed, status %u\n", rfcomm_event_channel_opened_get_status(packet));
                    } else {
                        rfcomm_channel_id = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
                        mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
                        printf("RFCOMM channel open succeeded. New RFCOMM Channel ID %u, max frame size %u\n", rfcomm_channel_id, mtu);
                    }
                    break;
                case RFCOMM_EVENT_CAN_SEND_NOW:
                    rfcomm_send(rfcomm_channel_id, (uint8_t*) lineBuffer, (uint16_t) strlen(lineBuffer));  
                    break;

                case RFCOMM_EVENT_CHANNEL_CLOSED:
                    printf("RFCOMM channel closed\n");
                    rfcomm_channel_id = 0;
                    break;
                
                default:
                    break;
            }
            break;

        case RFCOMM_DATA_PACKET:
            printf("RCV: '");
            for (i=0;i<size;i++){
                putchar(packet[i]);
                uart_putc(UART_ID, packet[i]);
            }
            printf("'\n");
            break;

        default:
            break;
    }
}
int btstack_main(){

    stdio_init_all();

    spp_service_setup();

    gap_discoverable_control(1);
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_set_local_name("SerialBT");

    // turn on!
    hci_power_control(HCI_POWER_ON);
    
    return 0;
}

int main(void)
{

	//set_sys_clock_khz(250000, false);

	uart_init(UART_ID, 2400);
	
	gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
	
	int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);
	uart_set_hw_flow(UART_ID, false, false);
	uart_set_fifo_enabled(UART_ID, false);
	int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
	irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

	uart_set_irq_enables(UART_ID, true, false);


    if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;
    }


	btstack_main();
/*
    while (1)
        tight_loop_contents();
*/
    btstack_run_loop_execute();
	return 0;
}