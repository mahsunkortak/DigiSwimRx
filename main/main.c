#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "./lora.h"           //LoRa Library
//#include "./u8g2_esp32_hal.h" //OLED Library

#include "driver/uart.h"//UART Library
#include "soc/uart_struct.h"
#include "string.h"

static const char *TAG = "main";

/************************ Ligacoes LoRa *************************/
#define SCK_PIN    5        // GPIO5  -- SX1278's SCK
#define MISO_PIN    19      // GPIO19 -- SX1278's MISnO
#define MOSI_PIN    27      // GPIO27 -- SX1278's MOSI
#define CS_PIN      18      // GPIO18 -- SX1278's CS
#define RST_PIN     12      // GPIO14 -- SX1278's RESET
#define DI0_PIN     26      // GPIO26 -- SX1278's IRQ(Interrupt Request)
#define LORA_FREQ  433E6    // Frequência -- 433 MHz

/************************ Ligacoes OLED *************************/
#define PIN_SDA 21          // GPIO 21 -- SDD1306's SDA
#define PIN_SCL 22          // GPIO 22 -- SDD1306's SCL

/************************ Ligacoes UART *************************/
#define TXD_PIN 1
#define RXD_PIN 3

#define RX_PACKET_SIZE 34
//mahsun 33 ten 34 cikardim



int version = 0;            //Variável com ID do LoRa
//u8g2_uint_t width = 0;
int horizontalCenter = 0;
int8_t buf[RX_PACKET_SIZE]; //Buffer onde é armazenada o packet do LoRa
int x = 0;
float snr = 0;
int rssi = 0;
char snrChar[9];
char rssiChar[13];
int bpm = 0;
int i = 0;
uint16_t packetID = 0;
uint16_t lastPacketID = 0;
int packetIDError = 0;
int firstIteration = 1;
short requestPacket[3]={0};
char stringPacket[6]={"Hello"};

static const int RX_BUF_SIZE = 1024;
int16_t uartTxData[30] = {0};
//mahsun 28 to 30
int8_t uartRxData[4];
uint8_t* uartRxDataPtr;
int uartRxBytes = 0;

uint8_t startComFlag = 0;
uint8_t loraPacketSended = 0;

int validReceivedDataFlag = 0;

const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

int sendUARTData(const char* data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(UART_NUM_1, data, len);
    return txBytes;
}

int waitingRequestedPacketsFlag = 0;
int firstResendedPacketFlag = 0;
int8_t auxByte = 0;

void app_main(void) {
    printf("MAIN\r\n");
    //Inicialização LoRa
    version = lora_init(CS_PIN, SCK_PIN, MISO_PIN, MOSI_PIN, RST_PIN);
    printf("LoRa ID: 0x%x\n", version);
    lora_set_frequency(LORA_FREQ);
    printf("LoRa Frequency: %.0f\n", LORA_FREQ);
    lora_enable_crc();
    printf("Deteção de erros: ACTIVA\n");
    /*printf("\nLoRa RegModemConfig1: 0x%x",lora_read_reg(0x1D));
    printf("\nLoRa RegModemConfig2: 0x%x",lora_read_reg(0x1E));
    printf("\nLoRa RegPreambleMSB: 0x%x",lora_read_reg(0x20));
    printf("\nLoRa RegPreambleLSB: 0x%x",lora_read_reg(0x21));
    printf("\nLoRa RegPayloadLength: 0x%x",lora_read_reg(0x22));
    printf("\nLoRa RegMaxPayloadLength: 0x%x",lora_read_reg(0x23));
    lora_write_reg(0x1D,0x92);
    printf("\nLoRa RegModemConfig1: 0x%x",lora_read_reg(0x1D));
    */
   printf("\nLoRa RegOpMOde: 0x%x", lora_read_reg(0x01));
   printf("\nLoRa Sync Word: 0x%x", lora_read_reg(0x39));
   lora_write_reg(0x39, 0x15);
   printf("\nLoRa Sync Word: 0x%x", lora_read_reg(0x39));
    //Configurações OLED
    /*u8g2_t u8g2;
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = PIN_SDA;
	u8g2_esp32_hal.scl  = PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);
    u8g2_Setup_ssd1306_i2c_128x64_noname_f(
		&u8g2,
		U8G2_R0,
		//u8x8_byte_sw_i2c,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);
	u8x8_SetI2CAddress(&u8g2.u8x8,0x78);

    //Inicialização OLED
    u8g2_InitDisplay(&u8g2);
    u8g2_SetPowerSave(&u8g2, 0);
    u8g2_ClearBuffer(&u8g2);

    width = u8g2_GetDisplayWidth(&u8g2);
    horizontalCenter = width/2;
    int heartPosition = horizontalCenter-15;

    //Escrever OLED
    u8g2_SetFontMode(&u8g2, 0);
    //u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
    u8g2_SetFont(&u8g2, u8g2_font_t0_14_tf);
    u8g2_DrawStr(&u8g2, 0,15,"Heart Receiver");
    u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
    u8g2_DrawGlyph(&u8g2, heartPosition, 30, 0x2661);
    u8g2_SendBuffer(&u8g2);*/

    //Inicializar UART
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    //printf("IS PRINTF WORKING?\n");
    //printf("IS PRINTF WORKING?\n");
    size_t*  bufferedDataSize=0;
    char bufferedDataSizeChar[2];
    char charArray[8];
    char comma[1]=",";
    int dummy = 0;

    while (true) {
        uartRxDataPtr = (uint8_t*) malloc(RX_BUF_SIZE+1);
        uartRxBytes = uart_read_bytes(UART_NUM_1, uartRxDataPtr, RX_BUF_SIZE, 70 / portTICK_RATE_MS);
        if (uartRxBytes > 0) {
            uartRxDataPtr[uartRxBytes] = 0;
            loraPacketSended=0;
            do{
                lora_send_packet((uint16_t*)uartRxDataPtr, uartRxBytes);
                //vTaskDelay(68 / portTICK_RATE_MS);
                lora_receive();
                while(lora_received()){
                    x = lora_receive_packet((uint8_t*)buf, sizeof(buf));
                    buf[x] = 0;
                    itoa(x,charArray,10);
                    if(buf[0]==49){
                        loraPacketSended=1;
                        startComFlag=1;
                        sendUARTData("1\n");
                    }else if(buf[0]==48){
                        loraPacketSended=1;
                        startComFlag=0;
                        sendUARTData("0\n");
                    }
                }
                vTaskDelay(150 / portTICK_RATE_MS);
            }while(loraPacketSended!=1);
        }
        free(uartRxDataPtr);
         
        lora_receive();
        while (lora_received() && startComFlag==1) {
            char comma[1]=",";
            x = lora_receive_packet((uint8_t*)buf, sizeof(buf));
            buf[x] = 0;
            
            //Imprime Dados recebidos por LoRa
            /*itoa(dummy++, charArray, 10);
            sendUARTData("\n");
            sendUARTData(charArray);
            sendUARTData(" Received Packet: ");
            for(i=0;i<sizeof(buf);i++){
                itoa(buf[i]&0xff,charArray,16);
                sendUARTData(charArray);
                sendUARTData(",");
                memset(charArray,'\0',sizeof(charArray));
            }*/
            
            if(waitingRequestedPacketsFlag == 0){
                lastPacketID = packetID;
                //packetID = buf[0];
                packetID = buf[0]<<8; 
                packetID = (packetID)|((((int16_t)buf[1])|0xFF00)^0xFF00);
                /*itoa(lastPacketID, charArray, 10);
                sendUARTData("\nlastPacketID: ");
                sendUARTData(charArray);
                itoa(packetID, charArray, 10);
                sendUARTData(" packetID: ");
                sendUARTData(charArray);*/
                if (firstIteration==1){
                    lastPacketID = packetID;
                    firstIteration = 0;
                }

                packetIDError = packetID-lastPacketID;
                if (packetIDError > 9){
                    lastPacketID = packetID;
                }

                if (packetIDError >= 2 && packetIDError <=9 && packetID > 0){
                    /*sendUARTData("\nPacket Lost: ");
                    sendUARTData(itoa(packetID, charArray, 10));
                    sendUARTData(" ");
                    sendUARTData(itoa(lastPacketID, charArray, 10));
                    sendUARTData(" ");
                    sendUARTData(itoa(packetIDError, charArray, 10));
                    sendUARTData("\n");*/
                    validReceivedDataFlag = 0;
                    requestPacket[0] = 2;
                    requestPacket[1] = lastPacketID+1;
                    for (i=1; i<packetIDError; i++){
                        requestPacket[i]=lastPacketID+(uint16_t)i; 
                    }
                    //sendUARTData("\nRequest Lost Packets from #ID: ");
                    //sendUARTData(itoa(requestPacket[1], charArray, 10));
                    /*for (i=1; i<packetIDError; i++){
                        sendUARTData(itoa(requestPacket[i], charArray, 10));
                        sendUARTData(",");
                    }*/
                    lora_send_packet((uint16_t*)requestPacket, 6);
                    //sendUARTData("\nRequest Sended");
                    x = lora_receive_packet((uint8_t*)buf, sizeof(buf));                 
                    waitingRequestedPacketsFlag = 1;
                    //lora_send_packet((uint16_t*)requestPacket, 22);
                    //printf("\nRequest Lost Packet: ");
                } else {
                    validReceivedDataFlag=1;
                }
            }else if (waitingRequestedPacketsFlag == 1){
                if(buf[RX_PACKET_SIZE-1] == -1){
                    firstResendedPacketFlag = 1;
                    validReceivedDataFlag = 1;
                    
                    //memset(buf,'\0',sizeof(buf));
                }else if ((buf[RX_PACKET_SIZE-1] != -1) && (firstResendedPacketFlag == 1)){
                    waitingRequestedPacketsFlag = 0;
                    validReceivedDataFlag = 1;
                    firstIteration = 1;
                    //sendUARTData("\nAcabou Burst\n");
                }
                /*if(buf[0] == (lastPacketID-1)){
                    waitingRequestedPacketsFlag = 0;
                    lastPacketID = lastPacketID + packetIDError;
                } */
            }
           /* printf("\nuartData:\n");
            for(i=0;i<28;i++){
                printf("%d,",uartData[i]);
            }*/
            //printf("\n\nPacket Size: %d",x);

            
            //lora_receive();
        }
        if (validReceivedDataFlag == 1){
            snr = lora_packet_snr();
            rssi = lora_packet_rssi();
            uartTxData[0]=snr*100;
            uartTxData[1]=rssi;
            if(waitingRequestedPacketsFlag == 1){
                x = x-1;
            }

            //uartTxData[2] = (((int16_t)buf[0])<<8)|buf[1];          //packetID
            uartTxData[2] = buf[0];
            uartTxData[2] = (uartTxData[2]<<8)|((((int16_t)buf[1])|0xFF00)^0xFF00);
            uartTxData[3] = buf[2];                    //senderID    
            uartTxData[4] = buf[3];                    //BPM Kyto
            
            //AX1, AX2, AX3, AX4
            uartTxData[5] = ((buf[4]>>2)|0xFFC0)^0xFFC0;                    //AX1
            uartTxData[6] = (buf[5] < 0) ? (buf[5]>>4)^0xF0 : buf[5]>>4;    //AX2
            uartTxData[6] = (((buf[4]<<4)|uartTxData[6])|0xFFC0)^0xFFC0;
            uartTxData[7] = (buf[6] < 0) ? (buf[6]>>6)^0xFC : buf[6]>>6;    //AX3
            uartTxData[7] = (((buf[5]<<2)|uartTxData[7])|0xFFC0)^0xFFC0;
            uartTxData[8] = (((buf[6]<<2)>>2)|0xFFC0)^0xFFC0;               //AX4

            //AY1, AY2, AY3, AY4
            uartTxData[9] = ((buf[7]>>2)|0xFFC0)^0xFFC0;                     //AY1
            uartTxData[10] = (buf[8] < 0) ? (buf[8]>>4)^0xF0 : buf[8]>>4;    //AY2
            uartTxData[10] = (((buf[7]<<4)|uartTxData[10])|0xFFC0)^0xFFC0;
            uartTxData[11] = (buf[9] < 0) ? (buf[9]>>6)^0xFC : buf[9]>>6;    //AY3
            uartTxData[11] = (((buf[8]<<2)|uartTxData[11])|0xFFC0)^0xFFC0;
            uartTxData[12] = (((buf[9]<<2)>>2)|0xFFC0)^0xFFC0;               //AY4

            //AZ1, AZ2, AZ3, AZ4
            uartTxData[13] = ((buf[10]>>2)|0xFFC0)^0xFFC0;                    //AZ1
            uartTxData[14] = (buf[11] < 0) ? (buf[11]>>4)^0xF0 : buf[11]>>4;  //AZ2
            uartTxData[14] = (((buf[10]<<4)|uartTxData[14])|0xFFC0)^0xFFC0;
            uartTxData[15] = (buf[12] < 0) ? (buf[12]>>6)^0xFC : buf[12]>>6;  //AZ3
            uartTxData[15] = (((buf[11]<<2)|uartTxData[15])|0xFFC0)^0xFFC0;
            uartTxData[16] = (((buf[12]<<2)>>2)|0xFFC0)^0xFFC0;               //AZ4

            for(i=5; i<17; i++){
                if(uartTxData[i]>0x1F){
                    uartTxData[i] = -(uartTxData[i]^0x20);
                }
            }
            //GX1, GX2, GX3, GX4
            uartTxData[17] = buf[13];
            uartTxData[17] = uartTxData[17]<<2;
            auxByte = (buf[14] < 0) ? (buf[14]>>6)^0xFC : buf[14]>>6; 
            uartTxData[17] = ((uartTxData[17]|auxByte)|0xFC00)^0xFC00;
            uartTxData[18] = buf[14];
            uartTxData[18] = uartTxData[18]<<4;
            auxByte = (buf[15] < 0) ? (buf[15]>>4)^0xF0 : buf[15]>>4; 
            uartTxData[18] = ((uartTxData[18]|auxByte)|0xFC00)^0xFC00;
            uartTxData[19] = buf[15];
            uartTxData[19] = uartTxData[19]<<6;
            auxByte = (buf[16] < 0) ? (buf[16]>>2)^0xC0 : buf[16]>>2; 
            uartTxData[19] = ((uartTxData[19]|auxByte)|0xFC00)^0xFC00;
            uartTxData[20] = buf[16];
            uartTxData[20] = uartTxData[20]<<8;
            uartTxData[20] = uartTxData[20]|((((int16_t)buf[17])|0xFF00)^0xFF00);
            uartTxData[20] = (uartTxData[20]|0xFC00)^0xFC00;

            //GY1, GY2, GY3, GY4
            uartTxData[21] = buf[18];
            uartTxData[21] = uartTxData[21]<<2;
            auxByte = (buf[19] < 0) ? (buf[19]>>6)^0xFC : buf[19]>>6; 
            uartTxData[21] = ((uartTxData[21]|auxByte)|0xFC00)^0xFC00;
            uartTxData[22] = buf[19];
            uartTxData[22] = uartTxData[22]<<4;
            auxByte = (buf[20] < 0) ? (buf[20]>>4)^0xF0 : buf[20]>>4; 
            uartTxData[22] = ((uartTxData[22]|auxByte)|0xFC00)^0xFC00;
            uartTxData[23] = buf[20];
            uartTxData[23] = uartTxData[23]<<6;
            auxByte = (buf[21] < 0) ? (buf[21]>>2)^0xC0 : buf[21]>>2; 
            uartTxData[23] = ((uartTxData[23]|auxByte)|0xFC00)^0xFC00;
            uartTxData[24] = buf[21];
            uartTxData[24] = uartTxData[24]<<8;
            uartTxData[24] = uartTxData[24]|((((int16_t)buf[22])|0xFF00)^0xFF00);
            uartTxData[24] = (uartTxData[24]|0xFC00)^0xFC00;

            //GZ1, GZ2, GZ3, GZ4
            uartTxData[25] = buf[23];
            uartTxData[25] = uartTxData[25]<<2;
            auxByte = (buf[24] < 0) ? (buf[24]>>6)^0xFC : buf[24]>>6; 
            uartTxData[25] = ((uartTxData[25]|auxByte)|0xFC00)^0xFC00;
            uartTxData[26] = buf[24];
            uartTxData[26] = uartTxData[26]<<4;
            auxByte = (buf[25] < 0) ? (buf[25]>>4)^0xF0 : buf[25]>>4; 
            uartTxData[26] = ((uartTxData[26]|auxByte)|0xFC00)^0xFC00;
            uartTxData[27] = buf[25];
            uartTxData[27] = uartTxData[27]<<6;
            auxByte = (buf[26] < 0) ? (buf[26]>>2)^0xC0 : buf[26]>>2; 
            uartTxData[27] = ((uartTxData[27]|auxByte)|0xFC00)^0xFC00;
            uartTxData[28] = buf[26];
            uartTxData[28] = uartTxData[28]<<8;
            uartTxData[28] = uartTxData[28]|((((int16_t)buf[27])|0xFF00)^0xFF00);
            uartTxData[28] = (uartTxData[28]|0xFC00)^0xFC00;

            for(i=17; i<29; i++){
                if(uartTxData[i]>0x1FF){
                    uartTxData[i] = -(uartTxData[i]^0x200);
                }
            }

            uartTxData[29] = buf[28]; //BPM MAX30102
            //uartTxData[30] = buf[29]; //SPO2 MAX30102
            //uartTxData[30] = (uartTxData[30]<<8)|((((int16_t)buf[30])|0xFF00)^0xFF00); //SPO2 MAX30102
            uartTxData[30] = buf[29]*100 + buf[30];
            uartTxData[31] = buf[31]; //Battery Voltage
            uartTxData[32] = buf[32]; //xcor
            uartTxData[33] = buf[33]; //ycor
            //sendUARTData("\nValid Packet: ");
            /************* Transmissão de Dados para a GUI **********/
            for(i=0;i<sizeof(buf)/sizeof(uint8_t);i++)
            { //mahsun i=0;i<sizeof(buf);i++ -> i=0;i<sizeof(buf)/sizeof(uint8_t);i++
                itoa(uartTxData[i],charArray,10);
                sendUARTData(charArray);
                sendUARTData(",");
                memset(charArray,'\0',sizeof(charArray));
            }
            sendUARTData("\n");
            /********************************************************/
            memset(buf,'\0',sizeof(buf));
            bpm = buf[0];
        //sprintf(snrChar, "SNR: %.2f", snr);
            sprintf(rssiChar, "RSSI: %d", rssi);
            /*u8g2_ClearBuffer(&u8g2);
            u8g2_SetFont(&u8g2, u8g2_font_t0_13_tf);
            u8g2_DrawStr(&u8g2, 0,15,"Heart Receiver");
            u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
            u8g2_DrawGlyph(&u8g2, heartPosition, 30, 0x2661);
            u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
            u8g2_DrawGlyph(&u8g2, heartPosition, 30, 0x2665);
            u8g2_DrawStr(&u8g2, 0, 45, &snrChar);
            u8g2_DrawStr(&u8g2, 0, 60, &rssiChar);
            u8g2_SendBuffer(&u8g2);*/
        }
        validReceivedDataFlag = 0;
        /*u8g2_SetFont(&u8g2, u8g2_font_unifont_t_symbols);
        u8g2_DrawGlyph(&u8g2, heartPosition, 30, 0x2661);
        u8g2_SendBuffer(&u8g2);*/
        memset(buf,'\0',sizeof(buf));
    }
}
