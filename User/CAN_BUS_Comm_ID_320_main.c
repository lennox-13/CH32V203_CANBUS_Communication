/*
 *Autor: lennox-13
 *CAN Normal mode, standard frame and expanded frame data transceiver
 *CAN_Tx(PB9),CAN_Rx(PB8)
 *Requires an external CAN transceiver in my case TJA1050
 */

#include "debug.h"

#define TIMEOUT_LIMIT 0xFFF          // Timeout limit for CAN transmission
#define CAN_STANDARD_ID 0x317        // Standard frame ID
#define CAN_EXTENDED_ID_HIGH 0x9092  // Extended frame high ID
#define CAN_EXTENDED_ID_LOW 0x2B3C   // Extended frame low ID
#define FRAME_FORMAT_STANDARD 0      // Standard frame format
#define FRAME_FORMAT_EXTENDED 1      // Extended frame format

/*********************************************************************
 * @fn      CAN_Mode_Init
 *
 * @brief   Initializes CAN communication 333kbs.
 *          Bps = Fpclk1/((tpb1+1+tbs2+1+1)*brp)
 *
 * @param   tsjw - CAN synchronisation jump width.
 *          tbs2 - CAN time quantum in bit segment 1.
 *          tbs1 - CAN time quantum in bit segment 2.
 *          brp - Specifies the length of a time quantum.
 *          mode - Test mode.
 *          frame_format - Standard or Extended frame format.
 *
 * @return  none
 */
void CAN_Mode_Init(u8 tsjw, u8 tbs2, u8 tbs1, u16 brp, u8 mode, u8 frame_format){
    GPIO_InitTypeDef      GPIO_InitStructure = {0};
    CAN_InitTypeDef       CAN_InitStructure = {0};
    CAN_FilterInitTypeDef CAN_FilterInitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = DISABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = ENABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = mode;
    CAN_InitStructure.CAN_SJW = tsjw;
    CAN_InitStructure.CAN_BS1 = tbs1;
    CAN_InitStructure.CAN_BS2 = tbs2;
    CAN_InitStructure.CAN_Prescaler = brp;
    CAN_Init(CAN1, &CAN_InitStructure);

    CAN_FilterInitStructure.CAN_FilterNumber = 0;

    if (frame_format == FRAME_FORMAT_STANDARD) {  // Standard Frame
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
        CAN_FilterInitStructure.CAN_FilterIdHigh = (CAN_STANDARD_ID << 5);  // Receives only with ID CAN_STANDARD_ID
        CAN_FilterInitStructure.CAN_FilterIdLow = 0;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFE0;
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0006;
    } else {  // Extended Frame
        CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
        CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
        CAN_FilterInitStructure.CAN_FilterIdHigh = CAN_EXTENDED_ID_HIGH;
        CAN_FilterInitStructure.CAN_FilterIdLow = CAN_EXTENDED_ID_LOW;
        CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0xFFFF;
        CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0xFFFE;
    }

    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);
}

/*********************************************************************
 * @fn      CAN_Send_Msg
 *
 * @brief   CAN Transmit function.
 *
 * @param   msg - Transmit data buffer.
 *          len - Data length.
 *          id - Identifier for the CAN message.
 *
 * @return  0 - Send successful.
 *          1 - Send failed.
 */
u8 CAN_Send_Msg(u8 *msg, u8 len, u16 id){
    u8  mbox;
    u16 i = 0;

    CanTxMsg CanTxStructure;

    CanTxStructure.StdId = id;  // Identifier
    CanTxStructure.IDE = CAN_Id_Standard;
    CanTxStructure.RTR = CAN_RTR_Data;
    CanTxStructure.DLC = len;

    for (i = 0; i < len; i++) {
        CanTxStructure.Data[i] = msg[i];
    }

    mbox = CAN_Transmit(CAN1, &CanTxStructure);
    i = 0;
    while ((CAN_TransmitStatus(CAN1, mbox) != CAN_TxStatus_Ok) && (i < TIMEOUT_LIMIT)) {
        i++;
    }
    return (i == TIMEOUT_LIMIT) ? 1 : 0;
}

/*********************************************************************
 * @fn      CAN_Receive_Msg
 *
 * @brief   CAN Receive function.
 *
 * @param   buf - Receive data buffer.
 *          id - Pointer to store the received message ID.
 *
 * @return  Received data length.
 */
u8 CAN_Receive_Msg(u8 *buf, u16 *id){
    CanRxMsg CanRxStructure;

    if (CAN_MessagePending(CAN1, CAN_FIFO0) == 0) {
        return 0;
    }

    CAN_Receive(CAN1, CAN_FIFO0, &CanRxStructure);

    *id = CanRxStructure.StdId;  // Store received message ID
    u8 len = CanRxStructure.DLC;
    for (u8 i = 0; i < len; i++) {
        buf[i] = CanRxStructure.Data[i];
    }

    return len;
}

/*********************************************************************
 * @fn      main
 */
int main(void){
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);

    u8 txbuf[8];  // Buffer for transmitted data
    u8 rxbuf[8];  // Buffer for received data
    u16 rx_id;    // Variable to hold the received message ID
    u8 cnt = 0;

    // CANBUS Speed 333Kbps
    CAN_Mode_Init(CAN_SJW_1tq, CAN_BS2_5tq, CAN_BS1_6tq, 12, CAN_Mode_Normal, FRAME_FORMAT_STANDARD);

    while (1) {
        // Test data to send
        ++cnt;
        txbuf[0] = cnt;
        txbuf[1] = 100 - cnt;
        txbuf[2] = 2 + cnt;
        txbuf[3] = 99;
        txbuf[4] = 120 + cnt;
        txbuf[5] = 13;
        txbuf[6] = 99 - cnt;
        txbuf[7] = 77 + cnt;

        // Send CAN message with custom ID 0x320
        if (CAN_Send_Msg(txbuf, 8, 0x320) == 1) {
            printf("Error: CAN_BUS failed!\r\n");
        }

        Delay_Ms(1000);

         // Check for received CAN message
        u8 len = CAN_Receive_Msg(rxbuf, &rx_id);
        if (len > 0) {
            printf("Received Data with ID: 0x%X\r\n", rx_id);
            for (u8 i = 0; i < len; i++) {
                printf("%d ", rxbuf[i]);
            }
            printf("\r\n");
        }
    }
}
