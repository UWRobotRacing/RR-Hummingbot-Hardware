#include "common.h"
#include "JetsonUart.h"
#include "fsl_lpuart.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* FTM Hardware Config */
// The Flextimer instance/channel used for board 

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define RX_RING_BUFFER_SIZE             (20U)
#define JETSON_UART_TX_BUFFER_SIZE      (sizeof(jetson_buf_t))
#define JETSON_UART_RX_BUFFER_SIZE      (sizeof(jetson_buf_t))

#define JETSON_LPUART                   (LPUART1)
#define JETSON_LPUART_CLOCK             (kCLOCK_ScgSysPllAsyncDiv2Clk)
#define JETSON_LPUART_FREQ              (CLOCK_GetFreq(JETSON_LPUART_CLOCK))

#define JETSON_DATA_BUFFER_SIZE         (2U)
/*******************************************************************************
 * typedef
 ******************************************************************************/
typedef struct
{
  lpuart_handle_t lpuart_handle;
  lpuart_config_t config;
  uint8_t         rxRingBuffer[RX_RING_BUFFER_SIZE]; /* RX ring buffer. */
  char            txBuffer[JETSON_UART_TX_BUFFER_SIZE];
  char            rxBuffer[JETSON_UART_RX_BUFFER_SIZE];
  bool            rxBufferEmpty;
  bool            txBufferFull;
  bool            txOnGoing;
  bool            rxOnGoing;

  // For synchronization to be successful, synced and prev_synced must be true
  lpuart_transfer_t sendXfer;
  lpuart_transfer_t receiveXfer;

  //jetson buffer
  jetson_buf_t      doubleBuffer[JETSON_DATA_BUFFER_SIZE];
  jetson_buf_t*     read_ptr;
  jetson_buf_t*     write_ptr;
  bool              double_buffer_lock;
  bool              isNewMessageAvail;

  size_t  receivedBytes;
  bool synced;
  bool prev_synced;
  bool synced_2_bytes;
  uint8_t sync_bytes[1];
}jetson_uart_data_S;

/*******************************************************************************
 * private variables
 ******************************************************************************/
static jetson_uart_data_S m_ju;

/*******************************************************************************
 * private function prototypes
 ******************************************************************************/
static void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * public function
 ******************************************************************************/
void JU_init(void)
{
  // init
  memset(&m_ju, 0x00, sizeof(m_ju));
  m_ju.rxBufferEmpty = true;
  m_ju.txBufferFull = false;
  m_ju.txOnGoing = false;
  m_ju.rxOnGoing = false;

  /* UART Configuration */
  /* -- DEFAULT --
  * config.baudRate_Bps = 115200U;
  * config.parityMode = kLPUART_ParityDisabled;
  * config.stopBitCount = kLPUART_OneStopBit;
  * config.txFifoWatermark = 0;
  * config.rxFifoWatermark = 0;
  * config.enableTx = false;
  * config.enableRx = false;
  */
  LPUART_GetDefaultConfig(&m_ju.config);
  m_ju.config.baudRate_Bps= 115200U;
  m_ju.config.enableTx    = true;
  m_ju.config.enableRx    = true;
  
  // setup double buffer ptr
  m_ju.read_ptr = &(m_ju.doubleBuffer[0]);
  m_ju.write_ptr = &(m_ju.doubleBuffer[1]);

  // TODO: optimize clock frequency
  //NOTE: clock frequency needs to match the clock register
  LPUART_Init(JETSON_LPUART, &m_ju.config, JETSON_LPUART_FREQ);
  LPUART_TransferCreateHandle(JETSON_LPUART, &m_ju.lpuart_handle, LPUART_UserCallback, NULL);
}

void JU_begin(void)
{
  LPUART_TransferStartRingBuffer(JETSON_LPUART, &m_ju.lpuart_handle, m_ju.rxRingBuffer, RX_RING_BUFFER_SIZE);
}

void JU_prepSync(void)
{
  //Temporarily accommodate the receive uart struct for receiving a single byte
  m_ju.sendXfer.data = NULL;
  m_ju.sendXfer.dataSize = 0;
  m_ju.receiveXfer.data = (uint8_t*) m_ju.sync_bytes;
  m_ju.receiveXfer.dataSize = sizeof(m_ju.sync_bytes);
}

bool JU_isSynced(void)
{
  return (m_ju.synced_2_bytes);
}

bool JU_trySync(void)
{
  if(!m_ju.synced_2_bytes){
    //read Byte only if there is no rx transfer occurring
    if(!m_ju.rxOnGoing)
    {
      m_ju.rxOnGoing = true;
      LPUART_TransferReceiveNonBlocking(JETSON_LPUART, &m_ju.lpuart_handle, &m_ju.receiveXfer, &m_ju.receivedBytes);

      //save synced boolean to be the previous
      m_ju.prev_synced = m_ju.synced;
      if (m_ju.sync_bytes[0] == 255U)
      {
        //Successfully synced if both previous and current byte are 255
        if(m_ju.prev_synced)
        {
          m_ju.synced_2_bytes = true;
        }
        m_ju.synced = true;
      }
      else
      {
        m_ju.synced = false;
      }
    }
  }
  return (m_ju.synced_2_bytes);
}

void JU_prepXfer(void)
{
  //initialize receive and send uart structs to be the size of the struct being sent between jetson and m4
  m_ju.sendXfer.data = (uint8_t*) m_ju.txBuffer;
  m_ju.sendXfer.dataSize = JETSON_UART_TX_BUFFER_SIZE;
  m_ju.receiveXfer.data = (uint8_t*) m_ju.rxBuffer;
  m_ju.receiveXfer.dataSize = JETSON_UART_RX_BUFFER_SIZE;
}

bool JU_readXfer(jetson_buf_t* readPtr)
{
  bool ret = m_ju.isNewMessageAvail;
  if(! m_ju.double_buffer_lock)
  {
    m_ju.double_buffer_lock = true;
    m_ju.isNewMessageAvail = false;
    memcpy(m_ju.read_ptr, readPtr, sizeof(JETSON_UART_RX_BUFFER_SIZE));
    m_ju.double_buffer_lock = false;
  }
  return (ret);
}

// TODO: remove this after testing, test for sync barriers
jetson_buf_t temp={
    .jetson_ang = 19,
    .jetson_spd = 22,
    .jetson_flag = 99,
    .jetson_pad = 0,
};

void JU_doXfer(void)
{
#if (JETSON_ENABLE_RECEIVER_ONLY_MODE)
  jetson_buf_t* tempPtr;
  /* If RX is idle and rxBuffer is empty, start to read data to rxBuffer. */
  if ((!m_ju.rxOnGoing) && m_ju.rxBufferEmpty)
  {
    m_ju.rxOnGoing = true;
    LPUART_TransferReceiveNonBlocking(JETSON_LPUART, & m_ju.lpuart_handle, & m_ju.receiveXfer, & m_ju.receivedBytes);
    if (JETSON_UART_TX_BUFFER_SIZE ==  m_ju.receivedBytes)
    {
      m_ju.rxBufferEmpty = false;
      m_ju.rxOnGoing = false;
    }
  }
  /* Use double buffer to circulate the message to ensure uncorrupt data read all the time*/
  if(!m_ju.rxBufferEmpty)
  {
    memcpy(m_ju.write_ptr, &(m_ju.rxBuffer), sizeof(JETSON_UART_RX_BUFFER_SIZE));
    // circuling buffer pointer
    if(! m_ju.double_buffer_lock)
    {
      m_ju.double_buffer_lock = true;
      tempPtr = m_ju.write_ptr;
      m_ju.write_ptr = m_ju.read_ptr;
      m_ju.read_ptr = tempPtr;
      m_ju.isNewMessageAvail = true;
      m_ju.double_buffer_lock = false;
    }
    m_ju.rxBufferEmpty = true;
  }

#if (JETSON_ENABLE_ECHO_MODE)
  /* If TX is idle and new message is available, start to send data. */
  if ((!m_ju.txOnGoing) && m_ju.isNewMessageAvail)
  {
    m_ju.txOnGoing = true;
//    memcpy(m_ju.txBuffer, &(m_ju.read_ptr), sizeof(JETSON_UART_RX_BUFFER_SIZE));
    memcpy(m_ju.txBuffer, &(temp), sizeof(JETSON_UART_RX_BUFFER_SIZE));
    LPUART_TransferSendNonBlocking(JETSON_LPUART, &m_ju.lpuart_handle, &m_ju.sendXfer);
  }
#endif //(JETSON_ENABLE_ECHO_MODE)
#endif //(JETSON_ENABLE_RECEIVER_ONLY_MODE)
}

#if (JETSON_UART_H_TEST_CASE)

#if (JETSON_UART_TEST_STRUCT)
#define ECHO_BUFFER_LENGTH (JETSON_UART_TX_BUFFER_SIZE)
#else
#define ECHO_BUFFER_LENGTH (8U)
uint8_t test_txBuffer[ECHO_BUFFER_LENGTH] = {0};
uint8_t test_rxBuffer[ECHO_BUFFER_LENGTH] = {0};
#endif //(JETSON_UART_TEST_STRUCT)

void JU_prepTesfer(void)
{
  /* Send g_tipString out. */
  // m_ju.sendXfer.data = g_tipString;
  // m_ju.sendXfer.dataSize = sizeof(g_tipString) - 1;
  // m_ju.rxOnGoing = true;
  // LPUART_TransferSendNonBlocking(JETSON_LPUART, & m_ju.lpuart_handle, &m_ju.sendXfer);
  //initialize receive and send uart structs to be the size of the struct being sent between jetson and m4
#if (JETSON_UART_TEST_STRUCT)
  m_ju.sendXfer.data = (uint8_t*) m_ju.txBuffer;
  m_ju.sendXfer.dataSize = ECHO_BUFFER_LENGTH;
  m_ju.receiveXfer.data = (uint8_t*) m_ju.rxBuffer;
  m_ju.receiveXfer.dataSize = ECHO_BUFFER_LENGTH;
#else
  m_ju.sendXfer.data = test_txBuffer;
  m_ju.sendXfer.dataSize = ECHO_BUFFER_LENGTH;
  m_ju.receiveXfer.data = test_rxBuffer;
  m_ju.receiveXfer.dataSize = ECHO_BUFFER_LENGTH;
#endif //(JETSON_UART_TEST_STRUCT)
  /* Wait send finished */
  // while (m_ju.rxOnGoing)
  // {
  // }
}

void JU_doTesfer(void)
{
  /* If RX is idle and rxBuffer is empty, start to read data to rxBuffer. */
  if ((!m_ju.rxOnGoing) && m_ju.rxBufferEmpty)
  {
    m_ju.rxOnGoing = true;
    LPUART_TransferReceiveNonBlocking(JETSON_LPUART, & m_ju.lpuart_handle, & m_ju.receiveXfer, & m_ju.receivedBytes);
    if (JETSON_UART_TX_BUFFER_SIZE ==  m_ju.receivedBytes)
    {
      m_ju.rxBufferEmpty = false;
      m_ju.rxOnGoing = false;
    }
  }

  /* If TX is idle and txBuffer is full, start to send data. */
  if ((!m_ju.txOnGoing) && (m_ju.txBufferFull))
  {
    m_ju.txOnGoing = true;
    LPUART_TransferSendNonBlocking(JETSON_LPUART, &m_ju.lpuart_handle, &m_ju.sendXfer);
  }

  /* If txBuffer is empty and rxBuffer is full, copy rxBuffer to txBuffer. */
  if ((!m_ju.rxBufferEmpty) && (!m_ju.txBufferFull))
  {
    // memcpy(&m_ju.txBuffer, &m_ju.rxBuffer, sizeof(m_ju.txBuffer));
    memcpy(m_ju.sendXfer.data,  m_ju.receiveXfer.data, ECHO_BUFFER_LENGTH);
    m_ju.rxBufferEmpty = true;
    m_ju.txBufferFull = true;
  }
}
#endif //(JETSON_UART_H_TEST_CASE)
/*******************************************************************************
 * private function
 ******************************************************************************/
/* LPUART user callback */
void LPUART_UserCallback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_LPUART_TxIdle == status)
    {
      m_ju.txBufferFull = false;
      m_ju.txOnGoing = false;
    }

    if (kStatus_LPUART_RxIdle == status)
    {
      m_ju.rxBufferEmpty = false;
      m_ju.rxOnGoing = false;
    }
}

