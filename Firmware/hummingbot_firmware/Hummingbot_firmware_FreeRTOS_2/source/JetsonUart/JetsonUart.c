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
#define LPUART0_CLOCK                   (CLOCK_GetFreq(kCLOCK_ScgSysPllAsyncDiv2Clk))
#define LPUART1_CLOCK                   (CLOCK_GetFreq(kCLOCK_ScgSysPllAsyncDiv2Clk))
/*******************************************************************************
 * typedef
 ******************************************************************************/
typedef struct
{
  lpuart_handle_t lpuart1_handle;
  lpuart_handle_t lpuart0_handle;
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
  size_t  receivedBytes;
  uint8_t synced;
  uint8_t prev_synced;
  uint8_t synced_2_bytes;
  uint8_t sync_bytes[1];
}jetson_uart_data_S;

/*******************************************************************************
 * private variables
 ******************************************************************************/
static jetson_uart_data_S m_ju;

/*******************************************************************************
 * private function prototypes
 ******************************************************************************/
static void lpuart1_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);
static void lpuart0_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData);

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
  LPUART_GetDefaultConfig(&m_ju.config);
  m_ju.config.baudRate_Bps= 115200U;
  m_ju.config.enableTx    = true;
  m_ju.config.enableRx    = true;

  // TODO: optimize clock frequency
  //NOTE: clock frequency needs to match the clock register
  LPUART_Init(LPUART1, &m_ju.config, LPUART1_CLOCK);
  LPUART_TransferCreateHandle(LPUART1, &m_ju.lpuart1_handle, lpuart1_callback, NULL);
  LPUART_Init(LPUART0, &m_ju.config, LPUART0_CLOCK);
  LPUART_TransferCreateHandle(LPUART0, &m_ju.lpuart0_handle, lpuart0_callback, NULL);
}

void JU_begin(void)
{
  LPUART_TransferStartRingBuffer(LPUART1, &m_ju.lpuart1_handle, m_ju.rxRingBuffer, 20U);
}

void JU_prepSync(void)
{
  //Temporarily accommodate the receive uart struct for receiving a single byte
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
      LPUART_TransferReceiveNonBlocking(LPUART1, &m_ju.lpuart1_handle, &m_ju.receiveXfer, &m_ju.receivedBytes);

      //save synced boolean to be the previous
      m_ju.prev_synced = m_ju.synced;
      if (m_ju.sync_bytes[0] == 255U)
      {
        //Successfully synced if both previous and current byte are 255
        if(m_ju.prev_synced)
        {
          m_ju.synced_2_bytes = 1;
        }
        m_ju.synced = 1;
      }
      else
      {
        m_ju.synced = 0U;
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

void JU_doXfer(void)
{
  /* If RX is idle and rxBuffer is empty, start to read data to rxBuffer. */
  if ((!m_ju.rxOnGoing) && m_ju.rxBufferEmpty)
  {
    m_ju.rxOnGoing = true;
    LPUART_TransferReceiveNonBlocking(LPUART1, & m_ju.lpuart1_handle, & m_ju.receiveXfer, & m_ju.receivedBytes);
    if (JETSON_UART_TX_BUFFER_SIZE ==  m_ju.receivedBytes)
    {
      m_ju.rxBufferEmpty = false;
      m_ju.rxOnGoing = false;
    }
  }

  /* If TX is idle and txBuffer is full, start to send data. */
  if ((!m_ju.txOnGoing) && m_ju.txBufferFull)
  {
    m_ju.txOnGoing = true;
    LPUART_TransferSendNonBlocking(LPUART0, &m_ju.lpuart0_handle, &m_ju.sendXfer);
  }

  /* If txBuffer is empty and rxBuffer is full, copy rxBuffer to txBuffer. */
  if ((!m_ju.rxBufferEmpty) && (!m_ju.txBufferFull))
  {
    memcpy(&m_ju.txBuffer, &m_ju.rxBuffer, sizeof(m_ju.txBuffer));

    m_ju.rxBufferEmpty = true;
    m_ju.txBufferFull = true;
  }
}
/*******************************************************************************
 * private function
 ******************************************************************************/
static void lpuart1_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
  if(status == kStatus_LPUART_RxIdle) {
    m_ju.rxBufferEmpty = false;
    m_ju.rxOnGoing = false;
  }
}

static void lpuart0_callback(LPUART_Type *base, lpuart_handle_t *handle, status_t status, void *userData)
{
  if(status == kStatus_LPUART_TxIdle){
    m_ju.txBufferFull = false;
    m_ju.txOnGoing = false;
  }
}




