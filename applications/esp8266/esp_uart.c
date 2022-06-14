#include "esp_uart.h"
#include "furi_hal.h"
#include <stream_buffer.h>

#define BUF_LEN 256

static const GpioPin* flow_pins[][2] = {
    {&gpio_ext_pa7, &gpio_ext_pa6}, // 2, 3
    {&gpio_ext_pb2, &gpio_ext_pc3}, // 6, 7
};

typedef enum {
    WorkerEvtStop = (1 << 0),
    WorkerEvtRxDone = (1 << 1),

    WorkerEvtTxStop = (1 << 2),
    WorkerEvtDataRx = (1 << 3),

    WorkerEvtCfgChange = (1 << 4),

    WorkerEvtLineCfgSet = (1 << 5),
    WorkerEvtCtrlLineSet = (1 << 6),

} WorkerEvtFlags;

#define WORKER_ALL_RX_EVENTS (WorkerEvtStop | WorkerEvtRxDone | WorkerEvtCfgChange | WorkerEvtLineCfgSet | WorkerEvtCtrlLineSet)
#define WORKER_ALL_TX_EVENTS (WorkerEvtTxStop | WorkerEvtCdcRx)

struct EspUart {
  EspUartConfig cfg;
  EspUartConfig cfg_new;

  FuriThread* thread;
  FuriThread* tx_thread;

  StreamBufferHandle_t rx_stream;

  osMutexId_t usb_mutex;

  osSemaphoreId_t tx_sem;

  EspUartState st;

  uint8_t rx_buf[BUF_LEN];
};

static int32_t esp_uart_tx_thread(void* context);

static void esp_uart_on_irq_cb(EspIrqEvent ev, uint8_t data, void* context) {
  // TODO
}

static void esp_uart_serial_init(EspUart* esp_uart, uint8_t uart_ch) {
  // TODO
}

static void esp_uart_serial_deinit(EspUart* esp_uart, uint8_t uart_ch) {
  // TODO
}

static void esp_uart_set_baudrate(EspUart* esp_uart, uint32_t baudrate) {
  // TODO
}

static uint32_t esp_uart_worker(void* context) {
  // TODO
}

static uint32_t esp_uart_tx_thread(void* context) {
  EspUart* esp_uart = (EspUart*)context;

  uint8_t data[BUF_LEN];
  while(1) {
    uint32_t events = osThreadFlagsWait(WORKER_ALL_TX_EVENTS, osFlagsWaitAny, osWaitForever);
    furi_check((events & osFlagsError) == 0);
    if(events & WorkerEvtTxStop) break;
    if(events & WorkerEvtDataRx) {
      // TODO: process/store data
    }
  }

  return 0;
}

EspUart* esp_uart_enable(EspUartConfig* cfg) {
  EspUart* esp_uart = malloc(sizeof(EspUart));

  memcpy(&(esp_uart->cfg_new), cfg, sizeof(EspUartConfig));

  esp_uart->thread = furi_thread_alloc();
  furi_thread_set_name(esp_uart->thread, "EspUartWorker");
  furi_thread_set_stack_size(esp_uart->thread, 1024);
  furi_thread_set_context(esp_uart->thread, esp_uart);
  furi_thread_set_callback(esp_uart->thread, esp_uart_worker);

  furi_thread_start(esp_uart->thread);
  return esp_uart;
}

void esp_uart_disable(EspUart* esp_uart) {
  furi_assert(esp_uart);

  osThreadFlagsSet(furi_thread_get_thread_id(esp_uart->thread), WorkerEvtStop);
  furi_thread_join(esp_uart->thread);
  furi_thread_free(esp_uart->thread);
  free(esp_uart);
}

void esp_uart_set_config(EspUart* esp_uart, EspUartConfig* cfg) {
  furi_assert(esp_uart);
  furi_assert(cfg);

  memcpy(&(esp_uart->cfg_new), cfg, sizeof(EspUartConfig));
  osThreadFlagsSet(furi_thread_get_thread_id(esp_uart->thread), WorkerEvtCfgChange);
}

void esp_uart_get_config(EspUart* esp_uart, EspUartConfig* cfg) {
  furi_assert(esp_uart);
  furi_assert(cfg);

  memcpy(cfg, &(esp_uart->cfg_new), sizeof(EspUartConfig));
}

void esp_uart_get_state(EspUart* esp_uart, EspUartState* st) {
  furi_assert(esp_uart);
  furi_assert(st);

  memcpy(st, &(esp_uart->st), sizeof(EspUartState));
}
