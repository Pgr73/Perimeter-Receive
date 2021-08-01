#include <Arduino.h>
#include <math.h>

//----------------------------
// Common Defines
//----------------------------

//#define SERIAL_PLOTTER
#define TIMER_PRESCALER 80

//----------------------------
// Perimeter function
//----------------------------
#define PERIMETER_PROCESSING_TASK_ESP_CORE 1
#define TIMER_PERIMETER_PERIOD 500 * 1000 // in microseconds
#define TIMER_PERIMETER_NUMBER 1
#define PERIMETER_QUEUE_LEN 5

#define PERIMETER_TRACE

//SemaphoreHandle_t g_PerimeterProcTimerSemaphore;
hw_timer_t *g_PerimeterTimerhandle = NULL;
portMUX_TYPE g_PerimeterTimerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t g_PerimeterProcTaskHandle;
QueueHandle_t g_PerimeterTimerQueue;
volatile int g_PerimeterQueuefull = 0;

volatile uint16_t g_PerimeterRawMax = 0;
volatile uint16_t g_PerimeterRawMin = INT_MAX;
volatile uint16_t g_PerimeterRawAvg = 0;
volatile bool g_isInsidePerimeter = false;
volatile bool g_PerimetersignalTimedOut = false;

volatile int g_PerimeterMagnitude = 0;
volatile int g_PerimeterSmoothMagnitude = 0;
volatile float g_PerimeterFilterQuality = 0;


#ifdef PERIMETER_TRACE
volatile long g_PerimeterinQueue = 0;
volatile int g_PerimeterinQueueMax = 0;
volatile unsigned long g_PerimeterTimerCount = 0;
#endif

//----------------------------
// Fast Ana Read function
//----------------------------
#include <driver/adc.h>
#include <driver/i2s.h>

//#define TIMER_FAST_NUMBER 0
//#define TIMER_FAST_FREQUENCY 2000 // in microseconds

#define SAMPLE_RATE 38400       // I2S scanning rate in samplees per second
//#define SAMPLE_RATE 38462

#define ADC_CHANNEL ADC1_CHANNEL_5 // gpio 33
//#define ADC_CHANNEL ADC1_CHANNEL_3  // gpio 39

#define I2S_READ_TIMEOUT 1 // tics 80 = 1 microsecs
//#define I2S_READ_TIMEOUT 80*1 // tics 80 = 1 microsecs
//#define I2S_READ_TIMEOUT portMAX_DELAY

#define BYTES_NEEDED 2
#define I2S_SAMPLES 256
#define DMA_BUF_LEN 24 * 4 * 2 //24 bits code length * 4 time oversampling (ardumower) * 2 Times oversampling => 192 samples
#define RAW_SAMPLES DMA_BUF_LEN * 10
#define ANA_READ_TASK_ESP_CORE 1

#define FAST_ANALOG_TRACE

//SemaphoreHandle_t g_FastTimerSemaphore;
hw_timer_t *g_FastTimerhandle = NULL;
portMUX_TYPE g_FastTimerMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t g_FastAnaReadTaskHandle;
QueueHandle_t g_I2SQueueHandle;
SemaphoreHandle_t g_ADCinUse;
SemaphoreHandle_t g_RawValuesSemaphore;

volatile int g_I2SSamples = 24 * 4 * 2;
//volatile int g_I2SSamples = 10;
volatile uint16_t g_raw[RAW_SAMPLES] = {0};
volatile int g_rawWritePtr = 0;
volatile size_t g_timeout = 0;
//volatile int g_I2SQueueFull = 0;

#ifdef FAST_ANALOG_TRACE
volatile unsigned long g_FastTimerCount = 0;
volatile unsigned long g_FastLoopCount = 0;
volatile unsigned long g_delay = 0;
volatile int g_test = 0;
volatile unsigned long g_FastAnaReadBytesRead = 0;
volatile long g_FastAnaReadLoopDurationMax = 0;
volatile long g_FastAnaReadLoopDuration = 0;
//volatile int g_FastAnaReadLoopDurationMaxLoop = 0;
volatile long g_FastAnaReadLoopDurationMin = 9999;
volatile bool g_FastAnaReadTaskSuspended = false;
volatile long g_readTime = 0;
volatile int g_inQueueMax = 0;
volatile long g_inQueue = 0;
#endif

//----------------------------
// Common
//----------------------------

SemaphoreHandle_t g_MyglobalSemaphore;

portMUX_TYPE g_MainMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_LoopMux = portMUX_INITIALIZER_UNLOCKED;

//-----------------------------------------------------------------------------
// I2S
//-----------------------------------------------------------------------------

void I2SAnalogRead(int Samples)
{
#ifdef FAST_ANALOG_TRACE
  unsigned long startRead = micros();
  //  size_t bytes_read = 0;
  unsigned long total_read = 0;
  size_t total_timeout = 0;
  //  uint16_t samplebuf[I2S_SAMPLES] = {0};    // samples are 16 bit integers
#endif

  size_t bytesNeeded = Samples * 2;
  uint16_t i2sData[Samples] = {0};
  size_t bytesRead = 0;
  //  do
  //  {
  // read data from the I2S peripheral
  // read from i2s
  //    i2s_read(I2S_NUM_0, i2sData, bytesNeeded, &bytesRead, portMAX_DELAY);
  xSemaphoreTake(g_ADCinUse, portMAX_DELAY);
  //    i2s_adc_enable(I2S_NUM_0);
  //    i2s_start(I2S_NUM_0);
  i2s_read(I2S_NUM_0, i2sData, bytesNeeded, &bytesRead, 1);
  //    i2s_stop(I2S_NUM_0);
  //    i2s_adc_disable(I2S_NUM_0);
  xSemaphoreGive(g_ADCinUse);

#ifdef FAST_ANALOG_TRACE
  if (bytesRead != bytesNeeded)
  {
    total_timeout = total_timeout + 1;
  }
  total_read = total_read + bytesRead;
  //    Serial.println("Read bytes:" + String(bytesRead) + " Loop:" + String(g_FastLoopCount) );
#endif

  //  } while (bytesRead > 0);

  /*
  while(total_read < bytesNeeded) {
    i2s_read(I2S_NUM_0, (void*) &samplebuf[total_read/2], bytesNeeded - total_read, &bytes_read, I2S_READ_TIMEOUT); // this will wait until enough data is ready
    if (bytes_read != (bytesNeeded - total_read)){total_timeout = total_timeout + 1;}
    total_read = total_read + bytes_read;
  }
*/

  portENTER_CRITICAL_SAFE(&g_LoopMux);
  xSemaphoreTake(g_RawValuesSemaphore, portMAX_DELAY);

  for (int i = 0; i < Samples; i++)
  {
    // get the channel and data
    //    uint32_t ch = samplebuf[i] >> 12;    // channel is in the top 4 bits
    //    if (ch == ADC_CHANNEL)
    //    {
    g_raw[g_rawWritePtr] = i2sData[i] & 0x0FFF;
    g_rawWritePtr = g_rawWritePtr + 1;
    if (g_rawWritePtr == RAW_SAMPLES)
    {
      g_rawWritePtr = 0;
    }
    //    };
  }

  xSemaphoreGive(g_RawValuesSemaphore);

#ifdef FAST_ANALOG_TRACE
  xSemaphoreTake(g_MyglobalSemaphore, portMAX_DELAY);
  g_test = g_test + 1;
  g_FastAnaReadBytesRead = g_FastAnaReadBytesRead + bytesRead;
  g_timeout = g_timeout + total_timeout;
  g_readTime = g_readTime + micros() - startRead;
  xSemaphoreGive(g_MyglobalSemaphore);
#endif

  portEXIT_CRITICAL_SAFE(&g_LoopMux);
}

void initI2S(void)
{
  i2s_config_t i2s_config = {
      //        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // stops the sample rate being doubled vs RIGHT_LEFT
      .communication_format = I2S_COMM_FORMAT_I2S_LSB,
      .intr_alloc_flags = 0,
      //        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,         // between 2 and 128
      .dma_buf_len = DMA_BUF_LEN, // between 8 and 1024
      .use_apll = false,
      //        .use_apll = true,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0
      //        .fixed_mclk = 1
  };

  //install and start i2s driver
  i2s_driver_install(I2S_NUM_0, &i2s_config, 1, &g_I2SQueueHandle);

  //    Serial.println("I2S Clock rate before:" + String(i2s_get_clk(I2S_NUM_0)));

  //Set clock rate
  //    i2s_set_clk(I2S_NUM_0, 300*I2S_BITS_PER_SAMPLE_16BIT*I2S_CHANNEL_MONO, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  //    i2s_set_sample_rates(I2S_NUM_0, 1000*16);
  //init ADC pad
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2s_adc_enable(I2S_NUM_0);

  // init DAC
  //    i2s_set_dac_mode(I2S_DAC_CHANNEL_RIGHT_EN); // right is GPIO25

  // invert the ADC so that the results match those from the normal adc read api
  adc_set_data_inv(ADC_UNIT_1, true);

  //    i2s_adc_enable(I2S_NUM_0);

  Serial.println("I2S Clock rate:" + String(i2s_get_clk(I2S_NUM_0)));
  Serial.println("initI2S I2Squeue:" + String(uxQueueMessagesWaiting(g_I2SQueueHandle)));
}

/*
//-----------------------------------------------------------------------------
//  AnaLog Read Timer (NOT USE ANY MORE)
//-----------------------------------------------------------------------------

ICACHE_RAM_ATTR void FastAnaReadTimerISR(void)
{
  portENTER_CRITICAL_ISR(&g_FastTimerMux);
  g_FastTimerCount = g_FastTimerCount + 1;
  xSemaphoreGive(g_FastTimerSemaphore);
  portEXIT_CRITICAL_ISR(&g_FastTimerMux);
}

void InitFastTimer(void)
{
    // Fast timer setup
  g_FastTimerhandle = timerBegin(TIMER_FAST_NUMBER, TIMER_PRESCALER, true);
  timerAttachInterrupt(g_FastTimerhandle, &FastAnaReadTimerISR, true);
  timerAlarmWrite(g_FastTimerhandle, TIMER_FAST_FREQUENCY, true);
  timerAlarmEnable(g_FastTimerhandle);

  g_FastTimerSemaphore = xSemaphoreCreateMutex();

  Serial.println("Timer init at " + String(TIMER_FAST_FREQUENCY));
  Serial.println();
}
*/

//------------------------------------------------------------------
// Analog Read Task
//------------------------------------------------------------------

void FastAnaReadLoopTask(void *dummyParameter)
{
  static unsigned long StartMicros = micros();
  static bool SetupDone = false;
  static unsigned long delay = 0;
  long duration = 0;

  for (;;)
  {

    //------------------------------------------------------------------
    // Task Setup (done only on 1st call)
    //------------------------------------------------------------------

    if (!SetupDone)
    {
      Serial.println("Analog aquisition Task Started on core " + String(xPortGetCoreID()));
      initI2S();
      //      InitFastTimer();

      // initialise array containing raw sample values
      for (int i = 0; i < RAW_SAMPLES; i++)
      {
        g_raw[i] = 0;
      }

      SetupDone = true;
      
      Serial.println("Task Setup end I2Squeue:" + String(uxQueueMessagesWaiting(g_I2SQueueHandle)));
    }

    //------------------------------------------------------------------
    // Task Loop (done on each timer semaphore release)
    //------------------------------------------------------------------

    i2s_event_t evt;
    while (xQueueReceive(g_I2SQueueHandle, &evt, portMAX_DELAY) == pdPASS)
    {
      int inQueue = uxQueueMessagesWaiting(g_I2SQueueHandle);

      if (evt.type == I2S_EVENT_RX_DONE)
      {
        StartMicros = micros();
        //        xSemaphoreTake(g_MyglobalSemaphore, portMAX_DELAY);
        //        bool suspended = g_FastAnaReadTaskSuspended;
        //        xSemaphoreGive(g_MyglobalSemaphore);

        //        if (!suspended)
        //        {
        I2SAnalogRead(g_I2SSamples);
        //        }

        //time consuming loop (if necessary !!!)
        delay = max(0UL, g_delay - (micros() - StartMicros));
        if (delay > g_delay)
        {
          delay = 0;
        }

        delayMicroseconds(delay);

        portENTER_CRITICAL_SAFE(&g_LoopMux);
        xSemaphoreTake(g_MyglobalSemaphore, portMAX_DELAY);
        g_inQueue = g_inQueue + inQueue;
        g_inQueueMax = max(inQueue, (int)g_inQueueMax);
        g_FastLoopCount = g_FastLoopCount + 1;
        duration = micros() - StartMicros;
        g_FastAnaReadLoopDuration = g_FastAnaReadLoopDuration + duration;
        g_FastAnaReadLoopDurationMax = max((long)g_FastAnaReadLoopDurationMax, duration);
        g_FastAnaReadLoopDurationMin = min((long)g_FastAnaReadLoopDurationMin, duration);
        xSemaphoreGive(g_MyglobalSemaphore);
        portEXIT_CRITICAL_SAFE(&g_LoopMux);
      }
    }
    //    else
    //    {
    //      Serial.println("Queue timeout expired !!!");
    //    }
  }
}

void FastAnaReadLoopTaskCreate(void)
{
  BaseType_t xReturned;
  xReturned = xTaskCreatePinnedToCore(
      FastAnaReadLoopTask,      /* Task function. */
      "AnaReadTsk",             /* String with name of task. */
      12000,                    /* Stack size in bytes. */
      NULL,                     /* Parameter passed as input of the task */
      1,                        /* Priority of the task. */
      &g_FastAnaReadTaskHandle, /* Task handle. */
      ANA_READ_TASK_ESP_CORE);

  if (xReturned == pdPASS)
  {
    Serial.println("Fast analog aquisition Task created on Core " + String(ANA_READ_TASK_ESP_CORE));
  }
  else
  {
    Serial.println("Fast analog aquisition Task creation failled (" + String(xReturned) + ")");
    //errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY	( -1 )
    //errQUEUE_BLOCKED						( -4 )
    //errQUEUE_YIELD							( -5 )
  }
}

void FastAnaReadLoopTaskSuspend(void)
{
  vTaskSuspend(g_FastAnaReadTaskHandle);
  Serial.println("Fast analog aquisition Task suspended");
}

void FastAnaReadLoopTaskResume(void)
{
  vTaskResume(g_FastAnaReadTaskHandle);
  Serial.println("Fast analog aquisition Task resumed");
}

//-----------------------------------------------------------------------------
// PERIMETER ISR
//-----------------------------------------------------------------------------

ICACHE_RAM_ATTR void PerimeterTimerISR(void)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;     // We have not woken a task at the start of the ISR. 
  BaseType_t QueueReturn;
  byte Message = 1;

  portENTER_CRITICAL_ISR(&g_PerimeterTimerMux);
  g_PerimeterTimerCount = g_PerimeterTimerCount + 1;

//  xSemaphoreGive(g_PerimeterProcTimerSemaphore);
  QueueReturn = xQueueSendToBackFromISR(g_PerimeterTimerQueue, &Message, &xHigherPriorityTaskWoken );
  if (QueueReturn != pdPASS) {
    g_PerimeterQueuefull = g_PerimeterQueuefull + 1;
  }
  if( xHigherPriorityTaskWoken )
 	{
  	portYIELD_FROM_ISR ();
 	}
  portEXIT_CRITICAL_ISR(&g_PerimeterTimerMux);
}

void PerimeterTimerInit(void)
{
  // Perimeter timer setup
  g_PerimeterTimerhandle = timerBegin(TIMER_PERIMETER_NUMBER, TIMER_PRESCALER, true);
  timerAttachInterrupt(g_PerimeterTimerhandle, &PerimeterTimerISR, true);
  timerAlarmWrite(g_PerimeterTimerhandle, TIMER_PERIMETER_PERIOD, true);
  timerAlarmEnable(g_PerimeterTimerhandle);

  //g_PerimeterProcTimerSemaphore = xSemaphoreCreateMutex();

  Serial.println("Perimeter task Timer init at " + String(TIMER_PERIMETER_PERIOD) + " ms");
  Serial.println();
}

void PerimeterQueueInit(void)
{
    // Fast timer queue

    /* Create a queue capable of containing bytes (used as booleans) */
    g_PerimeterTimerQueue = xQueueCreate(PERIMETER_QUEUE_LEN, sizeof(byte) );
    if( g_PerimeterTimerQueue == NULL )
    {
      Serial.println("Perimeter Queue creation problem !!");
    }
    else
    {
      Serial.println("PerimeterQueue init for " + String(PERIMETER_QUEUE_LEN));
    }

  Serial.println();
}

//-----------------------------------------------------------------------------
// PERIMETER Task
//-----------------------------------------------------------------------------

/**
 * Perimeter data processing task Setup function
 * 
 */
void PerimeterProcessingSetup(void)
{
  g_PerimeterRawMax = 0;
  g_PerimeterRawMin = INT_MAX;
  g_PerimeterRawAvg = 0;
  g_isInsidePerimeter = false;
  g_PerimetersignalTimedOut = false;
  g_PerimeterMagnitude = 0;
  g_PerimeterSmoothMagnitude = 0;
  g_PerimeterFilterQuality = 0;
    
  // NOT SURE WHAT SETUP NEEDS TO BE DONE !!!!!!!
}

/**
 * Perimeter data processing task Setup function
 * 
 */
void GetPerimeterRawValues(int Samples)
{
#ifdef PERIMETER_TRACE
  Serial.print("Get " + String(Samples) + " Perimeter values =>");
#endif

  long rawTotal = 0;

  portENTER_CRITICAL_SAFE(&g_PerimeterTimerMux);
  xSemaphoreTake(g_RawValuesSemaphore, portMAX_DELAY);

  // Get start and end point in g_Raw circular buffer
  int Ptr = g_rawWritePtr;
  int endPtr = Ptr - 1;
  if (endPtr < 0)
  {
    endPtr = RAW_SAMPLES - 1;
  }

  int startPtr = endPtr - DMA_BUF_LEN * 2;
  if (startPtr < 0)
  {
    startPtr = RAW_SAMPLES + startPtr;
  }

  int i = startPtr;

  // Get "Samples" values from g_Raw circular buffer and determine min/max/Total
  for (int l = 0; l < Samples; l++)
  {
    g_PerimeterRawMax = max(g_raw[i], g_PerimeterRawMax);
    g_PerimeterRawMin = min(g_raw[i], g_PerimeterRawMin);
    rawTotal = rawTotal + g_raw[i];
    i = i + 1;
    if (i == RAW_SAMPLES)
    {
      i = 0;
    }
  }

  // Calculate average of samples extracted from g_Raw circular buffer
  g_PerimeterRawAvg = rawTotal / Samples;
  xSemaphoreGive(g_RawValuesSemaphore);
  portEXIT_CRITICAL_SAFE(&g_PerimeterTimerMux);

#ifdef PERIMETER_TRACE
  Serial.println(" [" + String(g_PerimeterRawMin) + "," + String(g_PerimeterRawMax) + "] Avg=" + String(g_PerimeterRawAvg));
#endif
}

/**
 * Perimeter data permanent loop processing task
 * 
 */
void PerimeterProcessingLoopTask(void *dummyParameter)
{
#ifdef PERIMETER_TRACE
  static unsigned long StartMicros = micros();
#endif  
  static bool SetupDone = false;

  for (;;)
  {
    //------------------------------------------------------------------
    // Task Setup (done only on 1st call)
    //------------------------------------------------------------------

    if (!SetupDone)
    {
      Serial.println("Perimeter data processing Task Started on core " + String(xPortGetCoreID()));

      PerimeterQueueInit();         // Create queue used by timer based ISR
      PerimeterTimerInit();         // Create and start Timer based ISR
      PerimeterProcessingSetup();   // Initialise task value and results

      //     WHATEVER NEEDS TO BE DONE.....

      SetupDone = true;
      delayMicroseconds(TIMER_PERIMETER_PERIOD);

      Serial.println("PerimeterQueue:" + String(uxQueueMessagesWaiting(g_PerimeterTimerQueue)));

    }

    //------------------------------------------------------------------
    // Task Loop (done on each timer semaphore release)
    //------------------------------------------------------------------

    bool evt;
    while (xQueueReceive(g_PerimeterTimerQueue, &evt, portMAX_DELAY) == pdPASS)
    {

#ifdef PERIMETER_TRACE      
      int inQueue = uxQueueMessagesWaiting(g_PerimeterTimerQueue);
#endif
      if (evt == 1)
      {
      // Get values from fast aquisition Task and Calculate Min/Max/Avg
      GetPerimeterRawValues(192);

      // Run MatchFilter

      // Determine Perimeter status variables

      // DO OTHER STUFF ??

      }
    }
  }
}

/**
 * Perimeter data permanent loop processing task creation
 * 
 */
void PerimeterProcessingLoopTaskCreate(void)
{
  BaseType_t xReturned;
  xReturned = xTaskCreatePinnedToCore(
      PerimeterProcessingLoopTask, /* Task function. */
      "PerimProcTsk",              /* String with name of task. */
      12000,                       /* Stack size in bytes. */
      NULL,                        /* Parameter passed as input of the task */
      1,                           /* Priority of the task. */
      &g_PerimeterProcTaskHandle,  /* Task handle. */
      PERIMETER_PROCESSING_TASK_ESP_CORE);

  if (xReturned == pdPASS)
  {
    Serial.println("Perimeter data processing Task created on Core " + String(ANA_READ_TASK_ESP_CORE));
  }
  else
  {
    Serial.println("Perimeter data processing Task creation failled (" + String(xReturned) + ")");
    //errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY	( -1 )
    //errQUEUE_BLOCKED						( -4 )
    //errQUEUE_YIELD							( -5 )
  }
}

/**
 * Perimeter data permanent loop processing task suspension
 * 
 */
void PerimeterProcessingLoopTaskSuspend(void)
{
  vTaskSuspend(g_PerimeterProcTaskHandle);
  Serial.println("Perimeter data processing Task suspended");
}

/**
 * Perimeter data permanent loop processing task resume
 * 
 */
void PerimeterProcessingLoopTaskResume(void)
{
  vTaskResume(g_PerimeterProcTaskHandle);
  Serial.println("Perimeter data processing Task resumed");
}

//-----------------------------------------------------------------------------
// CPU LOAD
//-----------------------------------------------------------------------------

void ArtificialLoad(long duration)
{
  unsigned long Start = millis();

  while (millis() - Start < duration && duration != 0)
  {
    double count = 0;
    long iterations = 2147483647;
    for (long i = 0; i < iterations; i++)
    {
      for (long j = 0; j < iterations; j++)
      {
        for (long k = 0; k < iterations; k++)
        {
          count = (double)count + (count * PI * i) / j + k;
        }
      }
    }
  }
}

void ArtificialLoadIO(long duration)
{
  unsigned long Start = millis();

  //    xQueueReset(g_I2SQueueHandle);
  //    xSemaphoreTake(g_MyglobalSemaphore, portMAX_DELAY);
  //    g_FastAnaReadTaskSuspended = true;
  //    xSemaphoreGive(g_MyglobalSemaphore);

  //    FastAnaReadLoopTaskSuspend();

  //    i2s_driver_uninstall(I2S_NUM_0);

  //    i2s_adc_disable(I2S_NUM_0);
  //  i2s_stop(I2S_NUM_0);
  while (millis() - Start < duration && duration != 0)
  {

    xSemaphoreTake(g_ADCinUse, portMAX_DELAY);
    //Serial.println("before get_raw");

    i2s_stop(I2S_NUM_0);
    i2s_adc_disable(I2S_NUM_0);

    int val = adc1_get_raw(ADC1_CHANNEL_3);
    //    Serial.print(val); Serial.print(" ");

    //Serial.println("after get_raw");
    i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
    i2s_zero_dma_buffer(I2S_NUM_0);
    i2s_adc_enable(I2S_NUM_0);
    i2s_start(I2S_NUM_0);

    xSemaphoreGive(g_ADCinUse);

    for (int i = 32; i < 39; i++)
    {
      //        analogRead(i);
      digitalRead(i);
      digitalWrite(i, HIGH);
    }
    delay(5);
  }
  //   i2s_start(I2S_NUM_0);

  //    adc_i2s_mode_init(ADC_UNIT_1, ADC_CHANNEL_1);
  //    adc_set_data_inv(ADC_UNIT_1, true);
  //    i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);

  //    initI2S();
  //    xQueueReset(g_I2SQueueHandle);
  //    FastAnaReadLoopTaskResume();
  //    xSemaphoreTake(g_MyglobalSemaphore, portMAX_DELAY);
  //    g_FastAnaReadTaskSuspended = false;
  //    xSemaphoreGive(g_MyglobalSemaphore);
}

//-----------------------------------------------------------------------------
// Main setup
//-----------------------------------------------------------------------------

void setup()
{
  Serial.begin(115200); // For debug
  Serial.println();

  g_ADCinUse = xSemaphoreCreateMutex();
  g_RawValuesSemaphore = xSemaphoreCreateMutex();
  g_MyglobalSemaphore = xSemaphoreCreateMutex();
  
  FastAnaReadLoopTaskCreate();

  delay(50);

  PerimeterProcessingLoopTaskCreate();

  delay(100);
}

//-----------------------------------------------------------------------------
// Main Loop
//-----------------------------------------------------------------------------

void loop()
{
  static unsigned long previous = 0;
#ifdef FAST_ANALOG_TRACE
  unsigned long AnaReadBytesRead = 0;
  int test = 0;
  long inQueue = 0;
  int inQueueMax = 0;
#endif  
  uint16_t raw[RAW_SAMPLES] = {0};

  unsigned long now = 0;
  //    g_delay = TIMER_FAST_FREQUENCY - 10;  // 10 microsecs is assumed task processing fixed overhead
//  g_delay = 0;
  //    g_I2SSamples = 2;
  static long load = 0;

  //time consuming loop

  ArtificialLoad(load * 2 / 4);
//  ArtificialLoadIO(load * 2 / 4);

  unsigned long usedTime = micros() - previous;
//    Serial.println("Used time:" + String(usedTime) + " ms");
#ifndef SERIAL_PLOTTER
  delayMicroseconds(1000 * 1000UL - (micros() - previous));
#endif
  now = micros();

  //    FastAnaReadLoopTaskSuspend();

  portENTER_CRITICAL_SAFE(&g_LoopMux);
  xSemaphoreTake(g_MyglobalSemaphore, portMAX_DELAY);

  for (int i = 0; i < RAW_SAMPLES; i++)
  {
    raw[i] = g_raw[i];
  }

#ifdef FAST_ANALOG_TRACE
  //    unsigned long Timerscalls = g_FastTimerCount;
  unsigned long Loopcalls = g_FastLoopCount;
  unsigned long timeouts = g_timeout;
  unsigned long Readtime = g_readTime;
  unsigned long AnaReadLoopDuration = g_FastAnaReadLoopDuration;
  unsigned long AnaReadLoopDurationMax = g_FastAnaReadLoopDurationMax;
  //    unsigned long AnaReadLoopDurationMaxloop =  g_FastAnaReadLoopDurationMaxLoop;
  unsigned long AnaReadLoopDurationMin = g_FastAnaReadLoopDurationMin;
  AnaReadBytesRead = g_FastAnaReadBytesRead;
  test = g_test;
  inQueue = g_inQueue;
  inQueueMax = g_inQueueMax;
  g_FastTimerCount = 0;
  g_FastLoopCount = 0;
  g_timeout = 0;
  g_readTime = 0;
  g_inQueue = 0;
  g_inQueueMax = 0;
  g_FastAnaReadLoopDuration = 0;
  g_FastAnaReadLoopDurationMax = 0;
  //    g_FastAnaReadLoopDurationMaxLoop = 0;
  g_FastAnaReadLoopDurationMin = 9999;
  g_FastAnaReadBytesRead = 0;
#endif
  
  int Ptr = g_rawWritePtr;
  xSemaphoreGive(g_MyglobalSemaphore);

  portEXIT_CRITICAL_SAFE(&g_LoopMux);

#ifndef SERIAL_PLOTTER

  //    long missed = Timerscalls - Loopcalls;
  //    float missedPct = (float) missed/Timerscalls*100;

  //    Serial.print("TCalls:" + String(Timerscalls));
  //    Serial.print(" => " + String((float)Timerscalls/(now-previous)*1000,2) + " kHz");
  Serial.print(" |LCalls:" + String(Loopcalls));
  Serial.print(" => " + String((float)Loopcalls / (now - previous) * 1000, 2) + " kHz");
  //    Serial.print(" |Missed:" + String(missed) + " " + String(missedPct,2) + " %");
  Serial.print(" |Timeout:" + String(timeouts));
  Serial.print(" |ReadTime:" + String((float)Readtime / Loopcalls) + " us");
  Serial.print(" |LTime:" + String(AnaReadLoopDuration) + " us");
  Serial.print(" |LTime:" + String((float)AnaReadLoopDuration / Loopcalls) + " us");
  //    Serial.print(" [" + String(AnaReadLoopDurationMin) + "-" + String(AnaReadLoopDurationMax) + "] @" + String(AnaReadLoopDurationMaxloop));
  Serial.print(" [" + String(AnaReadLoopDurationMin) + "-" + String(AnaReadLoopDurationMax) + "]");
  Serial.print(" | Bytes:" + String(AnaReadBytesRead));
  //    Serial.print(" | Bytes/sample:" + String((float)AnaReadBytesRead/(g_I2SSamples),3));
  //    Serial.print(" | delta:" + String((float)((AnaReadBytesRead/Loopcalls)-(g_I2SSamples*2)),3));
  Serial.print(" | Ptr:" + String(Ptr));
  Serial.print(" || g_delay:" + String(g_delay));
  Serial.print(" | load:" + String(usedTime / 1000) + " ms");
  Serial.print(" | I2Ssamples:" + String(g_I2SSamples));
  Serial.print(" | FreeHeap:" + String((esp_get_minimum_free_heap_size())));
  Serial.print(" || Queue:" + String((float)inQueue / Loopcalls, 4));
  Serial.print(" | Max:" + String(inQueueMax));

  Serial.print(" | Test:" + String(uxQueueMessagesWaiting(g_I2SQueueHandle)));

  Serial.println();
#endif

  int endPtr = Ptr - 1;
  if (endPtr < 0)
  {
    endPtr = RAW_SAMPLES - 1;
  }
  int startPtr = endPtr - DMA_BUF_LEN * 2;
  if (startPtr < 0)
  {
    startPtr = RAW_SAMPLES + startPtr;
  }

#ifndef SERIAL_PLOTTER

  Serial.print("Ptr:" + String(Ptr));
  Serial.print(" From:" + String(startPtr));
  Serial.println(" to:" + String(endPtr));
  const int topRange = 200;
  const int midRange = 400;
  const int lowRange = 800;

#endif

  int i = startPtr;

  for (int l = 0; l < DMA_BUF_LEN * 2; l++)
  {
//      if (i == Ptr) {Serial.print("-[[");}
#ifndef SERIAL_PLOTTER
    if (raw[i] > 4095 - topRange)
    {
      Serial.print("*");
    }
    else if ((raw[i] > 4095 / 2 - midRange) && (raw[i] < 4095 / 2 + midRange))
    {
      Serial.print("-");
    }
    else if (raw[i] < lowRange)
    {
      Serial.print(".");
    }
    else
    {
      Serial.print(" ");
    }
#endif
#ifdef SERIAL_PLOTTER
    Serial.println(raw[i]);
#endif
    //      Serial.print(" ");
    i = i + 1;
    if (i == RAW_SAMPLES)
    {
      i = 0;
    }
    //      if (i == Ptr) {Serial.print("]]- ");}
    //      else {Serial.print(" ");}
  }

  Serial.println();

  /*
    portENTER_CRITICAL_SAFE(&g_LoopMux);
    xSemaphoreTake(g_MyglobalSemaphore, portMAX_DELAY);
    g_FastTimerCount = 0;
    g_FastLoopCount = 0;
    g_timeout = 0;
    g_readTime = 0;
    g_FastAnaReadLoopDuration = 0;
    g_FastAnaReadLoopDurationMax = 0;
//    g_FastAnaReadLoopDurationMaxLoop = 0;
    g_FastAnaReadLoopDurationMin = 9999;
    g_FastAnaReadBytesRead = 0;
    xSemaphoreGive(g_MyglobalSemaphore);
    portEXIT_CRITICAL_SAFE(&g_LoopMux);
*/

/*
  g_delay = g_delay + 20;
  if (g_delay > TIMER_FAST_FREQUENCY + 10)
  {
    g_delay = 0;
  }
*/

  //    g_I2SSamples = g_I2SSamples + 1;
  //    if(g_I2SSamples >= I2S_SAMPLES){g_I2SSamples=2;}

  load = load + 10;
  //    Serial.println("Load:" + String(load));
  if (load > 900)
  {
    load = 0;
  }

  previous = micros();

  //    xQueueReset(g_I2SQueueHandle);
  //    FastAnaReadLoopTaskResume();
}