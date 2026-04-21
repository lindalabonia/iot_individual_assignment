#include "config.h"
#include "globals.h"

// Shared state — volatile because these are written by one task and read by
// others on different cores. Forces the compiler to re-read from RAM every time
// instead of caching in a CPU register (which would miss cross-task updates).
volatile float adaptiveSamplingFreq = MAX_SAMPLING_FREQ;
volatile float detectedMaxFreq = 0.0;
volatile bool  fftDone = false;

// FFT buffers
double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];

// Synchronization
QueueHandle_t sampleQueue         = NULL;
TaskHandle_t  fftTaskHandle       = NULL;
TaskHandle_t  adaptiveTaskHandle  = NULL;

#ifdef USE_LORAWAN
QueueHandle_t txQueue             = NULL;
#endif

#ifdef BONUS
TaskHandle_t  filterTaskHandle    = NULL;
#endif
