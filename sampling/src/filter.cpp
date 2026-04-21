#include "filter.h"
#include "config.h"
#include "globals.h"
#include "fft_processor.h"
#include <esp_random.h>
#include <math.h>
#include <string.h>

#ifdef BONUS

// Filter task: replaces the FFT task in BONUS mode.
// Workflow (one-shot at boot):
//   1. wait for oversampler notification
//   2. inject Gaussian noise + sparse anomalies into vReal, save ground truth
//   3. run Z-score and Hampel on copies, time them, log TPR/FPR/MER/exec
//   4. run FFT on dirty + Z-score + Hampel buffers, compare f_max
//   5. apply ACTIVE_FILTER's f_max to the runtime, spawn adaptive sampler

// Random helpers

// Returns a uniform random float in [0, 1).
// esp_random() reads from the ESP32-S3 hardware TRNG and returns a 32-bit
// unsigned integer; dividing by 2^32 maps it into [0, 1).
static inline float randomUnit() {
    return (float)esp_random() / 4294967296.0f;  // 2^32
}

// Returns a sample from a Gaussian distribution N(0, sigma).
// Standard Box-Muller: two uniform [0,1) samples → one normally-distributed
// value. 
static inline float gaussianNoise(float sigma) {
    float u1 = randomUnit(); 
    float u2 = randomUnit();
    if (u1 < 1e-9f) u1 = 1e-9f;        // guard against log(0)
    return sigma * sqrtf(-2.0f * logf(u1)) * cosf(2.0f * (float)M_PI * u2);
}



// Noise + anomaly injection
static void injectNoiseAndAnomalies(double *buf, bool *isAnomaly, int n) {
    int spikes = 0;
    for (int i = 0; i < n; i++) {
        // n(t): every sample gets a small Gaussian perturbation with mean 0 and std NOISE_SIGMA_LSB
        buf[i] += gaussianNoise(NOISE_SIGMA_LSB);

        // A(t): with probability ANOMALY_PROB, add a large spike
        // of random sign and magnitude ~ U(SPIKE_MIN, SPIKE_MAX)
        if (randomUnit() < ANOMALY_PROB) {
            float sign      = (randomUnit() < 0.5f) ? -1.0f : 1.0f;
            float magnitude = SPIKE_MIN_LSB + randomUnit() * (SPIKE_MAX_LSB - SPIKE_MIN_LSB);
            buf[i] += sign * magnitude;
            isAnomaly[i] = true;        // isAnomaly[i] is set true iff a spike was injected at position i —
                                        // this is the ground truth used by evalMetrics() to compute TPR/FPR.
            spikes++;
        } else {
            isAnomaly[i] = false;
        }
    }
    Serial.printf("[INJECT] sigma=%.0f LSB, p=%.2f, injected=%d/%d spikes\n",
                  NOISE_SIGMA_LSB, ANOMALY_PROB, spikes, n);
}

// Sliding-window helpers (used by both filters)


// Given an index i and buffer size n, it returns the bounds [lo, hi) 
// of a centered sliding window of size FILTER_WINDOW around i.
// Windows near the buffer edges are truncated.
static inline void windowBounds(int i, int n, int &lo, int &hi) {
    int half = FILTER_WINDOW / 2;
    lo = i - half; if (lo < 0) lo = 0;
    hi = i + half + 1; if (hi > n) hi = n;
}

// In-place sort of a small array. Used by Hampel to compute the median
// of a sliding window.
static void insertionSort(double *a, int len) {
    for (int i = 1; i < len; i++) {
        double key = a[i];
        int j = i - 1;
        while (j >= 0 && a[j] > key) { a[j+1] = a[j]; j--; }
        a[j+1] = key;
    }
}

// Z-score filter
//
// For each sample i:
//   1. Take a centered sliding window of FILTER_WINDOW samples around i
//   2. Compute the local MEAN and STANDARD DEVIATION of the window
//   3. If |x[i] - mean| > THRESHOLD * std → flag as anomaly, replace with mean
//
// Cheap (O(n*W) and no sorting) but suffers from the MASKING effect:
// a single large spike inflates the local std, raising the detection
// threshold and hiding nearby spikes. Reliable when anomalies are
// rare (p ≤ ~1%), unreliable when they are dense.
static void zscoreFilter(const double *in, double *out, bool *detected, int n) {
    for (int i = 0; i < n; i++) {
        int lo, hi; windowBounds(i, n, lo, hi);
        int w = hi - lo;

        // Local mean
        double sum = 0.0;
        for (int k = lo; k < hi; k++) sum += in[k];
        double mean = sum / w;

        // Local standard deviation
        double sq = 0.0;
        for (int k = lo; k < hi; k++) { double d = in[k] - mean; sq += d * d; }
        double std = sqrt(sq / w);

        // Detection + imputation: replace flagged samples with the local mean
        if (std > 1e-9 && fabs(in[i] - mean) > ZSCORE_THRESHOLD * std) {
            detected[i] = true;
            out[i] = mean;
        } else {
            detected[i] = false;
            out[i] = in[i];
        }
    }
}

// Hampel filter
//
// For each sample i:
//   1. Take a centered sliding window of FILTER_WINDOW samples around i
//   2. Compute the local MEDIAN and MAD (median absolute deviation) of the window
//   3. If |x[i] - median| > THRESHOLD * MAD → flag as anomaly, replace with median
//
// Robust to dense anomalies — the median ignores up to 50% outliers, so
// the detection threshold doesn't get inflated by neighboring spikes.
// More expensive than Z-score: each sample requires sorting the window
// twice (once for the median of values, once for the median of deviations).
static void hampelFilter(const double *in, double *out, bool *detected, int n) {
    static double work[FILTER_WINDOW];   // workspace for sorting window values
    static double devs[FILTER_WINDOW];   // workspace for sorting deviations

    for (int i = 0; i < n; i++) {
        int lo, hi; windowBounds(i, n, lo, hi);
        int w = hi - lo;

        // Median of the window values
        for (int k = 0; k < w; k++) work[k] = in[lo + k];
        insertionSort(work, w);
        double median = (w & 1) ? work[w/2]
                                : 0.5 * (work[w/2 - 1] + work[w/2]);

        // MAD = median of |x - median|
        for (int k = 0; k < w; k++) devs[k] = fabs(in[lo + k] - median);
        insertionSort(devs, w);
        double mad = (w & 1) ? devs[w/2]
                             : 0.5 * (devs[w/2 - 1] + devs[w/2]);

        // The MAD is a robust measure of dispersion, but it is smaller than the standard deviation of a normal distribution. 
        // Multiplying by 1.4826 scales the MAD to match the std of a Gaussian distribution, 
        // ensuring the threshold for outlier detection is accurate.
        double sigmaEst = 1.4826 * mad;

        // Detection + imputation: replace flagged samples with the local median
        if (sigmaEst > 1e-9 && fabs(in[i] - median) > HAMPEL_THRESHOLD * sigmaEst) {
            detected[i] = true;
            out[i] = median;
        } else {
            detected[i] = false;
            out[i] = in[i];
        }
    }
}


// Compares filter detections against the injection ground truth and prints:
//   - Confusion matrix: TP, FP, FN, TN
//   - TPR: fraction of injected spikes correctly detected
//   - FPR: fraction of clean samples incorrectly flagged
//   - MER (Mean Error Reduction):
//         MER = 1 - MSE(filtered, clean) / MSE(dirty, clean)
//     MER=1 → perfect reconstruction, MER=0 → no improvement, MER<0 → worse
//   - exec: total filter execution time in microseconds
static void evalMetrics(const char *name, const bool *gt, const bool *det,
                        const double *clean, const double *dirty, const double *filt,
                        int n, unsigned long execUs) {
    int tp = 0, fp = 0, fn = 0, tn = 0;
    double mseDirty = 0.0, mseFilt = 0.0;
    for (int i = 0; i < n; i++) {
        if (gt[i]  &&  det[i]) tp++;
        if (!gt[i] &&  det[i]) fp++;
        if (gt[i]  && !det[i]) fn++;
        if (!gt[i] && !det[i]) tn++;
        double dD = dirty[i] - clean[i]; mseDirty += dD * dD;
        double dF = filt[i]  - clean[i]; mseFilt  += dF * dF;
    }
    mseDirty /= n; mseFilt /= n;
    float tpr = (tp + fn) > 0 ? (float)tp / (tp + fn) : 0.0f;
    float fpr = (fp + tn) > 0 ? (float)fp / (fp + tn) : 0.0f;
    float mer = mseDirty > 1e-9 ? (float)(1.0 - mseFilt / mseDirty) : 0.0f;

    Serial.printf("[%s] win=%d  TP=%d FP=%d FN=%d TN=%d  TPR=%.3f FPR=%.3f  MER=%.3f  exec=%lu us\n",
                  name, FILTER_WINDOW, tp, fp, fn, tn, tpr, fpr, mer, execUs);
}



void TaskFilter(void *pvParameters) {
    // static → i buffer vivono nella SRAM globale, non nello stack del task.
    // Lo stack del task è 16 KB; questi buffer insieme fanno ~18 KB → senza static
    // sarebbe stack overflow garantito al primo boot.
    static double vClean[FFT_SAMPLES];      // signal before injection
    static double vDirty[FFT_SAMPLES];      // signal after noise + anomaly injection
    static double vZscore[FFT_SAMPLES];     // output of the Z-score filter
    static double vHampel[FFT_SAMPLES];     // output of the Hampel filter
    static bool   isAnomaly[FFT_SAMPLES];   // ground truth: which samples carry a spike
    static bool   detZ[FFT_SAMPLES];        // Z-score detections (true = flagged as anomaly)
    static bool   detH[FFT_SAMPLES];        // Hampel detections (true = flagged as anomaly)

    // Wait for the oversampler to fill vReal with clean ADC readings
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Snapshot the clean signal, then contaminate vDirty
    memcpy(vClean, vReal, sizeof(vClean));
    memcpy(vDirty, vReal, sizeof(vDirty));
    injectNoiseAndAnomalies(vDirty, isAnomaly, FFT_SAMPLES);

    // Run both filters on independent copies, time each one
    unsigned long t0 = micros();
    zscoreFilter(vDirty, vZscore, detZ, FFT_SAMPLES);
    unsigned long execZ = micros() - t0;

    t0 = micros();
    hampelFilter(vDirty, vHampel, detH, FFT_SAMPLES);
    unsigned long execH = micros() - t0;

    evalMetrics("Z-SCORE", isAnomaly, detZ, vClean, vDirty, vZscore, FFT_SAMPLES, execZ);
    evalMetrics("HAMPEL ", isAnomaly, detH, vClean, vDirty, vHampel, FFT_SAMPLES, execH);

    // Run FFT on each variant.
    float fDirty  = runFFTAndFindMax(vDirty,  FFT_SAMPLES, MAX_SAMPLING_FREQ);
    float fZscore = runFFTAndFindMax(vZscore, FFT_SAMPLES, MAX_SAMPLING_FREQ);
    float fHampel = runFFTAndFindMax(vHampel, FFT_SAMPLES, MAX_SAMPLING_FREQ);

    Serial.printf("[FMAX] dirty=%.2f Hz  zscore=%.2f Hz  hampel=%.2f Hz\n",
                  fDirty, fZscore, fHampel);

    // Hand off the chosen filter's f_max to the runtime pipeline.
    float chosen;
#if ACTIVE_FILTER == FILTER_ZSCORE
    chosen = fZscore;
    Serial.println("[FILTER] active = Z-SCORE");
#else
    chosen = fHampel;
    Serial.println("[FILTER] active = HAMPEL");
#endif
    applyAdaptiveRate(chosen);

    vTaskDelete(NULL);
}

#endif
