# Adaptive Sampling IoT System on ESP32-S3

<!-- Badges: optional, you can remove or swap these. -->
![Platform](https://img.shields.io/badge/platform-ESP32--S3-blue)
![Framework](https://img.shields.io/badge/framework-Arduino%20%2B%20FreeRTOS-orange)
![Build](https://img.shields.io/badge/build-PlatformIO-informational)
![LoRaWAN](https://img.shields.io/badge/LoRaWAN-TTN%20EU868-brightgreen)
![MQTT](https://img.shields.io/badge/MQTT-Mosquitto-660066)

---

## Project info

| | |
|---|---|
| **Course** | Internet of Things |
| **Academic year** | 2025/2026 |
| **University** | Sapienza Università di Roma — M.Sc. in Engineering in Computer Science |
| **Instructors** | Prof. Andrea Vitaletti, Prof. Ioannis Chatzigiannakis |
| **Student** | Linda Labonia |
| **Matricola** | 2219714 |
| **Email** | labonia.2219714@studenti.uniroma1.it |

---

## Overview

This project implements an **adaptive sampling** pipeline on an ESP32-S3,
designed to sample a periodic analog signal at the lowest rate that still
preserves its information content and forward the aggregated measurements
both locally (edge) and to the cloud.

The core idea is a two-phase pipeline:

1. **Discovery phase (one-shot).** At boot, the ADC oversamples the signal
   and an FFT identifies the highest significant frequency component
   `f_max` in the spectrum.
2. **Steady-state phase (continuous).** An adaptive task then samples the
   same signal at a much lower rate (`2.2 × f_max`, the Nyquist bound plus
   a safety margin), computes a windowed average, and publishes it over
   either **MQTT/WiFi** (local Mosquitto broker) or **LoRaWAN/TTN**.

The system is fully self-contained: the ESP32 **generates its own test signal** via an
MCP4725 DAC whose output is physically looped back into the ADC input.
The microcontroller therefore plays both the role of signal source and of
sensing device.

A central goal of the assignment is to evaluate the overall performance impact of these design choices, in terms of sampling efficiency, communication cost, latency and energy consumption.
The bonus part further extends the evaluation by testing the robustness of the FFT-driven adaptive pipeline under noisy and spike-contaminated signals, comparing Z-score and Hampel filtering.

---

## Hardware

### Components and Wiring

| Component | Role |
|---|---|
| **Heltec WiFi LoRa 32 V3** | Microcontroller (ESP32-S3) with on-board LoRa SX1262 |
| **MCP4725** | External 12-bit I2C DAC |
| **LoRa antenna** | EU868 antenna connected to the Heltec V3 |

The complete wiring is shown in the figure below.

<p align="center">
  <img src="docs/hardware_setup.png" alt="Hardware wiring" width="550">
</p>

---

## Software architecture

The firmware is organized as a small FreeRTOS pipeline running on the two ESP32-S3 cores. Depending on the build, the system uses five tasks in MQTT mode or six tasks in LoRa mode. The build mode is selected at compile time through `config.h`, which enables either the MQTT or the LoRa communication path.


![System architecture](docs/system_architecture.png)

**Core 0**
- **DAC Generator** (pri 2, forever) — writes the precomputed signal LUT to the MCP4725 over I2C.
- **LoRa TX** (pri 1, forever, LoRa builds only) — transmits each aggregated average over LoRaWAN.

**Core 1**
- **Oversampler** (pri 2, one-shot) — at boot, fills the FFT buffer at the configured oversampling rate.
- **FFT Processor** (pri 1, one-shot) — computes the FFT, finds `f_max`, and spawns the adaptive sampler.
- **Adaptive Sampler** (pri 2, forever) — samples the ADC at `2.2 × f_max`.
- **Aggregator** (pri 1, forever) — computes window averages and either publishes them directly via MQTT or writes them to `txQueue`, where they are consumed by the LoRa TX task.

In **MQTT** mode there is no dedicated TX task: a local-network publish takes only a few milliseconds, so the aggregator (on Core 1) can call `mqttPublish()` synchronously. Radio activity still ends up on Core 0 even in this case, because the WiFi driver task of the ESP-IDF runtime is pinned there.  
In **LoRa** mode, instead, a dedicated task is created on Core 0. LoRaWAN Class A devices are required by the specification to open two receive windows after every uplink (RX1 at +5 s and RX2 at +6 s on TTN), so `sendReceive()` is inherently a multi-second blocking call and has to run in its own task to avoid stalling the pipeline.

### Task priorities

- **DAC (pri 2) > LoRa TX (pri 1)** on Core 0. Signal generation must not pause while a packet is being sent: `sendReceive()` blocks for ~7 s on TTN, and the scheduler preempts the TX task whenever a DAC write is due.
- **Adaptive Sampler (pri 2) > Aggregator (pri 1)** on Core 1. The sampler has hard periodic deadlines, while the aggregator only processes data that has already been acquired.

### Task pacing: `vTaskDelay` as the default

Since the whole system runs on a single ESP32-S3 with only two cores, the preferred design choice was to use `vTaskDelay()` for periodic tasks, so that the CPU is released to the scheduler and other tasks can run. The trade-off is timing accuracy, because `vTaskDelay()` carries ±1 ms of inherent jitter. This constraint influenced several design choices discussed in the following sections. The main exception is the Oversampler, which uses a busy-wait on `micros()` because it is a one-shot task and its much shorter sampling period would be too sensitive to such jitter.

### Inter-task communication

| Producer → Consumer | Mechanism |
|---|---|
| Oversampler → FFT | `xTaskNotifyGive` + shared global buffers `vReal[]`/`vImag[]` |
| FFT → Adaptive | dynamic task creation + shared global `adaptiveSamplingFreq` |
| FFT → Aggregator | global `fftDone` flag, polled by the aggregator every 100 ms |
| Adaptive → Aggregator | `sampleQueue`, size `MAX_SAMPLING_FREQ * WINDOW_DURATION_SEC`, `sizeof(float)` |
| Aggregator → LoRa TX | `txQueue`, size 1, `sizeof(TxFrame_t)` |

The steady-state pipeline (Adaptive Sampler → Aggregator → LoRa TX) uses **FreeRTOS queues**, which wake the consumer automatically when data arrives and avoid polling. For the one-off boot handshake between Oversampler and FFT, a task notification (`xTaskNotifyGive`) is used instead: it is FreeRTOS's cheapest primitive for one-producer-one-consumer
signalling.

Shared globals (`adaptiveSamplingFreq`, `detectedMaxFreq`, `fftDone`, and the FFT buffers) are declared **`volatile`** so that updates made by a task on one core are immediately visible to tasks on the other, without the compiler caching stale values in CPU registers.



---

## Signal processing

### Signal generation

The test signal is defined as a sum of sinusoids:

```
s(t) = Σ a_k · sin(2π · f_k · t)
```

with the `amplitudes[]` and `frequencies[]` arrays declared in
`signal_generator.cpp` and the number of components set by
`NUM_COMPONENTS` in `config.h`. 

Instead of computing `sin()` at every DAC write, `buildSignalLUT()`
precomputes a full period of the composite signal into a **lookup
table** at boot. The LUT is a standard optimisation: at high signal
frequencies the cost of a `sin()` call per sample becomes significant
relative to the time between writes, so precomputing the waveform
once and cycling through the array is much cheaper.


#### Upper bound on the generated signal frequency

From the hardware side, each MCP4725 write occupies about 29 I2C bits, including the 12-bit DAC value and the protocol overhead required for the transaction. With a 400 kHz I2C clock, this gives a minimum write time of

\[
T_{\text{write}} = \frac{29}{400\,000} \approx 72.5\,\mu s
\]

and therefore a theoretical maximum DAC update rate of

\[
f_{\text{update,max}} = \frac{1}{T_{\text{write}}} \approx 13.8\,\text{kHz}
\]

A further design choice in this project is to approximate each signal period with 40 DAC points, in order to obtain a reasonably smooth waveform. This means that the DAC update rate and the signal frequency are related by

\[
f_{\text{update}} = 40 \cdot f_{\text{signal}}
\]

so the corresponding theoretical hardware upper bound on the generated signal frequency is

\[
f_{\text{signal,max}} = \frac{13.8\,\text{kHz}}{40} \approx 345\,\text{Hz}
\]

------

A second limit is introduced by the software design. In order to release the CPU between two writes, `vTaskDelay()` should be used and the time between two consecutive writes must be at least 1 ms.

\[
T_{\text{write spacing}} = \frac{1000}{40 \cdot f_{\text{signal}}} \geq 1 \text{ ms}
\]

which gives

\[
f_{\text{signal,max}} \leq \frac{1000}{40} = 25 \text{ Hz}
\]

Therefore, although the hardware upper bound is much higher, the practical limit adopted in this project is 25 Hz, so that signal generation can coexist with the other tasks.


-----------

### Signal Sampling 


The ADC is an internal peripheral of the ESP32-S3, and a single `analogRead()` takes about **10–20 µs**, depending on the ADC configuration. This corresponds to a theoretical sampling-rate ceiling in the **50–100 kHz** range.

Since the generated test signals are intentionally limited to **25 Hz**, an initial oversampling rate of **500 Hz** was chosen. This value is already much higher than the signal frequency and is therefore sufficient to capture the waveform correctly. The figure below shows that the oversampler reconstructs the signal \( \sin(2\pi \cdot 5t) \) accurately.

![Oversampled signal at 500 Hz](docs/sin5hz_ric_oversam.png)


#### Sampling Theorem

The whole point of the adaptive-sampling pipeline rests on the
**Nyquist–Shannon sampling theorem**: a band-limited signal can be
reconstructed perfectly from its samples if and only if the sampling
rate is **strictly greater than twice the highest frequency present
in the signal**.

Sampling below `2 · f_max` causes *aliasing*: high-frequency content
gets folded down and appears as spurious low-frequency components,
and the original signal can no longer be recovered from the samples.

The animation below illustrates the theorem visually:

![Sampling theorem animation](docs\nyquist.gif)



This is exactly why the FFT step matters: once we know `f_max`, we
can drop the sampling rate to just above `2 · f_max` and still
reconstruct the signal — saving both energy (fewer ADC reads per
second) and data volume (fewer samples to aggregate and transmit).


### Finding `f_max`

Before running the FFT, two pre-processing steps are applied to the
oversampled buffer. Both address *artefacts* — things the FFT would
see but that are not really part of the signal.

**DC offset removal.**  
Because the MCP4725 can only output positive voltages, the generated waveform is not \(s(t)\) itself, but rather

\[
x(t) = 2048 + s(t)
\]

that is, the signal shifted around the DAC mid-scale. In the frequency domain, the constant term \(2048\) appears as a large spike at bin 0, i.e. the DC component. This would dominate the spectrum and make the 5% relative threshold depend on the DC offset rather than on the actual sinusoidal components. To avoid this, the mean of the 512 oversampled samples is subtracted before computing the FFT, so that the DC component is removed and the spectrum reflects only the real frequency content of \(s(t)\).

**Hamming windowing.**
A Hamming window is applied to suppress the **spectral leakage**
intrinsic to computing the FFT over finite time windows. By
attenuating the signal amplitude toward the edges of the buffer, the
Hamming function cancels the boundary discontinuities, improving
spectral identification and enabling a cleaner, more accurate
extraction of the dominant frequency `f_max`.


**Peak selection.**  
After DC offset removal and Hamming windowing, the FFT is converted into a magnitude spectrum and scanned to identify \(f_{\max}\). To avoid selecting noise or leakage bins, only bins that satisfy the following two conditions are kept:

- **Relative magnitude threshold:** a bin is kept only if its
     magnitude is at least `FFT_MAG_THRESHOLD_RATIO = 5 %` of the
     peak magnitude. Using a *ratio* rather than an absolute value
     means the threshold scales automatically with signal amplitude
     and ADC configuration — no manual tuning needed.
- **Local-peak check:** the bin must be strictly greater than both
     neighbours. When a true frequency is present, the FFT shows a main peak surrounded by smaller neighbouring bins caused by spectral leakage. By keeping only bins that are greater than both neighbours, the algorithm selects the central peak and discards the side bins.

Among the surviving candidates, the one with the highest frequency is selected as \(f_{\max}\), since adaptive sampling must track the highest significant component rather than the strongest one.

**Adaptive rate.**  
Once \(f_{\max}\) is known, the adaptive sampling rate is set as

\[
f_{\text{adaptive}} = 2.2 \cdot f_{\max}
\]

The factor 2.2 introduces a small safety margin over the strict Nyquist bound, compensating for the limited FFT bin resolution.

**Result.**  
For a 5 Hz test signal, the FFT returns \(f_{\max} \approx 4.88\) Hz. This is consistent with the FFT bin resolution, since with a 500 Hz sampling rate and 512 samples each bin corresponds to

\[
\frac{500}{512} \approx 0.98 \text{ Hz}
\]

so a 5 Hz component is detected at

\[
5 \cdot \frac{500}{512} \approx 4.88 \text{ Hz}
\]

The resulting adaptive rate is therefore \(f_{\text{adaptive}} \approx 10.7\) Hz, corresponding to about a \(47\times\) reduction with respect to the initial 500 Hz oversampling rate, while still preserving the waveform correctly.

![Adaptive sampling of the 5 Hz signal](docs/sin5hz_ric_adapt.png)



## Communication

### WiFi + MQTT

When the device is on the same LAN as the receiver, each aggregated value is published to a local **Mosquitto** broker running on the PC. In the current implementation, the ESP32 connects to WiFi once at boot and keeps both the WiFi association and the MQTT connection open across aggregation windows.

This choice depends on the aggregation window. For very short windows, reconnecting at every publish would add too much time and energy overhead; for very long windows, disconnecting and reconnecting both WiFi and MQTT could instead be preferable, because the radio could be turned off between transmissions. In our case, a persistent connection was chosen because it is simpler, adds no extra latency to each publish, and is consistent with the device already being kept awake by the DAC generator task.

Note that when using a persistent MQTT connection, the keepalive must be longer than the aggregation window; otherwise, the broker may close the connection between two consecutive publishes.

In this setup, each message is sent as a compact JSON string on topic `iot/sensor/avg`, with
the window number `w` and the average `avg` in volts. The subscriber
on the PC receives the stream
of averages in real time — the screenshot below is taken while the
device is generating the composite test signal
`s(t) = 2·sin(2π·3t) + 4·sin(2π·5t)`.

![MQTT subscriber receiving window averages](docs/mqtt_sub.png)

### LoRaWAN + TTN

The same avarage can be sent over **LoRaWAN**
to **The Things Network** (TTN). 

- **Band.** EU868.
- **Provisioning.** The device is registered on the TTN console as
  an **OTAA** (Over-The-Air Activation) end-device; the `DevEUI`, `JoinEUI`
  and `AppKey` are hardcoded in `config.h`. At boot the ESP32
  sends a JoinRequest and the network server replies with a
  JoinAccept, establishing the session keys used to encrypt every
  subsequent uplink.
- **Payload encoding.**  
  To reduce airtime, the MQTT JSON payload is replaced by a compact 4-byte binary payload for LoRaWAN.

  | Field | Encoding | Size |
  |---|---|---|
  | `avg` | `uint16(avg × 100)` | 2 B |
  | `window` | `uint16(window)` | 2 B |
  | **Total** |  | **4 B** |

  On the TTN side, a small JavaScript decoder converts the payload back into `{ avg, window }` for display.
- **Data rate.**  
  DR5 = **SF7 / 125 kHz**.  
  - `125 kHz` is the default EU868 bandwidth on mandatory channels.
  - Every gateway is guaranteed to listen on it.
  - With this configuration, the 4-byte payload lasts about **50 ms on air**.

- **Duty cycle.**  
  Two limits must be respected:
  - **EU868 regulation:** **1% airtime** (≤ 36 s/hour)
  - **TTN fair use:** **30 s/day**

  These limits constrain the choice of aggregation window and spreading factor.

- **Class A behaviour.**  
  After each uplink, the device opens two receive windows:
  - **RX1:** +5 s
  - **RX2:** +6 s

  So the radio remains busy for about **7 s per transmission**. This is why LoRa TX runs in a dedicated task on Core 0. See the animation below:
  ![Class A uplink + RX1/RX2 windows](docs/classeA_lora.gif)
  

The image below shows the TTN console correctly receiving the
aggregate values. The test signal during the capture is the same
used for MQTT, `s(t) = 2·sin(2π·3t) + 4·sin(2π·5t)`:

![TTN live data — uplink stream](docs/ttn_live_data.png)



## Performance evaluation

All current measurements reported in this section were taken with the
setup shown below using an **INA219** current-sense module.

![Energy measurement circuit](docs/circuit_image_energy.png)

### Energy: oversampling vs adaptive sampling

Both configurations were measured with `SKIP_TX` enabled in `config.h` (no radio
activity) and the composite test signal
`s(t) = 2·sin(2π·3t) + 4·sin(2π·5t)` at the DAC output, so the only
variable between the two runs is the rate of the adaptive sampler.

| Mode | Sampling rate | Mean current |
|---|---|---|
| Oversampling | 500 Hz | **~72.5 mA** |
| Adaptive | ~10.7 Hz | **~52 mA** |

The ~20 mA gap (**≈ 28 % reduction**) is the net cost of running the
ADC task at a much lower rate. Note that both figures include the
DAC generator task, which keeps running regardless of the sampling
mode — so the adaptive floor is not the cost of sampling alone, but
of sampling plus continuous signal generation.

### Energy: WiFi vs LoRa

Both measurements were taken with adaptive sampling enabled and one
transmission every 30 s; both traces plot instantaneous current in
mA, and both share the same ~50 mA baseline (the "always-on" cost of
the DAC generator + adaptive sampler, already analysed above).

![LoRa current profile](docs/consumo_lora.png)

In **LoRa** mode the trace is almost flat at baseline between events:
once per window the current jumps sharply to ~140 mA during the
uplink transmission. The
Class A receive windows that follow each TX draw so little power that they do not
even emerge visibly from the baseline on this scale.

![WiFi current profile](docs/consumo_wifi.png)

In **WiFi** mode the transmission event itself is visible as a tall
peak at ~200 mA, but a comb of smaller peaks sits on top of the idle
current for the entire run. These additional peaks are the price of keeping the WiFi
interface associated to the access point between one publish and
the next. The
trade-off behind this design choice is already discussed in the
*Communication* section.

**Conclusion.** These two traces confirm the well-known theoretical
expectation: **LoRa is the more energy-efficient of the two radios**.

### Latency: WiFi vs LoRa

To reflect the capabilities of each protocol stack, two distinct measurement methodologies were employed. 
- For **LoRa**, the time measured is
  between `node.sendReceive()` and the return of the call, i.e. the
  round-trip between uplink transmission and the end of the MAC-level
  receive windows. 
- For **WiFi**, NTP clock synchronization on the local
  network did not give reliable results, so device-to-broker delivery
  latency could not be measured directly. The adopted workaround is to
  have the device also subscribe to its own topic and measure the time
  between publish and reception of the same message. 
The two scenarios are not identical, but the **order of magnitude** is what matters.

![LoRa latency](docs/latency_lora.png)

In **LoRa** mode the measured values sit around ~6 s, consistent with
expectations: TTN opens RX1 at +5 s and RX2 at +6 s after each uplink,
and `sendReceive()` returns as soon as a downlink is received in one
of the two windows, or when RX2 closes with no downlink. This floor is structural and cannot be reduced without leaving Class A.

![WiFi latency](docs/latency_wifi.png)

In **WiFi** mode latencies fluctuate between ~15 ms and ~144 ms, with
visible jitter from one window to the next. The jitter comes from
WiFi being a shared medium: devices arbitrate access by listening
first and backing off for a random interval when the channel is busy
(CSMA/CA), which injects non-deterministic delay into every publish.

**Conclusion.** These two traces confirm that **WiFi is the more
suitable of the two radios for latency-constrained communications**.

### Window execution time

The pipeline is window-driven, not sample-driven: the aggregator
collects samples for a fixed interval and emits one measurement at
the end of each interval. Therefore, the effective per-window time is dominated by the window duration, plus a small computation and transmission overhead.

### Data volume: adaptive vs oversampling, WiFi vs LoRa

The transmitted quantity is one aggregated value per window, so the
data volume on the network is the same under adaptive sampling and
under oversampling: the savings of adaptive sampling are on the
device side, not on the link. If each raw sample
were transmitted instead of the window average, oversampling would
produce proportionally more traffic.

The two radios carry very different payloads by design. MQTT
publishes a compact JSON `{"w":N,"avg":V.VVVV}` (~20 B), while LoRa
encodes the same information in 4 binary bytes. On top of this, MQTT also carries
significantly larger protocol headers than LoRaWAN, so the gap on the actual bytes
on the air is even wider than the payload comparison alone suggests.





## Bonus

### 1. Different clean input signals and correctness of `f_max` detection

To verify that the FFT pipeline correctly identifies the highest significant frequency component, the system was tested on three different signals. The goal of these tests is to check that the detected `f_max` corresponds to the highest frequency actually present in the signal, independently of the amplitudes of the individual components.

The tested signals were:

\[
s_1(t)=2\sin(2\pi\cdot 5t)+5\sin(2\pi\cdot 25t)
\]

\[
s_2(t)=5\sin(2\pi\cdot 5t)+2\sin(2\pi\cdot 25t)
\]

\[
s_3(t)=5\sin(2\pi\cdot 1t)+2\sin(2\pi\cdot 25t)+4\sin(2\pi\cdot 5t)
\]

In all three cases, the system consistently reported an FFT-estimated maximum frequency of about \(25.39\) Hz. This value is coherent with the FFT resolution.

![FFT output for signal 1](docs/sig1.png)

### 2. Anomaly-aware filtering under noisy and spike-contaminated signals

This subsection evaluates the robustness of the adaptive sampling pipeline when the original clean signal is corrupted by two non-ideal components: a small Gaussian baseline noise and a sparse anomaly-spike process. In particular, the noisy signal follows the assignment model

\[
s(t)=2\sin(2\pi\cdot 3t)+4\sin(2\pi\cdot 5t)+n(t)+A(t)
\]

where 
- \(n(t)\) is a zero-mean Gaussian noise term with standard deviation \(\sigma=0.2\)
- \(A(t)\) is a sparse spike component such that, with probability \(p\), a large outlier with random sign and magnitude in the range \([5,15]\) is injected.

In the implementation, noise and anomalies are injected in a separate task on the samples of the clean signal after they have already been acquired at the oversampling frequency. This design makes it straightforward to preserve a clean ground truth and to compare, sample by sample, the contaminated and filtered versions of the signal. The same ground truth is then used to evaluate the performance of the filters.

Two sliding-window filters are considered: **Z-score** and **Hampel**.

In the **Z-score filter**, for each sample a local mean and a local standard deviation are computed over a window centered around the current point. A sample is classified as anomalous when its distance from the local mean exceeds a threshold proportional to the local standard deviation:

\[
|x_i-\mu_w|>k\sigma_w
\]

where \(\mu_w\) and \(\sigma_w\) are the mean and standard deviation computed inside the window, and \(k\) is a threshold parameter. When the condition is satisfied, the detected anomaly is replaced with the local mean. 
The main limitation of this method is that a spike influences the same mean and standard deviation used for detection. As a consequence, the threshold becomes larger, and the anomaly may no longer exceed it and therefore remain undetected.

The **Hampel filter** replaces these classical statistics with robust ones. In particular, it uses the local median \(m_w\) and the MAD (Median Absolute Deviation), defined as

\[
MAD = \mathrm{median}(|x_j-m_w|)
\]

where \(m_w\) is the median of the samples inside the window. The anomaly test becomes

\[
|x_i-m_w|>k\cdot 1.4826 \cdot MAD
\]

When the condition is satisfied, the detected anomaly is replaced with the local median. The factor \(1.4826\) rescales the MAD so that it is comparable to the standard deviation under Gaussian assumptions. Since median and MAD are much less sensitive to outliers than mean and standard deviation, the Hampel filter is expected to be more robust when the signal is affected by impulsive spikes.


#### Adaptive sampling frequency before and after filtering

The graph compares the adaptive sampling rates obtained after the FFT is computed in three cases: dirty signal, signal filtered with Z-score and signal filtered with Hampel.

![Adaptive sampling rate after Z-score and Hampel filtering](docs\fig4_adaptive_rate.png)

The first clear result is that the dirty signal always leads to very high adaptive rates, close to the oversampling regime. The reason is that a spike is a very abrupt variation in the time domain which produces in the spectrum energy spread over a wide frequency range. Because of this, the FFT interprets the dirty signal as containing much faster components than the original sinusoid, and the estimated \(f_{\max}\) becomes artificially high.

The Z-score results are not satisfactory. As already discussed, this filter is not well suited to impulsive spikes. For small \(p\), increasing the window size gives only a slight improvement: the effect of a single spike is diluted over a larger moving window, but the spike also stays inside that window for longer, so its impact is not removed completely.

The Hampel filter behaves much better, but its performance depends strongly on the window size. With \(W=5\), the recovery is almost perfect only for \(p=0.01\). When \(p\) increases, the window becomes too small: multiple nearby outliers can affect the local median and reduce the effectiveness of the filter, so the adaptive rate remains high.

With \(W=15\), Hampel gives the best overall behaviour in this graph. For low contamination it brings the adaptive rate very close to the ideal one, and even when \(p\) increases it still performs clearly better than Z-score. This suggests that the window is large enough to make the median and MAD stable, while still remaining local enough to follow the waveform.

With \(W=40\), the filter still removes a relevant part of the spike contamination, but the result is no longer as good as with \(W=15\). In this case the window is robust enough to detect many spikes, but it becomes less local. As a consequence, the replacement values are less tied to the instantaneous shape of the signal, and the filtered waveform may still contain distortions or local discontinuities. These are enough to leave residual high-frequency content in the spectrum, so the FFT still returns an adaptive rate significantly above the ideal one.

The case \(W=100\) is particularly interesting. Since the signal frequency is \(5\) Hz and the oversampling frequency is \(500\) Hz, one period of the clean signal is represented by exactly \(100\) samples. This means that the filter window spans one full cycle of the waveform. For a sinusoidal signal ranging between \(0\) and \(3.3\) V, the median of the window is approximately \(1.65\) V. Therefore, when a spike is detected, it is replaced by a value close to \(1.65\) V independently of the local phase of the signal. If the neighbouring samples are far from this value, the replacement introduces a sharp local transition. This abrupt step injects new high-frequency components into the spectrum, causing again a strong FFT response at high frequencies. 


#### Detection performance

![TPR and FPR of Z-score and Hampel filters under different anomaly probabilities and window sizes](docs/fig1_detection.png)

The graph shows that the **Z-score filter** has a **decreasing TPR**. The reason is that spikes increase the local mean and standard deviation, and therefore also the detection threshold; as a result, many anomalies are no longer detected. Using larger windows slightly mitigates the impact of a single spike, but not enough to obtain good detection performance. On the other hand, the **FPR is null in all cases**. This is the other side of the same effect: the filter is very conservative and rarely classifies normal samples as anomalies.

The **Hampel filter** shows much better detection performance overall, with **high TPR** and **low FPR** in most configurations. The results degrade only slightly for small windows, because multiple nearby outliers are more likely to contaminate the local median and, consequently, the detection threshold. In these cases, some anomalies may escape detection. 


#### Mean Error Reduction (MER)

![Mean Error Reduction of Z-score and Hampel filters under different anomaly probabilities and window sizes](docs/fig2_mer.png)

To evaluate how well the filters reconstruct the clean signal, the **Mean Error Reduction (MER)** is used:

\[
MER = 1 - \frac{MSE_{\text{filtered}}}{MSE_{\text{dirty}}}
\]

where \(MSE_{\text{dirty}}\) is the mean squared error between the dirty and clean signal, while \(MSE_{\text{filtered}}\) is the mean squared error between the filtered and clean signal. 

The graph confirms that the **Z-score filter** has limited reconstruction capability. Its MER decreases as the anomaly probability increases, and the worst case is again the small window \(W=5\), where the filter is almost unable to improve the signal. Larger windows slightly mitigate the effect of the spikes, but the overall reconstruction quality remains unsatisfactory, especially for higher contamination.

The **Hampel filter** instead shows consistently high MER values, confirming that it is much more effective in restoring the clean waveform. The only clear degradation appears for \(W=5\) when \(p\) becomes large, because with such a small window nearby outliers can more easily contaminate the local median and reduce the effectiveness of the correction. 

This graph also highlights an important point: a high MER does not automatically imply a correct recovery of the FFT-estimated `f_max`. MER measures the average time-domain reconstruction quality, while `f_max` is much more sensitive to residual local discontinuities and high-frequency artefacts.


#### Execution time and computational trade-off

![Execution time of Z-score and Hampel filters under different anomaly probabilities and window sizes](docs/fig3_exec.png)

For both filters, the execution time increases with the window size, since the sliding window must be processed at every sample. The increase is much steeper for Hampel, because it relies on repeated sorting operations to compute median and MAD, whereas Z-score only computes mean and standard deviation.

The only exception is \(W=5\), where Hampel is slightly faster. In this case, sorting only a few samples is still very cheap, while Z-score still has to perform the same local computations for every sample. As the window becomes larger, the sorting cost grows rapidly and Hampel becomes significantly more expensive.

This result is important because longer execution time means the CPU remains active for more time, which increases energy consumption. In addition, larger windows also require more temporary storage, so the trade-off is not only in time, but also in memory. Overall, Hampel provides better robustness, but at a higher computational, energy and memory cost.