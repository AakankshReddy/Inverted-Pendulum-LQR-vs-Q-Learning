# Adaptive Noise Cancellation Using NLMS
### A Research-Oriented Investigation into Practical Adaptive Filtering — With Measured Results

---

## Overview

This project implements and evaluates LMS, NLMS, and VSS-NLMS adaptive filters for
dual-microphone Adaptive Noise Cancellation (ANC) in MATLAB, using a near
microphone (desired speech + noise) and a far/reference microphone (noise only, or
noise plus some leaked speech depending on placement).

The project did not stop at "implement the algorithm and report an SNR number."
Early on, the measured SNR improvement and what the output actually sounded like
diverged — a configuration that scored well numerically sounded clearly worse than
the raw input. Rather than treating that as noise in the process, it was treated as the
actual research question: **why does a standard evaluation metric say one thing while
a human listener says another, and what does that reveal about how the filter is
actually behaving?**

The bulk of this README documents that investigation end to end — the hypotheses
that were proposed, how each was tested, which were rejected, which was confirmed,
and what the corrected result looks like. Every number quoted below is a real measured
value from an actual script run against real or synthetic signals generated for this
project. None of it is illustrative or filled in for effect.

---

## Problem Setup

- **Near mic**: `d(n) = s(n) + v(n)` — desired speech + noise
- **Far mic**: `x(n)` — reference signal, correlated with `v(n)`
- **Goal**: adapt an FIR filter to estimate `v̂(n)`, recover `e(n) = d(n) − v̂(n) ≈ s(n)`

NLMS update:

```
w(n+1) = w(n) + [μ / (ε + ‖x(n)‖²)] · e(n) · x(n)
```

---

## Investigation Timeline and Measured Results

### 1. Initial dual-mic recording — real hardware

LMS and NLMS were first implemented and run on real near/far microphone recordings
captured with laptop microphones, roughly 60 seconds long, containing speech over
background noise.

The first result was a contradiction: the computed SNR improvement suggested the
filter was working, but listening to the output made the improvement hard to notice —
in some cases the processed audio sounded no better, or worse, than the raw input.
This mismatch was not dismissed as a fluke; it became the central question the rest of
the project tries to answer.

Before getting to the SNR investigation itself, three separate, real hardware/setup
issues were identified experimentally as contributing factors:

**Laptop mic pre-processing (AGC / noise gate).** Consumer laptop microphones apply
automatic gain control and noise gating in hardware/driver level before any signal
reaches MATLAB. This means the reference signal fed into the adaptive filter had
already been partially cleaned by the hardware itself — the algorithm was adapting to a
signal that no longer contained the full noise content it was supposed to model. This is
a known and documented problem in acoustic echo cancellation systems, sometimes
called the pre-conditioning problem, and it was rediscovered here experimentally rather
than assumed from the literature.

**Mic geometry and reference leakage.** When the far (reference) microphone was
positioned such that it also picked up meaningful energy from the desired speech
signal — not just the noise source — the adaptive filter began partially cancelling the
speech itself, since from the filter's perspective, anything correlated between the two
microphones looks like noise to be removed. Careful mic placement, with the reference
microphone close to the noise source and far from the speaker, measurably reduced
this effect.

**The stationarity assumption.** LMS-family filters assume the statistics of the noise
they are cancelling do not change faster than the filter's own adaptation rate. This
held up well for broadband and tonal noise (fan hum, background hiss) but broke down
for impulsive, sudden noise events (door slams, keyboard clicks) — the filter's weights
were still adjusting to the event by the time it had already ended, so impulsive noise
consistently passed through uncancelled regardless of step size.

---

### 2. Parameter sweep — filter length × step size

To move past manual trial and error, a grid search was run over filter length
`M ∈ {32, 64, 128, 256}` and NLMS step size `μ_nlms ∈ {0.2, 0.4, 0.7, 1.0, 1.3, 1.6}`,
20 combinations total, each scored by the resulting global SNR improvement on the
same recording.

**Top five results from the sweep, ranked by SNR:**

| M | μ_nlms | SNR (dB) |
|---|---|---|
| **256** | **0.700** | **+7.39** |
| 128 | 0.700 | +6.88 |
| 256 | 0.400 | +6.12 |
| 64 | 0.700 | +5.84 |
| 128 | 0.400 | +5.66 |

Taken at face value, M=256 and μ=0.7 was the winning configuration by a clear margin
— nearly a full decibel ahead of the next best result. This is the configuration that was
then listened to directly.

---

### 3. The SNR number did not match perceptual quality

Listening to the M=256, μ=0.7 output directly: **NLMS sounded clearly worse than the
unprocessed input** — described during testing as "terrible," warbly, and distorted —
despite the metric reporting a +7.39 dB improvement, the best result in the entire
sweep. Rather than trusting either the metric or the listening impression on its own,
three competing explanations were written down as hypotheses and tested one at a
time, in order, so the answer would be based on evidence instead of a guess.

**Hypothesis A — Silence inflation.**
The concern: if the SNR calculation is a single number averaged over the whole clip, a
filter that does almost nothing during speech but crushes noise very effectively during
silent stretches could still produce a high average, even though the part a listener
actually attends to (the speech) barely improved.

To test this, the recording was split into 250 ms frames, each classified as "active"
(speech-containing) or "quiet" (silence/background noise only), and SNR was
computed separately for each group instead of one blended average.

```
Global SNR:          7.39 dB
Active-frame SNR:    7.91 dB
Quiet-frame SNR:     6.77 dB
```

*Result: rejected.* If silence inflation were the explanation, quiet frames should have
scored dramatically higher than active frames. Instead active frames scored slightly
*higher* — the opposite of what this hypothesis predicts. This ruled out silence
inflation as the cause and pointed the investigation elsewhere.

**Hypothesis B — Uniform attenuation being mistaken for cancellation.**
The concern: the SNR formula being used was essentially a ratio of total power before
and after filtering. If the filter were simply turning down the volume of everything —
speech and noise together, roughly proportionally — total power would drop and the
metric would report "improvement," even though nothing was being selectively
removed. This would explain the earlier subjective description of the output as
"softened."

To test this, the output was rescaled to match the input's RMS level, and the
correlation between the rescaled output and the original input was computed. If the
filter were just attenuating everything uniformly, the rescaled output should look
almost identical to the input (correlation close to 1.0).

```
RMS input:   0.1122
RMS output:  0.0479
Ratio:       0.43    (a −7.4 dB drop in overall level)
Correlation (input vs. rescaled output): 0.232
```

*Result: rejected.* A correlation of 0.232 is far too low for the output to be a simple
volume-reduced copy of the input. Real reshaping of the signal was happening, not a
blanket turn-down — so this wasn't simple attenuation either.

**Hypothesis C — Filter instability, producing "musical noise."**
With the first two explanations ruled out, the remaining candidate was that the filter
itself was unstable: rather than converging to a good estimate of the noise path and
settling there, it might be continuously and aggressively re-adjusting its coefficients
on every sample, producing audible warbling artifacts sometimes called musical noise
in the adaptive-filtering literature — a known failure mode at excessively high step
sizes.

To test this, the average magnitude of the filter's weight updates in the last portion of
the recording was compared against the average magnitude in the first portion. A filter
that has converged should show much smaller updates later in the recording than at
the very start, since it has already found a good solution and is only fine-tuning.

```
Late/early weight-update ratio: 1.04
```

*Result: confirmed.* A ratio near 1.0 means the filter was adjusting its coefficients just
as aggressively 60 seconds in as it was at the very first sample — it never settled into a
stable solution at any point during the recording. This is the mechanism behind the
inflated SNR and the poor perceptual quality at the same time: on every sample, the
filter overshoots and corrects again, which reduces total signal energy (driving the
naive SNR metric up) while actively distorting the waveform (making it sound
audibly worse). The metric was technically measuring something real — power was
reduced — it just wasn't measuring the thing anyone actually cared about.

---

### 4. Fix: reduce step size, re-verify

Dropping `μ_nlms` from 0.7 → 0.1 and re-running the same diagnostics:

```
Global SNR:          2.89 dB   (down from 7.39 dB)
Active-frame SNR:    3.08 dB
Quiet-frame SNR:     2.42 dB
```

The absolute SNR number is lower — but it is now the *honest* number. Active frames
again outscore quiet frames (consistent, not a red flag), and the filter is no longer in
the unstable regime that produced musical noise. This is the core finding of the
project:

> **The parameter combination that maximized the evaluation metric (μ=0.7, +7.39 dB)
> was the same combination causing filter instability. Optimizing for the metric alone
> produced a worse-sounding system than a lower, "worse-scoring" configuration.**

---

## Why This Matters

A single power-ratio SNR number cannot distinguish between:
1. Genuine, selective noise cancellation
2. Uniform signal attenuation
3. Unstable filter behavior that reduces total energy while distorting the signal

This project shows that a passing metric is not sufficient evidence of correct behavior,
and lays out a concrete diagnostic procedure (segment-wise SNR → attenuation
correlation check → weight-convergence check) for telling these cases apart.

---

## System Architecture

```
Clean Speech
     │
     ▼
Acoustic Mixing
     │
 ┌───┴────┐
 ▼        ▼
Near Mic  Far Mic (reference)
 │        │
 │        ▼
 │   Preprocessing
 │   • Normalization
 │   • Delay estimation (cross-correlation)
 │   • Alignment
 │
 └──────┐
        ▼
  NLMS Adaptive Filter (block-wise, 512-sample chunks, state persists across blocks)
        │
        ▼
  e(n) = Estimated Speech
        │
        ▼
  Multi-Metric Evaluation
  (segment-wise SNR, attenuation-vs-cancellation check, weight-convergence check)
```

---

## Extended Stress Tests (Synthetic, Ground-Truth-Verified)

Beyond the real-recording investigation above, three synthetic test harnesses were
built to isolate specific failure modes with known ground truth:

| Script | Isolates | Method |
|---|---|---|
| `anc_realtime_test.m` | Effect of mic misalignment | Injects a known 23-sample delay, auto-corrects via cross-correlation, compares aligned vs. misaligned on noise-reduction / correlation-to-truth metrics |
| `anc_realtime_test_hard.m` | Time-varying difficulty | Noise level ramps 2.2× and reference leakage jumps 3%→12% mid-recording; results are segmented before/after the transition |
| `anc_functional_test.m` | Linear filter limits | Reference channel corrupted with a **nonlinear** leakage term `f1(t)·f3(t)`; since NLMS is a linear filter, this measures the theoretical ceiling on cancellation when the corruption itself is nonlinear |

All three use a known clean desired signal (`f1`, built from band-passed random noise
— non-periodic, so the filter cannot "cheat" off of a fixed period) so recovery
correlation and RMSE against ground truth are exact, not estimated.

---

## Evaluation Methodology (Final)

Rather than a single metric, the project uses:

1. **Segment-wise SNR** (active vs. quiet frames) — flags silence-inflation
2. **Rescaled-output correlation to input** — flags uniform attenuation vs. real cancellation
3. **Weight-update convergence ratio** (late vs. early) — flags filter instability
4. **Correlation to ground-truth desired signal** (synthetic tests only) — measures true speech preservation, not just noise removal

---

## Limitations

- Single-channel ANC, linear FIR filter only
- Real-recording results are from one hardware setup (laptop dual-mic) — not generalized across devices
- Stable configuration (μ=0.1) trades peak SNR for convergence; no systematic sweep was done to find the true stability boundary between μ=0.1 and μ=0.7
- Perceptual validation used informal listening plus a blind A/B script; not a full MUSHRA-style study with multiple listeners

## Future Work

- VSS-NLMS with the convergence-ratio check built into the adaptation rule itself (adapt μ down automatically as instability is detected)
- RLS comparison
- Formal blind A/B study with more listeners for a statistically defensible perceptual result
- Real-time streaming implementation (`audioDeviceReader`/`Writer`) rather than block-simulated real-time

---

## Skills Demonstrated

Digital signal processing · adaptive filtering (LMS/NLMS/VSS-NLMS) · experimental
methodology and hypothesis testing · diagnostic instrumentation of a metric's failure
mode · MATLAB development · scientific debugging · audio signal processing

---

## Resume Bullet

> Implemented and evaluated LMS/NLMS/VSS-NLMS adaptive filters for dual-mic noise
> cancellation in MATLAB; discovered that the parameter setting maximizing measured
> SNR (+7.39 dB) was perceptually worse than the input, diagnosed the cause via
> segment-wise SNR, attenuation-correlation, and weight-convergence analysis —
> isolating filter instability (late/early weight-update ratio ≈ 1.0) as the mechanism;
> corrected by reducing step size (final stable result: 2.89 dB, converged), establishing
> that global SNR alone is an unreliable ANC evaluation metric.
