# MSK Transmitter / Receiver over USRP B200

A MATLAB implementation of a Minimum Shift Keying (MSK) transceiver, built around USRP B200 software-defined radios via the Communications Toolbox Support Package for USRP Radio. The transmitter builds and streams an MSK frame; the receiver performs streaming acquisition, carrier frequency offset (CFO) estimation, frame synchronization, and demodulation, and reports the resulting bit error rate (BER).

## Overview

MSK is a continuous-phase, constant-envelope binary modulation scheme (a special case of CPFSK with a modulation index of 0.5). This project sends a fixed frame structure over the air at a 400 MHz carrier and recovers it on the receive side without any external trigger or wired synchronization between the two radios — synchronization, carrier offset correction, and framing are all recovered purely from the received waveform.

## Files

- `MSK_tx.m` — builds the frame (sync + preamble + pilot + data + pilot), applies MSK modulation, upconverts to a digital IF, and streams it continuously through a USRP B200 transmitter.
- `MSK_rx.m` — continuously receives samples from a second USRP B200, down-converts to baseband, estimates and corrects CFO, detects the frame via cross-correlation, corrects the residual phase offset, demodulates the bits, and computes the BER against the known transmitted data.

## Frame structure

| Field     | Length     | Purpose                                         |
|-----------|------------|--------------------------------------------------|
| Sync      | 64 bits    | Alternating `1010...` pattern for clock sync     |
| Preamble  | 63 bits    | Fixed PN sequence (`rng(999)`) used for frame detection via cross-correlation |
| Pilot     | 32 bits    | Fixed PN sequence (`rng(12345)`) |
| Data      | 500 bits   | Payload bits (`rng(123)`, saved to `tx_data_bit.mat`) |
| Pilot     | 32 bits    | Repeated at the end of the frame |

The modulated frame is zero-padded (100 symbols on each side) before transmission to give the receiver's streaming loop clean silence to lock onto.

## Signal chain

**TX:** bits → `mskmod` (8 samples/symbol) → zero padding → digital upconversion to a 30 kHz IF → USRP B200 (`comm.SDRuTransmitter`, 400 MHz center frequency, 20 MHz master clock rate, interpolation factor 256).

**RX:** USRP B200 (`comm.SDRuReceiver`, matching center frequency and clock settings) → digital down-conversion (mix with the IF) → **CFO estimation**, using the classic 4th-power method exploited by MSK's spectral structure (raising the baseband signal to the 4th power produces two tones separated by `2×Rb`, whose midpoint gives the true carrier offset) → CFO correction → **frame synchronization** via cross-correlation against the known preamble waveform → bulk phase correction (de-rotation using the correlation peak's phase angle) → `mskdemod` (Viterbi-based) → bit slicing → BER computation against the known data bits.

## Requirements

- MATLAB with the Communications Toolbox
- Communications Toolbox Support Package for USRP Radio
- 2× USRP B200 (or compatible) SDR hardware, one for TX and one for RX
- USB 3.0 connections recommended for stable streaming at these sample rates

## Usage

1. Update the `SerialNum` fields in both scripts to match your own USRP devices.
2. Run `MSK_tx.m` first, and press Enter when prompted — it will stream the frame continuously.
3. Run `MSK_rx.m` on a second machine/radio. It will report the estimated CFO, the frame detection lag, show constellation/eye-diagram plots at each stage of the pipeline, and print the final BER once a frame is successfully captured and decoded.

## Notes / limitations

- The RX loop currently processes only the most recent buffer of samples on each iteration rather than accumulating a rolling window (`rx_buffer = rx_new`), so a frame that straddles two acquisition calls can be missed. Accumulating samples across iterations (with the existing `MAX_BUF` cap) would make detection more robust.
- Timing recovery is bypassed — `mskdemod`'s built-in Viterbi decoding handles this implicitly rather than through an explicit symbol timing loop (e.g. Mueller & Müller, Gardner, or Meyr & Oerder).
- The BER block currently regenerates a fresh random `data_bits` vector with `rng(123)` after the RX loop instead of loading the exact vector saved by the transmitter (`tx_data_bit.mat`); since both sides use the same seed this happens to line up, but loading the saved `.mat` file directly would be more robust.

## License

Add a license file (MIT is a common, permissive choice for portfolio/educational projects) before publishing if you intend this repository to be reused by others.
