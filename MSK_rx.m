%% FULL STREAMING RX – MSK ADAPTATION
clc; clear; close all;

num_bits_data = 500;           % MSK binary (1 bit per symbol)
Sps = 8;
Fs = 20e6/256;                 % 78.125 kHz
f_IF = 30e3;
pilot_Len = 32;
preamble_Len = 63;

rng(999);
tx_preamble_bits = randi([0 1], preamble_Len, 1);
tx_preamble_wave = mskmod(tx_preamble_bits, Sps);

rng(12345);
tx_pilot_bits = randi([0 1], pilot_Len, 1);
tx_pilot_wave = mskmod(tx_pilot_bits, Sps);

%% ================= 3. SPECTRUM ANALYZERS =================
specIF = spectrumAnalyzer('SampleRate', Fs, 'Title', 'RX IF', 'YLimits', [-140 -40]);
specBB = spectrumAnalyzer('SampleRate', Fs, 'Title', 'RX Baseband', 'YLimits', [-140 -40]);

%% ================= 4. USRP RX =================
rx_radio = comm.SDRuReceiver( ...
    'Platform','B200', ...
    'SerialNum','31FD9C8', ...
    'CenterFrequency',400e6, ...
    'MasterClockRate',20e6, ...
    'DecimationFactor',256, ...
    'SamplesPerFrame',15000, ...
    'OutputDataType','double', ...
    'Gain',55);

disp('=== STREAMING RX STARTED ===');

%% ================= STREAMING BUFFERS =================
rx_buffer = complex([]);
MAX_BUF = Fs * 2;
FRAME_FOUND = false;

%% ================= MAIN RX LOOP =================
 while true
    %% ---- 1) RECEIVE STREAM ----
    rx_new = rx_radio();
     if isempty(rx_new);
         continue;
     end
    specIF(rx_new);

    rx_buffer = rx_new;
    if length(rx_buffer) > MAX_BUF
        rx_buffer = rx_buffer(end-MAX_BUF+1:end);
    end

    %% ---- 2) DDC ----
    t = (0:length(rx_buffer)-1).' / Fs;
    rx_bb = rx_buffer .* exp(-1j*2*pi*f_IF*t);
    specBB(rx_bb);

    %% ---- 3) TIMING RECOVERY ----

    %% ---- 4) CFO ESTIMATION (MSK SPECIFIC) ----
    Rb = Fs / Sps; % Bit Rate calculation

    x4 = rx_bb.^4;
    psd = abs(fftshift(fft(x4)));
    f_axis = linspace(-Fs/2, Fs/2, length(psd));

    % Find the tallest peak
    [~, pidx] = max(psd);
    f_max = f_axis(pidx);

    % The distance between the two MSK x4 peaks is exactly 2*Rb.
    % We check both sides of f_max to find where the "twin" peak is located.
    [~, idx_minus] = min(abs(f_axis - (f_max - 2*Rb)));
    [~, idx_plus]  = min(abs(f_axis - (f_max + 2*Rb)));

    if psd(idx_minus) > psd(idx_plus)
        % The twin peak is below f_max. Therefore, f_max is the upper peak.
        f_center = f_max - Rb;
    else
        % The twin peak is above f_max. Therefore, f_max is the lower peak.
        f_center = f_max + Rb;
    end

    % The true carrier offset is exactly in the middle
    CFO = f_center / 4;

    n = (0:length(rx_bb)-1).' / Fs;
    rx_bb_cfo = rx_bb .* exp(-1j*2*pi*CFO*n);
    fprintf('    TRUE CFO = %.2f Hz\n', CFO);

    %% ---- 5) FRAME SYNCHRONIZATION ----
    [corr_val, lags] = xcorr(rx_bb_cfo, tx_preamble_wave);
    [peak_val, peak_idx] = max(abs(corr_val));

    THRESH = 10 * mean(abs(corr_val));

    if peak_val < THRESH
        continue;
    end

    % FIX: MATLAB is 1-indexed. A cross-correlation lag of N means
    % the signal starts at index N+1.
    start_idx = lags(peak_idx) + 1;

    if start_idx < 1
       start_idx = 1;
    end
    fprintf('\n>>> FRAME FOUND! Lag: %d | Peak: %.2f\n', start_idx-1, peak_val);
    %% ---- 6) BULK FRAME EXTRACTION ----
    total_frame_bits = preamble_Len + pilot_Len + num_bits_data + pilot_Len;
    total_frame_samples = total_frame_bits * Sps; % 627 * 8 = 5016

    if start_idx + total_frame_samples - 1 > length(rx_bb_cfo)
        fprintf('   -> Buffer insufficient, frame end missing.\n');
        continue;
    end

    rx_frame_wave = rx_bb_cfo(start_idx : start_idx + total_frame_samples - 1);
    %% ---- 7) BULK PHASE CORRECTION ----
    bulk_phase_offset = angle(corr_val(peak_idx));
    rx_frame_corrected = rx_frame_wave .* exp(-1j * bulk_phase_offset);
    %% --- EYE DIAGRAM ---
    eyediagram(rx_frame_corrected(1:800), Sps*2);

    %% ---- 8) DEMODULATION ----
    % Demodulate the entire signal. mskdemod uses the Viterbi algorithm.
    rx_all_bits = mskdemod(rx_frame_corrected, Sps);

    % 1. Raw Signal from USRP
    scatterplot(rx_new);
    title('1. Raw Received Signal (rx\_new)');

    % 2. Baseband Signal (Shifted to 0 Hz after DDC)
    scatterplot(rx_bb);
    title('2. Baseband Signal (rx\_bb)');

    % 3. CFO (Carrier Frequency Offset) Corrected Signal
    scatterplot(rx_bb_cfo);
    title('3. CFO Corrected (rx\_bb\_cfo)');

    % 4. Extracted Frame after Synchronization
    scatterplot(rx_frame_wave);
    title('4. Extracted Frame (rx\_frame\_wave)');

    % 5. Bulk Phase Corrected (De-rotated) Frame
    % This is where the MSK/GMSK circle should be seen most clearly
    scatterplot(rx_frame_corrected);
    title('5. Phase Corrected Frame (rx\_frame\_corrected)');

    % 6. Demodulated Bits
    % Note: These are no longer IQ symbols, but 1s and 0s, so they cluster only at (0,0) and (1,0).
    scatterplot(rx_all_bits);
    title('6. Demodulated Bits (rx\_all\_bits)');
    %% ---- 9) BIT SLICING ----
    bit_idx_data_start = preamble_Len + pilot_Len + 1;
    bit_idx_data_end   = bit_idx_data_start + num_bits_data - 1;

    rx_data_bits = rx_all_bits(bit_idx_data_start : bit_idx_data_end);

    disp('--- DATA DECODED ---');
    disp('First 50 bits:');
    disp(rx_data_bits(1:50).');

    FRAME_FOUND = true;
    break
 end
rng(123)
release(rx_radio);
disp('=== RX SESSION FINISHED ===');
data_bits = randi([0 1], num_bits_data, 1);

%% ---- 10) BER ----
%load('tx_data_bit.mat', 'data_bits');

tx_bits_actual = data_bits(:);
rx_data_bits = rx_data_bits(:);

min_len = min(length(tx_bits_actual), length(rx_data_bits));
tx_bits_actual = tx_bits_actual(1:min_len);
rx_data_bits = rx_data_bits(1:min_len);

tx_bits_actual = logical(tx_bits_actual);
rx_data_bits = logical(rx_data_bits);

[numErrors, ber] = biterr(tx_bits_actual, rx_data_bits);

fprintf('\n================================================\n');
fprintf('   BER ANALYSIS RESULTS:\n');
fprintf('   Total Bits Transmitted : %d\n', length(tx_bits_actual));
fprintf('   Bit Errors              : %d\n', numErrors);
fprintf('   Bit Error Rate (BER)    : %.4f (%%%.2f)\n', ber, ber*100);
fprintf('================================================\n');
