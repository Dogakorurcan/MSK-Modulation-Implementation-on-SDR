%% MSK
num_bits_data = 500;
Sps = 8;                   % Samples Per Symbol
%% Zero Padding 100bit
zero_Len = 100 * Sps;
tx_zeros = zeros(zero_Len, 1);
%% CLOCK SYNC & PREAMBLE & PILO
Sync_Len = 64;
% clock sync
tx_sync_bits = repmat([1; 0], Sync_Len/2, 1);
% Preamble 63 bit
rng(999);
tx_preamble_bits = randi([0 1], 63, 1);
pilot_Len = 32;
rng(12345);
tx_pilot_bits = randi([0 1], pilot_Len, 1);
rng(123)
data_bits = randi([0 1], num_bits_data, 1);
save('tx_data_bit.mat', 'data_bits')
%% 3. FRAME
tx_frame_bits = [tx_sync_bits; tx_preamble_bits; tx_pilot_bits; data_bits; tx_pilot_bits];
%% 4. MSK
fprintf('1. Modulation: Converting data to MSK waveform...\n');
tx_baseband = mskmod(tx_frame_bits, Sps);
% Zero padding
tx_waveform = [tx_zeros; tx_baseband; tx_zeros];
%% 5. DDC
masterclockrate = 20e6;
interpolate = 256;
Fs = masterclockrate / interpolate; % Fs = 78.125 kHz
f_IF = 30e3;  % 30 kHz (IF)
fprintf('2. DDC: Shifting signal to %d Hz intermediate frequency (IF)...\n', f_IF);
N_tx = length(tx_waveform);
t_tx = (0:N_tx-1).' / Fs;
% Mixing
mixer_signal = exp(+1j * 2 * pi * f_IF * t_tx);
tx_IF_waveform = tx_waveform .* mixer_signal;
%% 6. USRP
fprintf('Setting up USRP connection...\n');
tx_radio = comm.SDRuTransmitter(...
    'Platform',             'B200', ...
    'SerialNum',            '31FD9D5', ...
    'CenterFrequency',      400e6, ...
    'MasterClockRate',      masterclockrate, ...
    'Gain',                 50, ...
    'InterpolationFactor',  interpolate, ...
    'TransportDataType',    'int16');
fprintf('TX ready. Will transmit continuously, resetting each iteration.\n');
input('Press ENTER to start >> ');
while true
    tx_radio(tx_IF_waveform);
    % pause(0.1);
end
