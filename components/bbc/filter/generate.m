clc;
clear;
close all;

FILE = fullfile("40p_1000ms_80kHz.csv");

RESOLUTION = 2^12;

fs = 80e3;

M = readmatrix(FILE);

t = M(:, 1) ./ fs;
s = M(:, 2) ./ (RESOLUTION - 1);

t = downsample(t, 4);
s = downsample(s, 4);

fs = 20e3;
fn = fs / 2;

S = fft(s);
S = S(1:floor(end / 2));
w = linspace(0, pi, numel(S));
f = (w * fs) / (2 * pi);

mask = f >= 5;
S = S(mask);
f = f(mask);

S_dB = 20 * log10(abs(S));
dB_max = max(S_dB);

elements = f <= 100;

Wp = 30 / fn;
Ws = 50 / fn;
Rp = 3;
Rs = 30;

[n, Wn] = ellipord(Wp, Ws, Rp, Rs);
[z, p, k] = ellip(n, Rp, Rs, Wp);

[sos, g] = zp2sos(z, p, k);

f_response = linspace(0, Ws * fn + 20, 1e3);
h = g * freqz(sos, f_response, fs);

h_dB = 20 * log10(abs(h));

% ESP32S3 only has hardware support for single-point
s_filtered = single(g) * sosfilt(single(sos), single(s));

display(sos);
display(g);

figure;
figure_title = {
                'Brake Pressure Data w/ No Hardware Filter '
                '(fs = 20kHz, 12-bit Resolution, Normalized)\n'
                '40%% Pressure w/ Hydrastar, 10ms Delayed Start, 1000ms Hold'
               };
sgtitle(sprintf(strjoin(figure_title)));

subplot(2, 2, 1);
plot(t, s);
ylim([0.3, 0.9]);
title('Raw Data');
xlabel('Time [s]');

subplot(2, 2, 2);
plot(f(elements), S_dB(elements));
yline(dB_max - 20, '--', '-20dB');
yline(dB_max - 40, '--', '-40dB');
title('Raw Data FFT');
xlabel('Frequency [Hz]');
ylabel('Magnitude [dB]');

subplot(2, 2, 3);
plot(f_response, h_dB);
xline(Wp * fn, '--', 'Passband');
xline(Ws * fn, '--', 'Stopband');
xlim(f_response([1, end]));
yline(-Rp, '--', 'Passband');
yline(-Rs, '--', 'Stopband');
ylim([-Rs - 60, 0]);
title(sprintf('Elliptic Lowpass Filter (n = %d)', n));
xlabel('Frequency [Hz]');
ylabel('|H| [dB]');

subplot(2, 2, 4);
plot(t, (s_filtered - 0.45) ./ (0.7 - 0.45));
ylim([0, 1]);
title('Filtered Data (Normalized)');
xlabel('Time [s]');
