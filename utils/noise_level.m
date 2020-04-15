%
% matlab script to plot wispr noise level
% based on a script that Haru made for Wispr
% - cjones Jan 2018
%

clear all;

[file, dpath, filterindex] = uigetfile('./*.dat', 'Pick a waveform file');
name = fullfile(dpath,file);

% open file
format = 'ieee-le';
fp = fopen( name, 'r', format );

q = 5.0/8388608.0;  % ltc2512 scaling from datasheet

hydrosens = -178.3; % HTI 92 WB
gain = 0;

while (1)
    
count = 0;
data = [];

while( count < 10 )

  [hdr, raw] = wispr_read(fp);
  if(isempty(raw)) 
      break; 
  end
  data = [data; double(raw)*q]; % concatenate raw data buffer into one dat vector
    
end

fs = hdr.sampling_rate;
sig = data - mean(data);
t = (1:length(sig))/fs;

% power spectrum
nfft = 512;

%h = spectrum.welch;                  % Create a Welch spectral estimator.  
%set(h, 'SegmentLength', nfft);
%Hpsd = psd(h, sig, 'Fs',fs);             % Calculate the PSD 
%psd = Hpsd.Data;
%psd = 2*psd/fs;
%freq = Hpsd.Frequencies;

% my new power spectrum method
noverlap = nfft/2;
win = hanning(nfft);
%win = boxcar(nfft);
[psd, freq] = psd(sig,nfft,fs,win,noverlap);

db = 10*log10(psd);

figure; 
plot(freq, 10*log10(psd));
%axis([0 fs/2 20 120]);
strn=sprintf('power spectral density');
ylabel('Spectrum Level in dB re 1V ^2/Hz');
xlabel('Frequency [Hz]');
title(strn);
grid on;

%Remove preamp gain
% using measured gain from Haru
FrqSys = [1 2 5 10 20 50 100 200 500 1000 2000 5000 10000 20000 30000 40000 50000 60000 70000 80000 90000 100000 110000 120000 130000 140000 150000 160000 170000 180000 190000 200000];
PAGain = [1.0 8.3 14.2 16.1 16.5 16.9 17.1 17.9 20.3 24.7 29.9 37.1 41.9 45.1 45.9 46.3 46.3 46.2 46.1 46.0 45.7 45.5 45.2 45.0 44.7 44.5 44.2 44.0 43.6 43.3 43.1 42.8];
%PAGain = [0.0 7.8 14.2 16.1 16.7 16.9 17.2 17.8 20.4 24.5 29.5 37.0 41.9
%45.0 45.7 46.4 46.4 46.3 46.2 46.0 45.8 45.5 45.3 45.1 44.8 44.6 44.3 44.0 43.7 43.4 43.2 42.9];
%PAGain = [-4.4 -4.0 12.1 16.3 18.6 19.5 19.8 20.4 22.8 26.4 31.4 38.8 43.6  47.3  48.5  49.0  49.1  49.2  49.2  49.2  49.1  48.8  48.5  48.6   48.0   47.6];
%FrqSys=    [1    2   5   10  20   50   100  200  500  1000 2000 5000 10000 20000 30000 40000 50000 60000 62500 64500 70000 80000 90000 100000 110000 120000];

PAGainI = interp1(FrqSys,PAGain, freq,'pchip'); %interpolate
    
% total system sensitivity
SysSens = hydrosens + PAGainI + gain;

%figure(3); 
%plot(freq, SysSens);
%ylabel('System Sensitivity in dB re 1\muPa');
%xlabel('Frequency [Hz]');
%grid on;

%plot(freq, psd); % Plot the PSD.

Noise = 10*log10(psd) - SysSens;

% plot noise again sea state and ship noise
figure; clf;

hold on;
plot(freq, Noise, '.-', 'LineWidth', 2, 'Color','k');

frqW = [ 100. 200. 300. 500. 700 1000. 2000. 5000. 10000. 20000. 50000.];
%SeaSt0_Wenz = [ 39.0 44.0 56.  56.  54. 52.   47.   41.   36.    31.    24.];
SeaSt1_Wenz = [ 50.5 54.  56.  56.  54. 52.   47.   41.   36.    31.    24.];
SeaSt0_Wenz = SeaSt1_Wenz - 10;
SeaSt2_Wenz=[ 63.  66.  67.  66.  65. 63.   57.5  50.5  45.5    40.    33.5];
SeaSt6_Wenz = [71.   72.5 73.7  73.0 71.5 68.5  64.   57.   52.    47.    40.];
frqShip = [10.  20. 50. 100. 200. 500.];
NL_lightShip = [64.  67. 66. 58.  46.  30. ];
NL_moderate = [72.5 76. 75. 69.  58.  42.];
NL_heavy = [81.5 85. 85. 79.  69.  53.5];

plot(frqW, SeaSt0_Wenz, '--', 'LineWidth', 2);
plot(frqW, SeaSt1_Wenz, '--', 'LineWidth', 2);
plot(frqW, SeaSt2_Wenz, '--', 'LineWidth', 2);
%plot(frqW, SeaSt6_Wenz, '--', 'LineWidth', 2);
plot(frqShip, NL_lightShip, '--', 'LineWidth', 2);
plot(frqShip, NL_moderate, '--', 'LineWidth', 2);
plot(frqShip, NL_heavy, '--', 'LineWidth', 2);

text(1000,43,'SS 0','FontSize',14);
text(1000,53,'SS 1','FontSize',14);
text(1000,64,'SS 3','FontSize',14);
%text(1000,70,'SS 6','FontSize',14);
text(20,68,'Light','FontSize',14);
text(20,77,'Moderate','FontSize',14);
text(20,86, 'Heavy','FontSize',14);


set(gca,'XScale','log');
%axis([10 fs/2 10 90]);
strn = sprintf('Noise spectral density');
ylabel('Spectral Level in dB re 1\muPa^2/Hz');
xlabel('Frequency [Hz]');
title(strn);
grid on;
hold off;



go = input('quit: ');
if(go == 1) break; end;
    
end

