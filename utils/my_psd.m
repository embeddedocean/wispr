function [Spec, freq] = my_psd(x,fs,win,noverlap)
% PSD Power Spectral Density estimate.
% based on original matlab psd function and this app note:
%    http://www.mathworks.com/help/signal/ug/psd-estimate-using-fft.html
%
% cjones

% Make sure inputs are column vectors
x = x(:);		
win = win(:);

n = length(x);		    % Number of data points
nfft = length(win); % length of window
if n < nfft            % zero-pad x if it has length less than the window length
    x((n+1):nfft)=0;  
    n=nfft;
end

% Number of windows
navg = fix((n-noverlap)/(nfft-noverlap));

% Obtain the averaged periodogram using fft. 
% The signal is real-valued and has even length. 
Spec = zeros(nfft,1); 
index = 1:nfft;
for i=1:navg
    xw = win.*(x(index));
    index = index + (nfft - noverlap);
    Xx = abs(fft(xw,nfft)).^2;
    Spec = Spec + Xx;
end


% Because the signal is real-valued, you only need power estimates for the positive or negative frequencies. 
% Select first half
select = (1:nfft/2+1)';
freq = (select - 1)*fs/nfft;

% In order to conserve the total power, multiply all frequencies
% by a factor of 2. Zero frequency (DC) and the Nyquist frequency do not occur twice.
Spec = 2*Spec(select);

% normalization
winpow = norm(win)^2;
%Spec = Spec / (navg * nfft *nfft * winpow);
Spec = Spec / (navg * nfft *nfft);

%plot(freq_vector,10*log10(abs(P))), grid on
%xlabel('Frequency'), 
%ylabel('Power Spectrum Magnitude (dB)');
