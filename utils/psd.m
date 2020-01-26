function [Spec, freq] = psd(x,nfft,fs,window,noverlap)
% PSD Power Spectral Density estimate.
% based on original matlab psd function and this app note:
%    http://www.mathworks.com/help/signal/ug/psd-estimate-using-fft.html
%
% cjones

% compute PSD
window = window(:);
n = length(x);		    % Number of data points
nwind = length(window); % length of window
if n < nwind            % zero-pad x if it has length less than the window length
    x(nwind)=0;  n=nwind;
end

% Make sure x is a column vector; do this AFTER the zero-padding
% in case x is a scalar.
x = x(:);		

% Number of windows
k = fix((n-noverlap)/(nwind-noverlap));

% Obtain the averaged periodogram using fft. 
% The signal is real-valued and has even length. 
Spec = zeros(nfft,1); 
index = 1:nwind;
for i=1:k
    xw = window.*(x(index));
    index = index + (nwind - noverlap);
    Xx = abs(fft(xw,nfft)).^2;
    Spec = Spec + Xx;
end

KMU = k*norm(window)^2;	% Normalizing scale factor ==> asymptotically unbiased
Spec = Spec/KMU;

% Because the signal is real-valued, you only need power estimates for the positive or negative frequencies. 
% Select first half
select = (1:nfft/2+1)';
freq = (select - 1)*fs/nfft;

% In order to conserve the total power, multiply all frequencies
% by a factor of 2. Zero frequency (DC) and the Nyquist frequency do not occur twice.
Spec = 2*Spec(select);

% normalization
Spec = Spec / (nfft * fs );   

%plot(freq_vector,10*log10(abs(P))), grid on
%xlabel('Frequency'), 
%ylabel('Power Spectrum Magnitude (dB)');
