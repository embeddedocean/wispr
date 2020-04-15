%
% matlab script to plot wispr spectrum data
%
%

clear all;

[file, dpath, filterindex] = uigetfile('./*.psd', 'Pick a spectrum file');
name = fullfile(dpath,file);

% read file
format = 'ieee-le';
fp = fopen( name, 'r', format );

N = 1; % number of buffer to concatenate

count = 0;
go = 1;
while( go )

    % read data block
    [hdr, psd] = wispr_read(fp);
    
    hdr.gain = hdr.settings(1);
    hdr.df = hdr.settings(2);
    hdr.nfft = bitshift(hdr.settings(3),4);
    hdr.overlap = bitshift(hdr.settings(4),4);
     
    if(isempty(psd)) 
       go = 0;
       break; 
    end
    
    fs = hdr.sampling_rate;
    df = fs / hdr.nfft;
    freq = (0:(length(psd)-1)) * df;

    fprintf('time = %d\n', hdr.sec);
    
    % normalization
    psd = psd / (hdr.nfft * hdr.sampling_rate);

    % plot buffers to make sure data is not lost between buffers
    figure(3); clf;
    plot(freq/1000, 10*log10(double(psd)), '.-');
    ylabel('dB');
    xlabel('kHz');
    
    if(input('quit: ')) 
        go = 0;
        break; 
    end;
    
end

fclose(fp);

return;

