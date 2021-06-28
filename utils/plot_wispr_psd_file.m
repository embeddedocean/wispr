%
% matlab script to plot wispr spectrum data
%
%

clear all;

[file, dpath, filterindex] = uigetfile('./*.psd', 'Pick a spectrum file');
name = fullfile(dpath,file);

% read file
fp = fopen( name, 'r', 'ieee-le' );

N = 1; % number of buffer to concatenate

count = 0;
go = 1;
while( go )

    % read data block
    [hdr, psd] = wispr_read(fp);
    
    if(isempty(psd)) 
       go = 0;
       break; 
    end
        
    hdr.adc_bps = hdr.settings(1);
    hdr.wintype = hdr.settings(2);
    hdr.nfft = bitshift(hdr.settings(3),4);
    hdr.overlap = bitshift(hdr.settings(4),4);
     
    nfft = hdr.nfft;
    fs = hdr.sampling_rate;
    df = fs / hdr.nfft;
    freq = (0:(length(psd)-1)) * df;
    nbps = hdr.adc_bps;

    adc_vref = 5.0/2;
    max_adc_value = 2^(nbps*8-1)-1;
    adc_scaling = adc_vref / max_adc_value;
    scale = (adc_scaling / nfft)^2;
    
    if(hdr.wintype == 1) 
        scale = scale / 4.0;
    end
    
    fprintf('time = %d\n', hdr.sec);
    
    % normalization
    psd = psd * scale;

    % plot buffers to make sure data is not lost between buffers
    figure(3); clf;
    plot(freq/1000, 10*log10(double(psd)), '.-');
    grid on;
    ylabel('dB');
    xlabel('kHz');

    total_energy = sum(psd);
    title(['WISPR spectrum, Total Energy ' num2str(total_energy)]);
    
    if(input('quit: ')) 
        go = 0;
        break; 
    end;
    
end

fclose(fp);

return;

