%
% matlab script to plot wispr data
%
%

clear all;

[file, dpath, filterindex] = uigetfile('./*.dat', 'Pick a waveform file');
name = fullfile(dpath,file);

% read file
format = 'ieee-le';
fp = fopen( name, 'r', format );

N = 1; % number of buffer to concatenate

count = 0;
go = 1;
t0 = 0;
while( go )

    data = [];
    
    for n = 1:N
        
        % read block header
        [hdr, raw] = wispr_read(fp);
     
        if(isempty(raw)) 
            go = 0;
            break; 
        end
        
        if(hdr.sample_size == 2) 
            q = 5.0/32767.0;  % 16 bit scaling to volts
        elseif(hdr.sample_size == 3)
            q = 5.0/8388608.0;  % l24 bit scaling to volts
        elseif(hdr.sample_size == 4)
            q = 1.0;
        end

        % concatenate raw data buffer into one dat vector
        %data = [data; double(raw)*q]; 
        data(:,n) = double(raw)*q;
        dt = 1.0 / hdr.sampling_rate;
        t(:,n) = t0 + (1:length(raw)) * dt;
        t0 = t(end,n);
    
    end
    
    if(go == 0) 
        break; 
    end;
    
    %t = (1:length(data)) / hdr.sampling_rate;

    fprintf('time = %d\n', hdr.sec);
    
    % plot buffers to make sure data is not lost between buffers
    figure(1); clf;
    plot(t, data,'.-');
    ylabel('Volts');
    xlabel('Seconds');
    
    nfft = 1024;
    window = rectwin(nfft);
    %window = hamming(nfft);
    overlap = 256;
    fs = hdr.sampling_rate;
    [Spec, freq] = my_psd(data(:),fs,window,overlap);
    figure(2); clf;
    plot(1000*freq, 10*log10(Spec),'.-');
    grid on;
    xlabel('Frequency [kHz]'), 
    ylabel('Power Spectrum Magnitude (dB)');
    %axis([0 freq(end) -130 0]);

    total_energy = sum(Spec);
    sig_var = var(data);
    title(['WISPR spectrum, Total Energy ' num2str(total_energy) ', Variance ' num2str(sig_var)]);
        
    if(input('quit: ') == 1) 
        go = 0;
        break; 
    end;
    
end

fclose(fp);

return;

