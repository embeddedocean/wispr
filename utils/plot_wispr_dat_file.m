%
% matlab script to plot wispr data
%
%

clear all;

[file, dpath, filterindex] = uigetfile('./*.dat', 'Pick a waveform file');
name = fullfile(dpath,file);

vref = 5.0;

format long;

% read file
fp = fopen( name, 'r', 'ieee-le' );

N = 20; % number of buffer to concatenate

count = 0;
go = 1;
t0 = 0;
prev_secs = 0;
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
            q = vref/32767.0;  % 16 bit scaling to volts
        elseif(hdr.sample_size == 3)
            q = vref/8388608.0;  % l24 bit scaling to volts
        elseif(hdr.sample_size == 4)
            q = 1.0;
        end

        % concatenate raw data buffer into one dat vector
        %data = [data; double(raw)*q]; 
        data(:,n) = double(raw)*q;
        dt = 1.0 / hdr.sampling_rate;
        t(:,n) = t0 + (1:length(raw)) * dt;
        t0 = t(end,n);

        if(hdr.usec > 1000000) 
           fprintf('invalid usecs = %f\n', hdr.usec / 1000000 );
        end
        
        secs = hdr.sec + hdr.usec * 0.000001;     
        duration = hdr.samples_per_block * dt; 
        delta = (secs - prev_secs);
        if( delta ~= duration ) 
        end
        prev_secs = secs;

        % print time of first record
        fprintf(' - second = %f\n', secs );
        fprintf(' - duration  %f sec\n', duration);
        fprintf(' - timestamp delta %f sec\n', delta);
        fprintf(' - difference %f sec (%d samps)\n\n', (duration-delta), round((duration-delta)/dt));
        
    end
    
    if(go == 0) 
        break; 
    end;
    
    %t = (1:length(data)) / hdr.sampling_rate;

    % plot buffers to make sure data is not lost between buffers
    figure(2); clf;
    plot(t, data,'.-');
    ylabel('Volts');
    xlabel('Seconds');
    grid on;
    
    nfft = 1024;
    %window = rectwin(nfft);
    window = hamming(nfft);
    overlap = 256;
    fs = hdr.sampling_rate;
    [Spec, freq] = my_psd(data(:),fs,window,overlap);
    figure(3); clf;
    plot(freq/1000, 10*log10(Spec),'.-');
    grid on;
    xlabel('Frequency [kHz]'), 
    ylabel('Power Spectrum Magnitude (dB)');
    %axis([0 freq(end) -130 0]);

    total_energy = sum(Spec);
    sig_var = var(data(:));
    title(['WISPR spectrum, Total Energy ' num2str(total_energy) ', Variance ' num2str(sig_var)]);
        
    if(input('quit: ') == 1) 
        go = 0;
        break; 
    end;
    
end

fclose(fp);

return;

