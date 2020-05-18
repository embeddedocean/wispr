%
% matlab script to plot wispr data
%
%

clear all;

[file, dpath, filterindex] = uigetfile('./*.txt', 'Pick a header file');
name = fullfile(dpath,file);

fp = fopen(name, 'r');
str = fgets(fp);
while(1)
  str = fgets(fp);
  if(str < 0) 
      break; 
  end;
  eval(str);
end;
fclose(fp);

dat_name = [];
if( bitand(uint8(mode),1) )
   dat_name = [name(1:end-3) 'dat'];
end

psd_name = [];
if( bitand(uint8(mode),2) )
   psd_name = [name(1:end-3) 'psd'];
   nbins = fft_size / 2;
end

if(sample_size == 2) 
   q = adc_vref/32767.0;  % 16 bit scaling to volts
   fmt = 'int16';
elseif(sample_size == 3)
   q = adc_vref/8388608.0;  % l24 bit scaling to volts
   fmt = 'bit24';
elseif(sample_size == 4)
   q = 1.0;
   fmt = 'int32';
end

% read file
if(dat_name) 
    dat_fp = fopen( dat_name, 'r', 'ieee-le' );
end

if(psd_name)
    psd_fp = fopen( psd_name, 'r', 'ieee-le' );
end

N = 8; % number of buffer to concatenate

count = 0;
go = 1;
t0 = 0;
prev_secs = 0;
while( go )
    
    data = [];
    psd = [];
    time = [];
    freq = [];
    
    for n = 1:N
        
        % read a data buffer
        if(dat_name) 
            
            raw = fread(dat_fp, samples_per_buffer, fmt ); % data block
            if( isempty(raw) ) 
                break; 
            end

            % add raw data buffer as a column
            data(:,n) = double(raw)*q;
            dt = 1.0 / sampling_rate;
            time(:,n) = t0 + (1:length(raw)) * dt;
            t0 = time(end,n);
        
        end

        if(psd_name) 
        
            S = fread(psd_fp, nbins, 'real*4' );
            if( isempty(S) ) 
                break; 
            end
            max_adc_value = 2^(sample_size*8-1)-1;
            adc_scaling = adc_vref / max_adc_value;  
            scale = (adc_scaling / fft_size)^2;
            if(fft_window_type == 1) % hamming 
                scale = scale / 4.0;
            end
            psd(:,n) = scale * S;
            df = sampling_rate / fft_size;
            freq(:,n) = (0:(length(S)-1)) * df;

        end

        duration = samples_per_buffer * dt; 
       
        count = count + 1;
        
    end
        
    %t = (1:length(data)) / hdr.sampling_rate;

    % plot buffers to make sure data is not lost between buffers
    if(dat_name) 
        
        figure(1); clf;
        plot(time, data,'.-');
        ylabel('Volts');
        xlabel('Seconds');
        grid on;

        window = rectwin(fft_size);
        %window = hamming(nfft);
        overlap = 256;
        fs = sampling_rate;
        [Spec, freq] = my_psd(data(:),fs,window,overlap);
        
        figure(2); clf;
        plot(freq/1000, 10*log10(Spec),'.-');
        grid on;
        xlabel('Frequency [kHz]'), 
        ylabel('Power Spectrum Magnitude (dB)');
        %axis([0 freq(end) -130 0]);

        total_energy = sum(Spec);
        sig_var = var(data(:));
        title(['Matlab spectrum, Total Energy ' num2str(total_energy) ', Variance ' num2str(sig_var)]);
    
    end

    if(psd_name) 
        figure(3); clf;
        plot(10*log10(psd),'.-');
        grid on;
        xlabel('Frequency [kHz]'), 
        ylabel('Power Spectrum Magnitude (dB)');
        total_energy = sum(psd(:)/N);
        sig_var = var(data(:));
        title(['WISPR spectrum, Total Energy ' num2str(total_energy) ', Variance ' num2str(sig_var)]);
    end
    
        
    if( count >= number_buffers ) 
        go = 0;
        break; 
    end
        
    if(input('quit: ') == 1) 
        go = 0;
        break; 
    end;
    
end

fclose(fp);

return;

