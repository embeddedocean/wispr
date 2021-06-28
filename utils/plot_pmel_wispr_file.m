%
% matlab script to plot wispr data
%
% A data file consists of an ascii file header followed by binary data
% buffers. The ascii header is formatted as matlab expressions. 
% The binary data words are formatted as signed 16 or 24 bit integers.
%
% The data file format is:
% - 512 byte ascii header.
% - adc buffer 1
% - adc buffer 2
% ...
% - adc buffer N
% where N is the number of adc buffers per file
%
% The number of adc buffers is defined as 
% number_buffers = file_size*512 / buffer_size;
%
% The total data file size is always a multiple of 512 bytes blocks. 
% The variable 'file_size' is the number of 512 blocks in the file.
%
% Each adc buffer is of length 'buffer_size' bytes.
% The adc buffer is always a multiple of 512 bytes blocks (32 blocks in most cases). 
% Each adc buffer contains a fixed number of sample (samples_per_buffer).
% Each sample is of fixed size in bytes (sample_size).
% The sample size can be 2 or 3.
% If 3 byte samples are used, there will be extra bytes of padded at the end of each adc buffer.
% The number of bytes of padding is defined as:
% padding_per_buffer = buffer_size - (samples_per_buffer * sample_size);
%
%Spectral energy is corrected for the type of window function used for time
%series. H.M. 6/21/2021
%

clear all;

[file, dpath, filterindex] = uigetfile('G:/*.dat', 'Pick a data file');
name = fullfile(dpath,file);

fp = fopen( name, 'r', 'ieee-le' );

% read and eval the ascii header lines
for n = 1:14
    str = fgets(fp, 64); % read 64 chars max in each line
    % read ascii lines until a null is found, so header buffer must be null terminated
    if( str(1) == 0 )
        break;
    end
    eval(str);
    fprintf('%s', str);
end

% seek to the start of data
% header is always 512 bytes
fseek(fp, 512, -1);

if(sample_size == 2)
    q = adc_vref/32767.0;  % 16 bit scaling to volts
    fmt = 'int16';
elseif(sample_size == 3)
    q = adc_vref/8388608.0;  % 24 bit scaling to volts
    fmt = 'bit24';
elseif(sample_size == 4)
    q = 1.0;
    fmt = 'int32';
end

% The number of adc buffers in the file
number_buffers = file_size*512 / buffer_size;

% The number of bytes of padding after each adc buffer, if any
padding_per_buffer = buffer_size - (samples_per_buffer * sample_size);

% number of buffer to concatenate and plot
num_bufs_to_display = 8; 

count = 0;
go = 1;
t0 = 0;
prev_secs = 0;
hack = 1;

fft_size = 1024;
nbins = fft_size / 2 + 1;

while( go )

    data = [];
    psd = [];
    time = [];
    freq = [];

    for n = 1:num_bufs_to_display

        % read a data buffer
        raw = fread(fp, samples_per_buffer, fmt ); % data block
        if( length(raw) ~= samples_per_buffer )
            break;
        end
        
        % read padding, if any
        junk = fread(fp, padding_per_buffer, 'char');

        % add raw data buffer as a column
        data(:,n) = double(raw)*q;
        dt = 1.0 / sampling_rate;
        time(:,n) = t0 + (1:length(raw)) * dt;
        t0 = time(end,n);

        duration = samples_per_buffer * dt;

        count = count + 1;

    end

    % plot data buffers 
    figure(1); %clf;
    plot(time, data,'.-');
    ylabel('Volts');
    xlabel('Seconds');
    grid on;
    axis([min(min(time)) max(max(time)) -5.1 5.1]);

    % Calc spectrum of data
    %window = rectwin(fft_size);
    window = hamming(fft_size)*1.59; %multiply energy correction
    %window = hann(fft_size)*1.63;
    overlap = 0;
    fs = sampling_rate;
    [Spec, f] = my_psd(data(:),fs,window,overlap);
       
    % plot spectrum
    figure(2); clf;  
    hold on;
    plot(f, 10*log10(Spec),'.-'); %normalize the power spec    
    grid(gca,'minor');
    grid on;
    xlabel('Frequency [kHz]'),
    ylabel('Power Spectrum Magnitude (dB)');
    %axis([0 freq(end) -130 0]);

    total_energy = sum(Spec);
    sig_var = var(data(:));
    title(['Matlab spectrum, Total Energy ' num2str(total_energy) ', Variance ' num2str(sig_var)]);

    if(count >= number_buffers) 
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

