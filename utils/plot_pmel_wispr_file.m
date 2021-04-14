%
% matlab script to plot wispr data
%
%

clear all;

[file, dpath, filterindex] = uigetfile('./*.dat', 'Pick a data file');
name = fullfile(dpath,file);

fp = fopen( name, 'r', 'ieee-le' );

% read the ascii header lines
str = fgets(fp);
for n = 1:17
    str = fgets(fp);
    if(str < 0)
        break;
    end;
    eval(str);
    fprintf('%s', str);
end;

% seek to the start of data
% header is always 512 bytes
fseek(fp, 512, -1);

nbins = fft_size / 2;

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

% read file

N = 8; % number of buffer to concatenate

count = 0;
go = 1;
t0 = 0;
prev_secs = 0;
hack = 1;

%samples_per_buffer = 5461;

while( go )

    data = [];
    psd = [];
    time = [];
    freq = [];

    for n = 1:N

        % read a data buffer

        raw = fread(fp, samples_per_buffer, fmt ); % data block
        if( isempty(raw) )
            break;
        end

        % add raw data buffer as a column
        data(:,n) = double(raw)*q;
        dt = 1.0 / sampling_rate;
        time(:,n) = t0 + (1:length(raw)) * dt;
        t0 = time(end,n);


        duration = samples_per_buffer * dt;

        count = count + 1;

    end

    %t = (1:length(data)) / hdr.sampling_rate;

    % plot buffers to make sure data is not lost between buffers

    figure(1); clf;
    plot(time, data,'.-');
    ylabel('Volts');
    xlabel('Seconds');
    grid on;
    axis([min(min(time)) max(max(time)) -5.1 5.1]);

    window = rectwin(fft_size);
    %window = hamming(nfft);
    overlap = 256;
    fs = sampling_rate;
    [Spec, f] = my_psd(data(:),fs,window,overlap);

    figure(2); clf;
    plot(f/1000, 10*log10(Spec),'.-');
    grid on;
    xlabel('Frequency [kHz]'),
    ylabel('Power Spectrum Magnitude (dB)');
    %axis([0 freq(end) -130 0]);

    total_energy = sum(Spec);
    sig_var = var(data(:));
    title(['Matlab spectrum, Total Energy ' num2str(total_energy) ', Variance ' num2str(sig_var)]);

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

