%
% matlab script to plot rockhopper data
%
%

clear all;

[file, dpath, filterindex] = uigetfile('./*', 'Pick a waveform file');
name = fullfile(dpath,file);

% read file
format = 'ieee-le';
fp = fopen( name, 'r', format );

q = 1.0; %5.0/8388608.0;  % ltc2512 scaling to volts

N = 1; % number of buffer to concatenate

count = 0;
go = 1;
while( go )

    data = [];
    
    for n = 1:N
        
        % read block header
        [hdr, raw] = wispr_read(fp);
        if(isempty(raw)) 
            go = 0;
            break; 
        end
        data = [data; double(raw)*q]; % concatenate raw data buffer into one dat vector
    
    end
    
    if(go == 0) 
        break; 
    end;
    
    t = (1:length(data)) / hdr.sampling_rate;

    fprintf('time = %d\n', hdr.sec);
    
    % plot buffers to make sure data is not lost between buffers
    figure(1); clf;
    plot(t, data,'.-');
    
    if(input('quit: ') == 1) 
        go = 0;
        break; 
    end;
    
end

fclose(fp);

return;

