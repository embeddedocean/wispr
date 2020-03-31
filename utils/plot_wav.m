%
% matlab script to plot wispr data
%
%

clear all;

[file, dpath, filterindex] = uigetfile('./*.wav', 'Pick a wav file');
filename = fullfile(dpath,file);

% read file info
[m d] = wavfinfo(filename);
fprintf('%s\n', d);

[data, fs, nbits] = wavread(filename);
    
t = (1:length(data)) / fs;
    
% plot buffers to make sure data is not lost between buffers
figure(1); clf;
plot(t, data,'.-');
    
return;

