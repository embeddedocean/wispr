function [hdr, data] = wispr_read(fp)
%
% Read an openned wispr data file
%

WISPR_DATA_HEADER_SIZE = 32;

hdr = [];
data = [];

name = fread(fp, 6, 'uint8=>char' );
if( ~strcmp(name', 'WISPR2') ) 
   fprintf('Unknown data record name %s\n', name);
   return;
end

hdr.version(1) = fread(fp, 1, 'uint8' );
hdr.version(2) = fread(fp, 1, 'uint8' );

hdr.type = fread(fp, 1, 'uint8' );

hdr.sec = fread(fp, 1, 'uint32' );
hdr.usec = fread(fp, 1, 'uint32' );

hdr.settings(1) = fread(fp, 1, 'uint8' );
hdr.settings(2) = fread(fp, 1, 'uint8' );
hdr.settings(3) = fread(fp, 1, 'uint8' );
hdr.settings(4) = fread(fp, 1, 'uint8' );

hdr.block_size = fread(fp, 1, 'uint16' );

hdr.sample_size = fread(fp, 1, 'uint8'  );
hdr.samples_per_block = fread(fp, 1, 'uint16' );
hdr.sampling_rate = fread(fp, 1, 'uint32' );

hdr.checksum1 = fread(fp, 1, 'uint8' );
hdr.checksum2 = fread(fp, 1, 'uint8' );

if(hdr.sample_size == 2)
    raw = fread(fp, hdr.samples_per_block, 'int16' ); % data block
elseif (hdr.sample_size == 3)
    raw = fread(fp, hdr.samples_per_block, 'bit24' ); % data block
elseif (hdr.sample_size == 4)
    if(hdr.type == 2) 
        %raw = fread(fp, hdr.samples_per_block, 'real*4=>double' ); % data block
        raw = fread(fp, hdr.samples_per_block, 'real*4' ); % data block
    else
        raw = fread(fp, hdr.samples_per_block, 'int32' ); % data block
    end
else
    fprintf('Unknown sample size %d\n', hdr.sample_size);
end

% sometimes there is padding at the end of the data buffer
npad = hdr.block_size - (WISPR_DATA_HEADER_SIZE + hdr.samples_per_block * hdr.sample_size);
if( npad > 0) 
    junk = fread(fp, npad, 'int8' );
end

data = double(raw);

return;
