function flogOut = CropFLog( flogIn, tStart, tEnd, adjust )
%CROPFLOG Crop flight log
%   Written by:    J.X.J. Bannwarth, 2017/10/31
%   Last modified: J.X.J. Bannwarth, 2017/10/31
    fields = fieldnames(flogIn);
    for i=1:length(fields)
        if strcmp( fields{i}, 'params' )
            flogOut.params = flogIn.(fields{i});
        else
            tmp = flogIn.(fields{i});
            toKeep = ( tmp.time > tStart ) & ( tmp.time <= tEnd );
            tmp(~toKeep,:) = [];
            if adjust
                tmp.time = tmp.time - tStart;
            end
            flogOut.(fields{i}) = tmp;
        end
    end
end
