function latestFilename = GetLatestFile( dirIn, restrictStr, excludeStr )
%GETLATESTFILE Return the name of the most recently modified file in folder
%   LATESTFILENAME = GETLATESTFILE( DIRIN ) finds most recent file in DIRIN
%   LATESTFILENAME = GETLATESTFILE( DIRIN, RESTRICTSTR ) only includes files that match
%   LATESTFILENAME = GETLATESTFILE( DIRIN, RESTRICTSTR, EXCLUDESTR ) excludes files
%
%   See also DIR.
%
%   Written: 2021/02/17, J.X.J. Bannwarth
    arguments
        dirIn       (1,:) char
        restrictStr (1,:) char = ''
        excludeStr  (1,:) char = ''
    end

    % Error
    if ~isfolder( dirIn )
        error( 'Target directory %s does not exist', dirIn )
    end

    % Restrict to files matching desired pattern
    if contains( restrictStr, '*' )
        % Already contains wildcard, keep as is
        files = dir( fullfile( dirIn, restrictStr ) );
    else
        % Add wildcards - if restrictStr is empty, '**' returns all files
        % in dir
        files = dir( fullfile( dirIn, [ '*' restrictStr '*' ] ) );
    end

    % Exclude files matching exclude pattern
    if ~isempty( excludeStr )
        files = files( ~contains( {files.name}, excludeStr ));
    end

    % Get rid of any remaining folders
    files = files( ~[files.isdir] );
    
    % Error
    if isempty( files )
        error( 'No files in target directory %s', dirIn )
    end
    
    % Get latest file
    dates = datetime( {files.date}, 'Format', 'dd-MMM-yyyy HH:mm:ss' );
    [~, idxMostRecent] = max( dates );
    
    latestFilename = files(idxMostRecent).name;
end