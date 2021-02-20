function latestFilename = GetLatestFile( dirIn, searchPattern )
%GETLATESTFILE Return the name of the most recently modified file in folder
%   LATESTFILENAME = GETLATESTFILE( DIRIN ) finds most recent file in DIRIN
%   LATESTFILENAME = GETLATESTFILE( DIRIN, SEARCHPATTERN ) specifies a search pattern
%
%   See also DIR.
%
%   Written: 2021/02/17, J.X.J. Bannwarth
    arguments
        dirIn (1,:) char
        searchPattern (1,:) char = ''
    end
    
    % Error
    if ~isfolder( dirIn )
        error( 'Target directory %s does not exist', dirIn )
    end
    
    if ~isempty( searchPattern )
        files = dir( [ dirIn '/' searchPattern ] );
    else
        files = dir( dirIn );
    end
    
    % Get rid of folders
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