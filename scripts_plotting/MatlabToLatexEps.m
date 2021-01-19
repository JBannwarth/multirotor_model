function MatlabToLatexEps( filename, res, printEps )
%MATLABTOLATEXEPS Print figure to PDF, EPS (optional), and PNG (optional)
%   MatlabToLatexEps(filename)
%   MatlabToLatexEps(filename, res)
%   MatlabToLatexEps(filename, res, printEps)
%   Written:                   Z.J. Chen
%   Last modified: 2021/01/19, J.X.J. Bannwarth
    arguments
        filename (1,:) char
        res      (1,1) double = -1
        printEps (1,1) double = false
    end

    if res > 0
        % Set quality
        resString = sprintf( '-r%d', res );
        print( gcf, filename, '-dpng', resString )
    end
    
    print( gcf, filename, '-dpdf', '-loose' )
    
    if ( printEps )
        % Export initial eps file
        print( gcf, filename, '-depsc', '-loose' )
        % Replace \n by \r\n in eps file
        eps = fileread( [filename, '.eps'] );
        fid = fopen( [filename, '.eps'], 'wt' );
        fwrite( fid, eps );
        fclose( fid );
    end
end