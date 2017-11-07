function MatlabToLatexEps(filename, varargin )
%MATLABTOLATEXEPS Print figure to PDF, EPS (optional), and PNG (optional)
%   MatlabToLatexEps(filename)
%   MatlabToLatexEps(filename, res)
%   MatlabToLatexEps(filename, res, printEps)
%   Written by: Z.J. Chen
%   Last modified by: J.X.J. Bannwarth, 11/10/2017
    nArg = length(varargin);
    res = [];
    printEps = false;
    
    if ( nArg == 1 )
        res = varargin{1};
    elseif ( nArg == 2 )
        res = varargin{1};
        printEps = varargin{2};
    elseif ( nArg > 2 )
        error('Too many input');
    end

    if ~isempty(res)
        % Set quality
        resString = sprintf('-r%d', res);
        print(gcf, filename, '-dpng', resString)
    end
    
    print(gcf,filename,'-dpdf','-loose')
    
    if ( printEps )
        % Export initial eps file
        print(gcf, filename, '-depsc', '-loose')
        % Replace \n by \r\n in eps file
        eps = fileread( [filename, '.eps'] );
        fid = fopen( [filename, '.eps'], 'wt' );
        fwrite(fid, eps);
        fclose(fid);
    end
end