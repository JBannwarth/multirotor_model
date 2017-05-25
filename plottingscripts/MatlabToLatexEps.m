function MatlabToLatexEps(filename, res, printEps)
%MATLABTOLATEXEPS Print figure to PDF, EPS (optional), and PNG (optional)
%   Written by: Z.J. Chen
%   Last modified by: J.X.J. Bannwarth, 10/05/2017
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