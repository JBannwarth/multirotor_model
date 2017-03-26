function matlabToLatexEps(filename,res)
% MATLABTOLATEXEPS Save current figure to pdf, eps, and png
% Written by: Z. Jeremy Chen

    if ~isempty(res)
        % Set quality
        resString = sprintf('-r%d',res);
        print(gcf,filename,'-dpng',resString)
    end
    % Export initial eps file
    print(gcf,filename,'-depsc','-loose')
    print(gcf,filename,'-dpdf','-loose')
    % Replace \n by \r\n in eps file
    eps = fileread([filename,'.eps']);
    fid = fopen([filename,'.eps'],'wt');
    fwrite(fid,eps);
    fclose(fid);
    
end