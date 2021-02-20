%DISPLAYAEROEQUATIONS Display aerodynamic factors equations in LaTeX format
%   Written: 2021/01/17, J.X.J. Bannwarth

clearvars;
project = simulinkproject; projectRoot = project.RootFolder;

%% Load data
load( fullfile( projectRoot, 'data_misc', 'AeroBodyOrientedAIAAv3.mat' ) )

factors = fields( Aero );
factors = factors( contains( factors, 'C' ) );

disp( '\begin{align}' )
for ii = 1:length( factors )
    factor = factors{ii};
    factor = sprintf('%s_{%s,%s}', factor(1), factor(2), factor(3) );
    equation = Aero.(factors{ii}).equation;
    coefs = Aero.(factors{ii}).coefs;
    for jj = 1:length( coefs )
        equation = replace( equation, sprintf('p%d', jj-1), ...
            num2str( round(Aero.(factors{ii}).coefs(jj), 4, 'significant') ) );
    end
    if Aero.(factors{ii}).x2 == '-'
        equation = replace( equation, 'x', Aero.(factors{ii}).x1 ) ;
    else
        equation = replace( equation, 'x1', Aero.(factors{ii}).x1 ) ;
        equation = replace( equation, 'x2', Aero.(factors{ii}).x2 ) ;
    end
    equation = replace( equation, 'y', [ factor ' &'] );
    equation = replace( equation, 'sin', '\sin' );
    equation = replace( equation, 'cos', '\cos' );
    equation = replace( equation, '*', ' ' );
    equation = replace( equation, 'lambda', '\lambda' );
    equation = replace( equation, 'alpha', '\alpha' );
    equation = replace( equation, '+ -', '- ' );
    if contains ( equation, 'exp' )
        idxStart = strfind( equation, 'exp' );
        idxEnd = idxStart;
        for jj = 1:length( idxStart )
            idxEnd(jj) = find( equation(idxStart(jj):end) == ')', 1 );
            equation(idxEnd(jj)+idxStart(jj)-1) = '}';
        end
        equation = replace( equation, 'exp(', 'e^{' );
    end
    equation = sprintf( '    %s', equation );
    if ii < length(factors)
        equation = [ equation ',\\' ];
    else
        equation = [ equation '.' ];
    end
    disp( equation )
end
disp( '\end{align}' )