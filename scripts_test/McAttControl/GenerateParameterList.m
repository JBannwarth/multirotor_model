fid = fopen( 'AttCtrlVariables.txt', 'r' );
variables = textscan( fid , '%s %s %s', 'Delimiter', ',' );
fclose( fid );
fid = fopen( 'Params.txt', 'r' );
values = textscan( fid , '%s %f %f %f %s %s', 'Delimiter', ';' );
fclose( fid );

found =  false( length(variables{1}), 1 );

maxVarLength = 1;
for i = 1:length(values{1})
    for j = 1:length(variables{1})
        if strcmp( values{1}{i}, variables{3}{j} )
            found(j) = true;
            if length( variables{2}{j} ) > maxVarLength
                maxVarLength = length( variables{2}{j} );
            end
        end
    end
end
mvl = num2str( maxVarLength );

for i = 1:length(values{1})
    for j = 1:length(variables{1})
        if strcmp( values{1}{i}, variables{3}{j} )
            checks = {};
            col = ' ';
            lims = '';
            if ( values{3}(i) ~= 9999 )
                checks{end+1} = sprintf( ['mustBeGreaterThanOrEqual(%-' mvl 's,% 5g)'], variables{2}{j}, values{3}(i) );
                col = ',';
                lims = [ num2str(values{3}(i))];
            else
                checks{end+1} = repmat(' ', 1, length( 'mustBeGreaterThanOrEqual(,)' ) + maxVarLength + 5);
            end
            if ( values{4}(i) ~= 9999 )
                checks{end+1} = sprintf( ['%s mustBeLessThanOrEqual(%-' mvl 's,% 5g)'], col, variables{2}{j}, values{4}(i) );
                col = ',';
                if isempty(lims)
                    lims = [ '[<' num2str(values{4}(i)) ']' ];
                else
                    lims = [ '[' lims '-' num2str(values{4}(i)) ']' ];
                end
            else
                checks{end+1} = repmat(' ', 1, 2 + length( 'mustBeLessThanOrEqual(,)' ) + maxVarLength + 5);
                col = ' ';
                if ~isempty(lims)
                    lims = [ '[>' lims ']' ];
                end
            end
            switch variables{1}{j}
                case 'BOOL'
                    checks{end+1} = sprintf( ['%s mustBeInteger(%-' mvl 's)'], col, variables{2}{j});
                case 'INT'
                    checks{end+1} = sprintf( ['%s mustBeInteger(%-' mvl 's)'], col, variables{2}{j});
                otherwise
                    % Nothing
                    checks{end+1} = repmat(' ', 1, 2 + length( 'mustBeInteger()' ) + maxVarLength);
            end
            if ~isempty(checks)
                checksStr = [ '{' char(join( checks, '' )) '} ' ];
            else
                checksStr = '';
            end
            initLine{i,1} = sprintf( ['%-' mvl 's(1,1) % 100s= %5g; %% %s %s'], variables{2}{j}, checksStr, values{2}(i), variables{3}{j}, lims );
            break
        end
    end
end

for i=1:length(initLine)
    disp(initLine{i})
end
% for j = 1:length(variables{1})
%     if ~found(j)
%         disp( variables{3}{j} )
%     end
% end