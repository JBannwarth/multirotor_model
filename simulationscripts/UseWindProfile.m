function UseWindProfile( modelName, toUse )
%USEWINDPROFILE Switches the varying wind input on/off
%   Written by: Jeremie Bannwarth, 21/08/2017
    
    if toUse
        set_param( [modelName '/Varying wind input'], 'commented', 'off' );
        set_param( [modelName '/Wind switch'], 'sw', '1' );
    else
        set_param( [modelName '/Varying wind input'], 'commented', 'on' );
        set_param( [modelName '/Wind switch'], 'sw', '0' );
    end
end