function UseWindProfile( modelName, toUse )
%USEWINDPROFILE Switches the varying wind input on/off
%   Written: 21/08/2017, Jeremie Bannwarth
    
    if toUse
        set_param( [modelName '/Input choice'], 'Value', '1' );
        set_param( [modelName '/Varying wind input'], 'commented', 'off' );
    else
        set_param( [modelName '/Input choice'], 'Value', '2' );
        set_param( [modelName '/Varying wind input'], 'commented', 'on' );
    end
end