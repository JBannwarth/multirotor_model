% Get mask
maskObj = Simulink.Mask.get('Px4Library/mc_att_control');

limits = { ...
'MC_ROLL_TC',         '0.2',   0.15,  0.25;
'MC_PITCH_TC',        '0.2',   0.15,  0.25;
'MC_ROLL_P',          '6.5',   0.0,    8.0;
'MC_ROLLRATE_P',      '0.15',  0.0,    0.5;
'MC_ROLLRATE_I',      '0.05',  0.0,   10.0;
'MC_RR_INT_LIM',      '0.30',  0.0,   10.0;
'MC_ROLLRATE_D',      '0.003', 0.0,   0.01;
'MC_ROLLRATE_FF',     '0.0',   0.0,   10.0;
'MC_PITCH_P',         '6.5',   0.0,   10.0;
'MC_PITCHRATE_P',     '0.15',  0.0,    0.6;
'MC_PITCHRATE_I',     '0.05',  0.0,   10.0;
'MC_PR_INT_LIM',      '0.30',  0.0,   10.0;
'MC_PITCHRATE_D',     '0.003', 0.0,   10.0;
'MC_PITCHRATE_FF',    '0.0',   0.0,   10.0;
'MC_YAW_P',           '2.8',   0.0,    5.0;
'MC_YAWRATE_P',       '0.2',   0.0,    0.6;
'MC_YAWRATE_I',       '0.1',   0.0,   10.0;
'MC_YR_INT_LIM',      '0.30',  0.0,   10.0;
'MC_YAWRATE_D',       '0.0',   0.0,   10.0;
'MC_YAWRATE_FF',      '0.0',   0.0,   10.0;
'MC_YAW_FF',          '0.5',   0.0,    1.0;
'MC_ROLLRATE_MAX',  '220.0',   0.0,  360.0;
'MC_PITCHRATE_MAX', '220.0',   0.0,  360.0;
'MC_YAWRATE_MAX',   '200.0',   0.0,  360.0;
'MC_YAWRAUTO_MAX',   '45.0',   0.0,  120.0;
'MC_ACRO_R_MAX',    '360.0',   0.0, 1000.0;
'MC_ACRO_P_MAX',    '360.0',   0.0, 1000.0;
'MC_ACRO_Y_MAX',    '360.0',   0.0, 1000.0;
'MC_RATT_TH',         '0.8',   0.0,    1.0;
'MC_BAT_SCALE_EN',    '0.0',   0.0,    1.0;
'MC_TPA_BREAK_P',     '1.0',   0.0,    1.0;
'MC_TPA_BREAK_I',     '1.0',   0.0,    1.0;
'MC_TPA_BREAK_D',     '1.0',   0.0,    1.0;
'MC_TPA_RATE_P',      '0.0',   0.0,    1.0;
'MC_TPA_RATE_I',      '0.0',   0.0,    1.0;
'MC_TPA_RATE_D',      '0.0',   0.0,    1.0;
'VT_TYPE',            '2.0',   0.0,    2.0;
'MC_TPA_BREAK',       '1.0',   0.0,    1.0;
'MC_TPA_SLOPE',       '1.0',   0.0,    1.0;
'VT_OPT_RECOV_EN',    '0.0',   0.0,    1.0;
'VT_WV_YAWR_SCL',     '0.15',  0.0,    1.0 ...
};

for i = 1:length(maskObj.Parameters)
    val = maskObj.Parameters(i).Value;
    name = maskObj.Parameters(i).Name;
    
    for j = 1:length(limits)
        if strcmp(name, limits{j, 1})
            maskObj.Parameters(i).Type  = 'slider';
            maskObj.Parameters(i).Value = val;
            maskObj.Parameters(i).Range = [ limits{j, 3}, limits{j, 4} ];
            maskObj.Parameters(i).Value = limits{j,2};
        end
        
    end
end