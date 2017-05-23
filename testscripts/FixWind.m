clear; close all;
windFiles = { 'p5_z150_tfwt15', 'p5_z150_tfwt20', 'p5_z150_tfwt25', ...
              'p5_z150_tfwt30', 'p5_z150_tfwt35', 'p5_z150_tfwt40' };
inFolder  = 'inputdata';


for i = 1:length( windFiles )
    load( [ inFolder '/' windFiles{i} '.mat' ] )
    data = windInput.Data;
    [ azi, ele, rad ] = cart2sph( data(:,1), data(:,2), data(:,3) );
    
    figure('name', 'spherical')
    subplot(3,1,1)
    plot( windInput.Time, azi)
    subplot(3,1,2)
    plot( windInput.Time, ele)
    subplot(3,1,3)
    plot( windInput.Time, rad)
    
    goodData = (rad ~= 0);
    aziGood = azi(goodData);
    eleGood = ele(goodData);
    radGood = rad(goodData);
    t   = windInput.Time(goodData);
    
    aziNew = interp1( t, aziGood, windInput.Time );
    eleNew = interp1( t, eleGood, windInput.Time );
    radNew = interp1( t, radGood, windInput.Time );
    
    figure('name', 'spherical resampled')
    subplot(3,1,1)
    plot( windInput.Time, azi, windInput.Time, aziNew)
    subplot(3,1,2)
    plot( windInput.Time, ele, windInput.Time, eleNew)
    subplot(3,1,3)
    plot( windInput.Time, rad, windInput.Time, radNew)
    
    [U,V,W] = sph2cart(aziNew,eleNew,radNew);
    
    figure
    subplot(3,1,1)
    plot(windInput.Time,[data(:,1),U])
    subplot(3,1,2)
    plot(windInput.Time,[data(:,2),V])
    subplot(3,1,3)
    plot(windInput.Time,[data(:,3),W])
    
    windInput.Data = [U, V W ];
    save( [ inFolder '/' windFiles{i} 'NoDrops' '.mat'], 'windInput' );
    
end