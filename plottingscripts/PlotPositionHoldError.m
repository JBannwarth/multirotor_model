% Compute statistics
dt = str2double( get_param( 'MultirotorSimulationController/To Workspace errX', 'SampleTime') );
windSpeedX = [0;2.55999837265402;3.45522298273824;4.40686338715537;5.17829923510479;6.09013240228385;6.79493302374613];
expStdX    = [0.0296679478599681;0.105171083773901;0.0716849769094517;0.0663881874801405;0.0612953353079428;0.0799559016743535;0.103126049099762];
for i = 1:length( output )
    % Get error
    error(i).x = output(i).get('posTrackingX');
    error(i).t = (0:dt:Simulation.T_END)';
    
    % UAV takes about 20 s to recover from initial wind
    error(i).x = error(i).x( error(i).t >= 30 );
    error(i).t = error(i).t( error(i).t >= 30 ) - 30;
    
    meanX(i) = mean( error(i).x );
    stdX(i)  =  std( error(i).x );
end

figure( 'Name', 'Simulation error statistics' )
subplot(2,1,1); hold on; grid on; box on
plot( windSpeedX, meanX, 'o' )
ylabel('Mean $x$-axis error (m)', 'Interpreter', 'LaTeX')
xlabel('Wind speed (m/s)', 'Interpreter', 'LaTeX')
set( gca, 'TickLabelInterpreter', 'latex' )

subplot(2,1,2); hold on; grid on; box on
plot( windSpeedX, stdX, 'o', windSpeedX, expStdX, '+' )
ylabel('Stdev $x$-axis error (m)', 'Interpreter', 'LaTeX')
xlabel('Wind speed (m/s)', 'Interpreter', 'LaTeX')
set( gca, 'TickLabelInterpreter', 'latex' )
legend( { 'sim', 'exp' }, 'Interpreter', 'LaTeX')