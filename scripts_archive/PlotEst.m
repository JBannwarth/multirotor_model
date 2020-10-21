rpy = out.get('rpy');
rpyEst = out.get('rpyEst');
figure;
subplot(3,1,1); hold on; grid on; box on;
t = rpy.time(rpy.time < 2);
val = rpy.signals.values( rpy.time < 2, 1 );
tEst = rpyEst.time( rpyEst.time < 2 );
valEst = rpyEst.signals.values( rpyEst.time < 2, 1 );
stairs( t, val*180/pi )
stairs( tEst, valEst*180/pi )
xlabel('Time (s)')
ylabel('Roll (deg)')
legend('actual', 'estimated')

subplot(3,1,2); hold on; grid on; box on;
t = rpy.time(rpy.time < 2);
val = rpy.signals.values( rpy.time < 2, 2 );
tEst = rpyEst.time( rpyEst.time < 2 );
valEst = rpyEst.signals.values( rpyEst.time < 2, 2 );
stairs( t, val*180/pi )
stairs( tEst, valEst*180/pi )
xlabel('Time (s)')
ylabel('Pitch (deg)')
legend('actual', 'estimated')

subplot(3,1,3); hold on; grid on; box on;
t = rpy.time(rpy.time < 2);
val = rpy.signals.values( rpy.time < 2, 3 );
tEst = rpyEst.time( rpyEst.time < 2 );
valEst = rpyEst.signals.values( rpyEst.time < 2, 3 );
stairs( t, val*180/pi )
stairs( tEst, valEst*180/pi )
xlabel('Time (s)')
ylabel('Yaw (deg)')
legend('actual', 'estimated')

pos = out.get('pos');
posEst = out.get('posEst');
figure;
subplot(3,1,1); hold on; grid on; box on;
t = pos.time(pos.time < 5);
val = pos.signals.values( pos.time < 5, 1 );
tEst = posEst.time( posEst.time < 5 );
valEst = posEst.signals.values( posEst.time < 5, 1 );
stairs( t, val )
stairs( tEst, valEst )
xlabel('Time (s)')
ylabel('x (m)')
legend('actual', 'estimated')

subplot(3,1,2); hold on; grid on; box on;
t = pos.time(pos.time < 5);
val = pos.signals.values( pos.time < 5, 2 );
tEst = posEst.time( posEst.time < 5 );
valEst = posEst.signals.values( posEst.time < 5, 2 );
stairs( t, val )
stairs( tEst, valEst )
xlabel('Time (s)')
ylabel('y (m)')
legend('actual', 'estimated')

subplot(3,1,3); hold on; grid on; box on;
t = pos.time(pos.time < 5);
val = pos.signals.values( pos.time < 5, 3 );
tEst = posEst.time( posEst.time < 5 );
valEst = posEst.signals.values( posEst.time < 5, 3 );
stairs( t, val )
stairs( tEst, valEst )
xlabel('Time (s)')
ylabel('z (m)')
legend('actual', 'estimated')