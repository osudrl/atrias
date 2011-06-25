% Devin Koepl
% scp drl@bors:/home/drl/atrias/drl-sim/atrias/log_files/atrias_log.dat atrias_log.dat

G = 20;

% Torque sensitivity
Kt = 0.427;

data = importdata('atrias_log.dat', ',', 1);

t = 1e-3 * ( 1 : size(data.data, 1) )';

mtr_angA = data.data(:, 2);
mtr_angB = data.data(:, 3);

leg_angA = data.data(:, 4);
leg_angB = data.data(:, 5);

mtr_velA = data.data(:, 7);
mtr_velB = data.data(:, 8);

height   = data.data(:, 11);

mtr_curA = data.data(:, 14);
mtr_curB = data.data(:, 15);

toe_switch = data.data(:, 18);

command = data.data(:, 19);

defA = abs( mtr_angA - leg_angA );
defB = abs( mtr_angB - leg_angB );

ind = ( 1 : size( t, 1 ) )';
% ind = ( 3.2e5 : 3.35e5 )';

SUBPLOTS = 4;
figure( 'name', 'ATRIAS Experiment' );
% Toe Switch
subplot( SUBPLOTS, 1, 1 );
imagesc( t, 0, toe_switch' );
title( 'Toe Switch' );
xlabel( 'Time (s)' );
axis([0 t(end) -0.5, 0.5] );
% CoM Height
subplot( SUBPLOTS, 1, 2);
plot( t, height, 'linewidth', 3 );
title( 'CoM Height' );
xlabel( 'Time (s)' );
ylabel( 'Height (m)' );
axis([0 t(end) 0 max(height)]);
% Motor Torques
subplot( SUBPLOTS, 1, 3);
plot( t, [mtr_curA, mtr_curB], 'linewidth', 3  );
title( 'Motor Current' );
xlabel( 'Time (s)' );
ylabel( 'Current (A)' );
legend( 'A', 'B' );
axis([0 t(end) min([mtr_curA; mtr_curB]) max([mtr_curA; mtr_curB])]);
% Spring Deflections
subplot( SUBPLOTS, 1, 4);
plot( t, [defA, defB] * 180 / pi, 'linewidth', 3  );
title( 'Spring Deflection' );
xlabel( 'Time (s)' );
ylabel( 'Deflection (deg)' );
legend( 'A', 'B' );
axis( [0 t(end) 0 max( [defA; defB] * 180/pi) ] );

% figure( 'name', 'Motor Angles' );
% plot( t, [mtr_trqA, mtr_trqB], 'linewidth', 3 );
% plot( t, [mtr_angA, mtr_angB, leg_angA, leg_angB], 'linewidth', 3 );
% plot( ind(1) : ind(end), [mtr_trqA(ind), abs( mtr_angA(ind) - leg_angA(ind) ),...
%     mtr_trqB(ind), abs( mtr_angB(ind) - leg_angB(ind) )],...
%     'linewidth', 3 );
% xlabel( 'Time (s)' );
% ylabel( 'Angle (rad)' );
% ylabel( 'Torque (Nm)' );
% ylim( [ -2 * pi, 2 * pi ] );
% legend( 'Motor Torque A', 'Spring Deflection A', 'Motor Torque B', 'Spring Deflection B' );
% legend( 'Motor Torque A', 'Motor Torque B' );

% tauA = Kt * mean( abs( mtr_trqA(ind) ) );
% tauB = Kt * mean( abs( mtr_trqB(ind) ) );

% defA = mean( abs( mtr_angA(ind) - leg_angA(ind)) );
% defB = mean( abs( mtr_angB(ind) - leg_angB(ind)) );

% fprintf( '\n\n\n\n' );
% 
% fprintf( 'Spring A:\n' );
% fprintf( '\tMotor Torque: %f +/- %f A\n', Kt * mean( abs( mtr_trqA(ind) ) ), std( mtr_trqA(ind) ) );
% fprintf( '\tSpring Deflection: %f +/- %f rad\n', mean( abs( mtr_angA(ind) - leg_angA(ind)) ),...
%     std( abs( mtr_angA(ind) - leg_angA(ind) ) ) );
% fprintf( 'k = %f Nm/rad\n', G * tauA / defA ); 
% 
% fprintf( 'Spring B:\n' );
% fprintf( '\tMotor Torque: %f +/- %f A\n', mean( abs( mtr_trqB(ind) ) ), std( mtr_trqB(ind) ) );
% fprintf( '\tSpring Deflection: %f +/- %f\n', mean( abs( mtr_angB(ind) - leg_angB(ind)) ),...
%     std( abs( mtr_angB(ind) - leg_angB(ind) ) ) );
% fprintf( 'k = %f Nm/rad\n', G * tauB / defB ); 
% 
% fprintf( '\n\n\n\n' ); 

% figure( 'name', 'Motor Velocities' );
% plot( t, [mtr_trqA, mtr_trqB], 'linewidth', 3 );
% xlabel( 'Time (s)' );
% ylabel( 'Angular Velocity (rad/s)' );

% figure( 'name', 'Motor Torques' );
% plot( t, [mtr_trqA, mtr_trqB], 'linewidth', 3 );
% xlabel( 'Time (s)' );