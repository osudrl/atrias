G = 20;

% Torque sensitivity
Kt = 0.427;

data = importdata('atrias_log_jun2120112.dat', ',', 1);

t = 1e-3 * ( 1 : size(data.data, 1) )';

mtr_angA = data.data(:, 2);
mtr_angB = data.data(:, 3);

leg_angA = data.data(:, 4);
leg_angB = data.data(:, 5);

mtr_velA = data.data(:, 7);
mtr_velB = data.data(:, 8);

mtr_trqA = data.data(:, 14);
mtr_trqB = data.data(:, 15);

% ind = ( 1 : size( t, 1 ) )';
ind = ( 3.2e5 : 3.35e5 )';

figure( 'name', 'Motor Angles' );
% plot( t, [mtr_angA, mtr_angB, leg_angA, leg_angB], 'linewidth', 3 );
plot( ind(1) : ind(end), [mtr_trqA(ind), abs( mtr_angA(ind) - leg_angA(ind) ),...
    mtr_trqB(ind), abs( mtr_angB(ind) - leg_angB(ind) )],...
    'linewidth', 3 );
% xlabel( 'Time (s)' );
% ylabel( 'Angle (rad)' );
% ylim( [ -2 * pi, 2 * pi ] );
legend( 'Motor Torque A', 'Spring Deflection A', 'Motor Torque B', 'Spring Deflection B' );

tauA = Kt * mean( abs( mtr_trqA(ind) ) );
tauB = Kt * mean( abs( mtr_trqB(ind) ) );

defA = mean( abs( mtr_angA(ind) - leg_angA(ind)) );
defB = mean( abs( mtr_angB(ind) - leg_angB(ind)) );

fprintf( '\n\n\n\n' );

fprintf( 'Spring A:\n' );
% fprintf( '\tMotor Torque: %f +/- %f A\n', Kt * mean( abs( mtr_trqA(ind) ) ), std( mtr_trqA(ind) ) );
% fprintf( '\tSpring Deflection: %f +/- %f rad\n', mean( abs( mtr_angA(ind) - leg_angA(ind)) ),...
%     std( abs( mtr_angA(ind) - leg_angA(ind) ) ) );
fprintf( 'k = %f Nm/rad\n', G * tauA / defA ); 

fprintf( 'Spring B:\n' );
% fprintf( '\tMotor Torque: %f +/- %f A\n', mean( abs( mtr_trqB(ind) ) ), std( mtr_trqB(ind) ) );
% fprintf( '\tSpring Deflection: %f +/- %f\n', mean( abs( mtr_angB(ind) - leg_angB(ind)) ),...
%     std( abs( mtr_angB(ind) - leg_angB(ind) ) ) );
fprintf( 'k = %f Nm/rad\n', G * tauB / defB ); 

fprintf( '\n\n\n\n' ); 

% figure( 'name', 'Motor Velocities' );
% plot( t, [mtr_trqA, mtr_trqB], 'linewidth', 3 );
% xlabel( 'Time (s)' );
% ylabel( 'Angular Velocity (rad/s)' );

% figure( 'name', 'Motor Torques' );
% plot( t, [mtr_trqA, mtr_trqB], 'linewidth', 3 );
% xlabel( 'Time (s)' );