// Devin Koepl

#include <atrias_controllers/data_logger.h>

DataLogger::DataLogger( const char *filename )
{
	// Open log file for writing.
	fp = fopen( filename, "w" );

	// Create file header.
	fprintf( fp, "Body Angle, Motor Angle A, Motor Angle B, Leg Angle A, Leg Angle B, Body Angular Velocity, Motor Velocity A, Motor Velocity B, Leg Velocity A, Leg Velocity B, Height, Horizontal Velocity, Vertical Velocity, Motor Torque A, Motor Torque B\n");		
}

DataLogger::~DataLogger()
{
	fclose(fp);
}

// Log a new entry.
void DataLogger::log( DataToUspace *data )
{
	fprintf( fp, "%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", data->controller_input.body_angle, data->controller_input.motor_angleA, data->controller_input.motor_angleB,
		data->controller_input.leg_angleA, data->controller_input.leg_angleB, data->controller_input.body_ang_vel, data->controller_input.motor_velocityA, data->controller_input.motor_velocityB,
		data->controller_input.leg_velocityA, data->controller_input.leg_velocityB, data->controller_input.height, data->controller_input.horizontal_velocity, data->controller_input.vertical_velocity,
		data->controller_output.motor_torqueA, data->controller_output.motor_torqueB );
}
