// Devin Koepl

#include <stdio.h>
#include <stdlib.h>

#include <atrias_controllers/controller.h>
#include <atrias_controllers/data_logger.h>

DataLogger data_logger = DataLogger( "test.dat" );

int main( int argc, char **argv )
{
	int i;

	ControllerInput controller_input = { 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. };
	ControllerOutput controller_output = { 0., 0. };
	DataToUspace log_entry = { 0, controller_input, controller_output };

	for ( i = 0; i < 10; i++ )
		data_logger.log( &log_entry );
}
