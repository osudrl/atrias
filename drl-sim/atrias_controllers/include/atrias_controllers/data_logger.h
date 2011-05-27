// Devin Koepl

#ifndef FUNCS_H_DATA_LOGGER
#define FUNCS_H_DATA_LOGGER

#include <stdio.h>

#include <atrias_controllers/controller.h>
#include <atrias_controllers/uspace_kern_shm.h>

class DataLogger
{
	private:
		FILE *fp;

	public:
		DataLogger( const char * );
		~DataLogger();

		void log( DataToUspace * ); 
};

#endif // FUNCS_H_DATA_LOGGER
