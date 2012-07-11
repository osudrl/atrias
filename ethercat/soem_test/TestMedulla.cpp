#include "TestMedulla.h"

TestMedulla::TestMedulla(ec_slavet* slave) {
	/* These are in the order of the PDO entries in the ESI XML file.
	 * I'd love cur_index to be a void * here, but C doesn't like doing pointer
	 * arithmetic w/ void pointers due to an unhelpful "feature" of the language
	 * (specifically, cur_index + 1 is equivalent to &(cur_index[1]) regardless
	 * of type...)
	 */
	uint8_t * cur_index = slave->outputs;
	
	command             = (uint8_t*) cur_index;
	cur_index          += sizeof(*command);
	
	current             = (uint16_t*) cur_index;
	cur_index          += sizeof(*command);
	
	cur_index = slave->inputs;
	
	timestamp           = (uint16_t*) cur_index;
	cur_index          += sizeof(*timestamp);
	
	encoder             = (uint32_t*) cur_index;
	cur_index          += sizeof(*encoder);
}
