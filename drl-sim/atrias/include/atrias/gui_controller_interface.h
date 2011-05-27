// Devin Koepl

#include <atrias/controller.h>
#include <gui_msg.h>

typedef struct
{
	int lock;

	GuiMsg gui_msg;
	ControllerInput controller_input;
	ControllerOutput controller_output;
} GuiControllerInterface;
