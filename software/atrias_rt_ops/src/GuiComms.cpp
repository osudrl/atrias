#include "atrias_rt_ops/GuiComms.h"

namespace atrias {

namespace rtOps {

GuiComms::GuiComms(RTOps *rt_ops) :
          guiInPort("rt_ops_gui_in")
{
	rtOps = rt_ops;
	rtOps->addEventPort(guiInPort);
	checkNeeded = false;
	guiIn = RtOpsState::DISABLED;
}

RtOpsState GuiComms::getGuiCmd() {
	if (checkNeeded) {
		// We don't simply do checkNeeded = guiInPort.read() here because
		// that would cause a race condition between this thread and the
		// \a updateHook() thread.
		checkNeeded = false;
		if (guiInPort.read(guiIn)) {
			rtOps->sendEvent(buildEvent(RtOpsEvent::ACK_GUI));
			// There may be another message in the port after this one.
			checkNeeded = true;
		}
	}

	return guiIn;
}

void GuiComms::updateHook() {
	// If we were to do the check in this thread, we would need to
	// use a lock or complicated lockless data communication to get
	// the new command into the controller thread -- hence we just
	// tell the other thread to check for us.
	checkNeeded = true;
}

}

}

// vim: noexpandtab
