#include "atrias_asc_loader/ASCLoader.hpp"

namespace atrias {
namespace controller {

ASCLoader::ASCLoader() {
	subcontroller = nullptr;
}

RTT::TaskContext* ASCLoader::load(RTT::TaskContext* task_context, std::string package, std::string type) {
	// Obtain access to the deployer
	deployer = task_context->getPeer("Deployer");

	// Let's import this package, so the deployer recognizes the component type.
	RTT::OperationCaller<bool(std::string)> import;
	import = deployer->getOperation("import");
	import(package);

	// And create a name for our new component.
	RTT::TaskContext* controllerManager = deployer->getPeer("atrias_cm");
	RTT::OperationCaller<std::string(std::string, std::string)> getUniqueName;
	getUniqueName = controllerManager->getOperation("getUniqueName");
	name = getUniqueName(task_context->getName(), type);

	// And actually load the controller
	RTT::OperationCaller<bool(std::string, std::string)> loadComponent;
	loadComponent = deployer->getOperation("loadComponent");
	loadComponent(name, type);

	// We need to be able to talk to this component.
	// Note: After construction, the deployer automatically adds
	// it as a peer, but it's only a one-way link.
	subcontroller = deployer->getPeer(name);

	// Make the deployer a peer of this component.
	// This makes the link two-way
	subcontroller->addPeer(deployer);

	// Configure it.
	RTT::OperationCaller<bool(void)> configureController;
	configureController = subcontroller->getOperation("configure");
	configureController();

	// And start it.
	RTT::OperationCaller<bool(void)> startController;
	startController = subcontroller->getOperation("start");
	startController();

	return subcontroller;
}

ASCLoader::~ASCLoader() {
	// Verify that the controller's actually been loaded.
	if (!subcontroller)
		return;
	
	// Stop it
	RTT::OperationCaller<bool(void)> stopController;
	stopController = subcontroller->getOperation("stop");
	stopController();

	// And tell it to clean up.
	RTT::OperationCaller<bool(void)> cleanupController;
	cleanupController = subcontroller->getOperation("cleanup");
	cleanupController();

	// Then unload it.
	RTT::OperationCaller<bool(std::string)> unloadComponent;
	unloadComponent = deployer->getOperation("unloadComponent");
	unloadComponent(name);
}

}
}

// vim: noexpandtab
