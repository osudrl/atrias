#include "atrias_asc_loader/ASCLoader.hpp"

namespace atrias {
namespace controller {

ASCLoader::ASCLoader() {
	subcontroller = nullptr;
}

RTT::TaskContext* ASCLoader::load(RTT::TaskContext* task_context, std::string package, std::string type) {
	// Sanity checks
	if (!task_context) {
		reportLoadError("Null task_context!", task_context, package, type);
		return nullptr;
	}
	if (package.empty()) {
		reportLoadError("Empty package name given!", task_context, package, type);
		return nullptr;
	}
	if (type.empty()) {
		reportLoadError("Empty component type given!", task_context, package, type);
		return nullptr;
	}

	// Verify that we haven't been run before.
	if (subcontroller) {
		reportLoadError("load() has already been called!", task_context, package, type);
		return nullptr;
	}

	// Obtain access to the deployer
	deployer = task_context->getPeer("Deployer");
	if (!deployer) {
		reportLoadError("Unable to access the deployer!", task_context, package, type);
		return nullptr;
	}

	// Let's import this package, so the deployer recognizes the component type.
	RTT::OperationCaller<bool(std::string)> import;
	import = deployer->getOperation("import");
	if (!import.ready()) {
		reportLoadError("Could not access deployer's import operation!", task_context, package, type);
		return nullptr;
	}
	if (!import(package)) {
		reportLoadError("Unable to import package!", task_context, package, type);
		return nullptr;
	}

	// And create a name for our new component.
	// First, we need access to the Controller Manager
	RTT::TaskContext* controllerManager = deployer->getPeer("atrias_cm");
	if (!controllerManager) {
		reportLoadError("Could not access the Controller Manager!", task_context, package, type);
		return nullptr;
	}

	// And now to actually get the name.
	RTT::OperationCaller<std::string(std::string, std::string)> getUniqueName;
	getUniqueName = controllerManager->getOperation("getUniqueName");
	if (!getUniqueName.ready()) {
		reportLoadError("Could not get the Controller Manager's getUniqueName Operation!", task_context, package, type);
		return nullptr;
	}
	name = getUniqueName(task_context->getName(), type);

	// And actually load the controller
	RTT::OperationCaller<bool(std::string, std::string)> loadComponent;
	loadComponent = deployer->getOperation("loadComponent");
	if (!loadComponent.ready()) {
		reportLoadError("Could not get the deployer's loadComponent Operation!", task_context, package, type);
		return nullptr;
	}
	if (!loadComponent(name, type)) {
		reportLoadError("Could not load the subcontroller!", task_context, package, type);
		return nullptr;
	}

	// We need to be able to talk to this component.
	// Note: After construction, the deployer automatically adds
	// it as a peer, but it's only a one-way link.
	subcontroller = deployer->getPeer(name);
	if (!subcontroller) {
		reportLoadError("Could not connect with the subcontroller!", task_context, package, type);
		return nullptr;
	}

	// Make the deployer a peer of this component.
	// This makes the link two-way
	if (!subcontroller->addPeer(deployer)) {
		reportLoadError("Could not add the deployer as a peer of the subcontroller!", task_context, package, type);
		return nullptr;
	}

	// Configure it.
	RTT::OperationCaller<bool(void)> configureController;
	configureController = subcontroller->getOperation("configure");
	if (!configureController.ready()) {
		reportLoadError("Unable to get subcontroller's configure operation!", task_context, package, type);
		return nullptr;
	}
	if (!configureController()) {
		reportLoadError("Subcontroller configuration failed!", task_context, package, type);
		return nullptr;
	}

	// And start it.
	RTT::OperationCaller<bool(void)> startController;
	startController = subcontroller->getOperation("start");
	if (!startController.ready()) {
		reportLoadError("Unable to get the subcontroller's start operation!", task_context, package, type);
		return nullptr;
	}
	if (!startController()) {
		reportLoadError("Subcontroller starting failed!", task_context, package, type);
		return nullptr;
	}

	return subcontroller;
}

ASCLoader::~ASCLoader() {
	// Verify that the controller's actually been loaded.
	if (!subcontroller)
		return;
	
	// Stop it
	RTT::OperationCaller<bool(void)> stopController;
	stopController = subcontroller->getOperation("stop");
	if (!stopController.ready()) {
		reportUnloadError("Could not acquire stop operation!");
	} else {
		stopController();
	}

	// And tell it to clean up.
	RTT::OperationCaller<bool(void)> cleanupController;
	cleanupController = subcontroller->getOperation("cleanup");
	if (!cleanupController.ready()) {
		reportUnloadError("Unable to acquire cleanup operation!");
	} else if (!cleanupController()) {
		reportUnloadError("Unable to cleanup subcontroller!");
	}

	// Then unload it.
	RTT::OperationCaller<bool(std::string)> unloadComponent;
	unloadComponent = deployer->getOperation("unloadComponent");
	if (!unloadComponent.ready()) {
		reportUnloadError("Unable to acquire deployer's unloadComponent operation!");
	} else if (!unloadComponent(name)) {
		reportUnloadError("Unable to unload the component!");
	}
}

void ASCLoader::reportLoadError(std::string       message,
                                RTT::TaskContext* task_context,
                                std::string       package,
                                std::string       type)
{
	log(RTT::Error) << "[ASCLoader] " << message;

	// Let them know what controller this is happening in
	if (task_context) {
		log(RTT::Error) << " Calling component: " << task_context->getName();
	}

	// More info to help them:
	log(RTT::Error) << " Requested component: \"" << type
	                << "\" from package: \"" << package << "\"" << RTT::endlog();
}

void ASCLoader::reportUnloadError(std::string message) {
	log(RTT::Warning) << "[ASCLoader] " << message
	                << " Subcontroller name: "
	                << subcontroller->getName() << RTT::endlog();
}

}
}

// vim: noexpandtab
