#include "atrias_component_loader/ComponentLoader.hpp"

namespace atrias {
namespace ComponentLoader {

ComponentLoader::ComponentLoader() {
	component = nullptr;
}

RTT::TaskContext* ComponentLoader::loadComponent(RTT::TaskContext* task_context, std::string package, std::string type) {
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

	// And actually load the component
	RTT::OperationCaller<bool(std::string, std::string)> loadComponent;
	loadComponent = deployer->getOperation("loadComponent");
	loadComponent(name, type);

	// We need to be able to talk to this component.
	// Note: After construction, the deployer automatically adds
	// it as a peer.
	component = deployer->getPeer(name);

	// Configure it.
	RTT::OperationCaller<bool(void)> configureComponent;
	configureComponent = component->getOperation("configure");
	configureComponent();

	// And start it.
	RTT::OperationCaller<bool(void)> startComponent;
	startComponent = component->getOperation("start");
	startComponent();

	return component;
}

ComponentLoader::~ComponentLoader() {
	// Verify that the component's actually been loaded.
	if (!component)
		return;
	
	// Stop it
	RTT::OperationCaller<bool(void)> stopComponent;
	stopComponent = component->getOperation("stop");
	stopComponent();

	// And tell it to clean up.
	RTT::OperationCaller<bool(void)> cleanupComponent;
	cleanupComponent = component->getOperation("cleanup");
	cleanupComponent();

	// Then unload it.
	RTT::OperationCaller<bool(std::string)> unloadComponent;
	unloadComponent = deployer->getOperation("unloadComponent");
	unloadComponent(name);
}

}
}

// vim: noexpandtab
