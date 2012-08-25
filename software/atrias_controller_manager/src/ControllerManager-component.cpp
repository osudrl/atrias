/*
 * ControllerManager-component.cpp
 *
 * This is the Orocos component for the Atrias Controller Manager. It handles
 * the loading, unloading, starting, and stopping of controllers. It also acts
 * as the intermediary between the GUI and RT Ops.
 *
 *  Created on: Jul 26, 2012
 *      Author: Michael Anderson
 */

#include <atrias_controller_manager/ControllerManager-component.h>

namespace atrias {
namespace controllerManager {

ControllerManager::ControllerManager(std::string const& name) :
        TaskContext(name),
                guiDataIn("gui_data_in"),
                guiDataOut("gui_data_out"),
                rtOpsDataIn("rt_ops_data_in"),
                rtOpsDataOut("rt_ops_data_out")
{
    addPort(guiDataOut);
    addEventPort(guiDataIn);
    addPort(rtOpsDataOut);
    addEventPort(rtOpsDataIn);

    this->addOperation("getUniqueName", &ControllerManager::getUniqueName, this, ClientThread)
            .doc("Get a unique name for a sub-controller given its type and the name of its parent.");
    this->addOperation("resetControllerNames", &ControllerManager::resetControllerNames, this, ClientThread)
            .doc("Free all unique names created for sub-controllers and make them re-available for assignment.");

    state = ControllerManagerState::NO_CONTROLLER_LOADED;
    lastError = ControllerManagerError::NO_ERROR;
    currentControllerName = "";
    controllerLoaded = false;
    commandPending = false;

    eManager = new EventManager(this);
    eManager->start();

    rosbagPID = 0;
    //std::cout << "AtriasControllerManager constructed !" << std::endl;
}

bool ControllerManager::configureHook() {
    // We need to get the Deployer's instance of the ScriptingService because
    // only it has access to the deployer's operations (like loadComponent)
    scriptingProvider = boost::dynamic_pointer_cast<scripting::ScriptingService>(getPeer("Deployer")->provides()->getService("scripting"));
    assert(scriptingProvider);
    std::cout << "AtriasControllerManager configured !" << std::endl;
    return true;
}

bool ControllerManager::startHook() {
    std::cout << "AtriasControllerManager started !" << std::endl;
    return true;
}

void ControllerManager::updateHook() {
    // Have we been contacted by RT Ops?
    if (NewData == rtOpsDataIn.read(rtOpsOutput)) {
        std::cout << "Got event from RT ops! Event #: " << (int)rtOpsOutput.event << std::endl;
        eManager->eventCallback((RtOpsEvent)rtOpsOutput.event);
    }

    //Any errors are ancient history
    lastError = ControllerManagerError::NO_ERROR;

    // Have we received a command from the GUI?
    if (NewData == guiDataIn.read(guiOutput)) {
        // Spawn rosbag logging node when requested.
        if (guiOutput.enableLogging == true && rosbagPID == 0) {
            // Spawn a child to run the program.
            rosbagPID = fork();
            if (rosbagPID == 0) { // Child process
                int corePID = fork();
                int execResult;
                if (corePID == 0) {
                    execResult = execlp("rosrun", "rosrun", "atrias", "rosbag_move_cores.sh", NULL);
                    if (execResult < 0) {
                        log(Warning) << "[ControllerManager] Failed to run rosbag_move_cores.sh script!" << endlog();
                    }
                    else {
                        log(Info) << "[ControllerManager] Moved rosbag processes to non-realtime core." << endlog();
                    }
                    exit(127);
                }
                else {
                    execResult = execlp("roslaunch", "roslaunch", "atrias", "rosbag.launch", NULL);
                    if (execResult < 0) {
                        log(Warning) << "[ControllerManager] Failed to exec rosbag logger!" << endlog();
                    }
                    else {
                        log(Info) << "[ControllerManager] Launched rosbag." << endlog();
                    }
                    exit(127);
                }
            }
            //else {
            //    printf("Parent here! My child is %d.\n", (int) rosbagPID);
            //}
        }
        // Kill rosbag node when requested.
        else if (guiOutput.enableLogging == false && rosbagPID != 0) {
            kill(rosbagPID, SIGINT);
            rosbagPID = 0;
        }

        commandBuffer.push_back((UserCommand)guiOutput.command);
    }

    if (!commandPending && !(commandBuffer.empty())) {
        os::MutexTryLock tryLock(commandRunMutex);
        if (tryLock.isSuccessful()) {
            handleUserCommand(commandBuffer.front());
            commandBuffer.pop_front();
            commandRunMutex.unlock();
        }
    }

    //Update the GUI with all of our new information
    updateGui();
    //std::cout << "AtriasControllerManager executes updateHook !" << std::endl;
}

void ControllerManager::stopHook() {
    std::cout << "AtriasControllerManager executes stopping !" << std::endl;
}

void ControllerManager::cleanupHook() {
    std::cout << "AtriasControllerManager cleaning up !" << std::endl;
}

void ControllerManager::updateGui() {
    guiInput.status = (ControllerManagerState_t) state;
    guiInput.errorType = (ControllerManagerError_t) lastError;
    guiDataOut.write(guiInput);
}

void ControllerManager::handleUserCommand(UserCommand command) {
    switch (state) {
        //We're ready to load a controller
        case ControllerManagerState::NO_CONTROLLER_LOADED: {
            //We're only going to try if the GUI is set to stopped
            if (command == UserCommand::STOP) {
                loadController(guiOutput.requestedController);
            }
            break;
        }
        //We've got a controller loaded
        case ControllerManagerState::CONTROLLER_STOPPED: {
            //Does the controller say that we should start it?
            if (command == UserCommand::RUN) {
                state = ControllerManagerState::CONTROLLER_RUNNING;
                //Tell RT Ops to start the controller
                rtOpsDataOut.write((RtOpsCommand_t) RtOpsCommand::ENABLE);
            }
            //Should we restart?
            else if (command == UserCommand::UNLOAD_CONTROLLER) {
                unloadController();
            }
            break;
        }
        case ControllerManagerState::CONTROLLER_RUNNING: {
            //Should we stop the controller?
            if (command == UserCommand::STOP) {
                state = ControllerManagerState::CONTROLLER_STOPPED;
                //Tell RT Ops to start the controller
                rtOpsDataOut.write((RtOpsCommand_t) RtOpsCommand::DISABLE);
            }
            //Should we restart?
            else if (command == UserCommand::UNLOAD_CONTROLLER) {
                unloadController();
            }
            else if (command == UserCommand::E_STOP) {
                throwEstop();
            }
            break;
        }
        case ControllerManagerState::CONTROLLER_ESTOPPED: {
            //We'll only respond to a restart command
            //Once we're back to stopped state the GUI should send a new
            //command to load the controller
            if (command == UserCommand::UNLOAD_CONTROLLER) {
                //Unload the controller (if present) and reset rt ops
                unloadController();
            }
            break;
        }
    }
}

void ControllerManager::throwEstop(bool alertRtOps) {
    state = ControllerManagerState::CONTROLLER_ESTOPPED;
    if (alertRtOps) {
        eManager->setEventWait(RtOpsEvent::ACK_E_STOP);
        rtOpsDataOut.write((RtOpsCommand_t) RtOpsCommand::E_STOP);
    }
}

bool ControllerManager::loadController(string controllerName) {
    if (state == ControllerManagerState::NO_CONTROLLER_LOADED) {
        //Make sure that an actual controller was specified
        if (controllerName != "" && controllerName != "none") {
            string path = ros::package::getPath(controllerName);
            metadata = controllerMetadata::loadControllerMetadata(path, controllerName);
            if (scriptingProvider->runScript(metadata.startScriptPath)) {
                state = ControllerManagerState::CONTROLLER_STOPPED;
                eManager->setEventWait(RtOpsEvent::ACK_DISABLE);
                rtOpsDataOut.write((RtOpsCommand_t) RtOpsCommand::DISABLE);
                currentControllerName = controllerName;
                return true;
            }
            //Something is wrong with the load script
            //loadStateMachine will have already set the error flag
            return false;
        }
        else {
            //We didn't encounter an error, but neither
            //did we load a controller
            return true;
        }
    }
    //No such controller, set our error flag
    lastError = ControllerManagerError::CONTROLLER_PACKAGE_NOT_FOUND;
    return false;
}

/*
 * This function performs the actual unloading of the controller.
 * The reset flag should be set if the controller is being reset,
 * not just unloaded.
 */
void ControllerManager::unloadController() {
    eManager->setEventWait(RtOpsEvent::ACK_RESET);
    rtOpsDataOut.write((RtOpsCommand_t) RtOpsCommand::RESET);
}

/*bool ControllerManager::waitForRtOpsState(RtOpsCommand rtoState) {
    os::TimeService::ticks startTicks = os::TimeService::Instance()->getTicks();
    while (true) {
        if (NewData == rtOpsDataIn.read(rtOpsOutput)) {
            //if (rtOpsOutput.rtOpsStatus == (uint8_t)rtoState)
            if ((RtOpsCommand) rtOpsOutput.rtOpsStatus == rtoState)
                return true;
        }
        if (os::TimeService::Instance()->secondsSince(startTicks) > RT_OPS_WAIT_TIMEOUT_SECS)
            return false;
    }
    return false;
}*/

void ControllerManager::setState(ControllerManagerState newState) {
    if (newState == ControllerManagerState::CONTROLLER_STOPPED) {
        controllerLoaded = true;
    }
    else if (newState == ControllerManagerState::NO_CONTROLLER_LOADED) {
        if (controllerLoaded) {
            assert(scriptingProvider->runScript(metadata.stopScriptPath));
            currentControllerName = "";
            controllerLoaded = false;
        }
    }
    state = newState;
    updateGui();

    if (!commandPending && !(commandBuffer.empty())) {
        os::MutexTryLock tryLock(commandRunMutex);
        if (tryLock.isSuccessful()) {
            handleUserCommand(commandBuffer.front());
            commandBuffer.pop_front();
            commandRunMutex.unlock();
        }
    }
}

ControllerManagerState ControllerManager::getState() {
    return state;
}

string ControllerManager::getUniqueName(string parentName, string childType) {
    uint16_t count;

    // Tack on the delimiter and type of the child to get a base name
    parentName.append(CONTROLLER_LEVEL_DELIMITER);
    parentName.append(childType);

    /*
     * Ugh. So map::insert() returns a std::pair which contains two values.
     * The second value is a boolean which is true if the map already contained
     * a pair with that key. The first value is an iterator (basically a pointer)
     * which points to yet another pair. That pair is the pair actually stored
     * within the map, containing the key and value for that entry.
     *
     * Still with me? We want to check the boolean stored in the second field of
     * the first pair to see whether this is the first time we've encountered this
     * base name. If so, we want to append a 0. If not, we want to append 1 more
     * than the last value we assigned for this base name.
     *
     * To do this, we dereference the iterator to the second pair, then access its
     * second value to get the number of times we've assigned a number for this
     * base name. We then increment it (which updates it inside the map) and
     * assign it to a temporary variable so that we can append it to our final
     * unique name.
     */
    os::MutexLock lock(nameCacheMutex);
    std::pair<std::map<string, uint16_t>::iterator, bool> result =
            controllerChildCounts.insert(std::make_pair(parentName, uint8_t()));
    if (result.second) {
        count = 0;
    }
    else {
        //Add 1 to the count in the map then use the resulting value
        count = ++(result.first->second);
    }

    //Now we add the underscore and the number that make the name unique
    parentName.append("_");
    char temp[6];
    sprintf(temp, "%u", count);
    parentName.append(temp);

    return parentName;
}

void ControllerManager::resetControllerNames() {
    os::MutexLock lock(nameCacheMutex);
    controllerChildCounts.clear();
}

ORO_CREATE_COMPONENT(ControllerManager);
}
}
