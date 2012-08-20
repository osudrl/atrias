#include <atrias_rt_ops/RTOps.h>

RTOps::RTOps(std::string name) :
        RTT::TaskContext(name),
                signal(0),
                cManagerDataOut("controller_manager_data_out"),
                cManagerDataIn("controller_manager_data_in"),
                /*gazeboDataIn("gazebo_data_in"),
                 gazeboDataOut("gazebo_data_out"),*/
                guiDataOut("rt_cycle_gui_out"),
                logDataOut("rt_cycle_log_out")
{
    counter = 0;
    comm = NULL;
    RTT::os::MutexLock lock(bigComponentLock);
    // Initialize controllerOutput.
    controllerOutput.lLeg.motorCurrentA = 0.0;
    controllerOutput.lLeg.motorCurrentB = 0.0;

    this->provides("timestamps")
            ->addOperation("getTimestamp", &RTOps::getTimestamp, this, RTT::OwnThread)
            .doc("Get timestamp.");
    this->provides("timestamps")
            ->addOperation("getRosHeader", &RTOps::getRosHeader, this, RTT::OwnThread)
            .doc("Get a ROS header to be used to timestamp data that will be logged.");
    this->requires("atc")
            ->addOperationCaller(runController);

    addPort(cManagerDataOut);
    addEventPort(cManagerDataIn);

    guiPubTimer = new GuiPublishTimer(50);
    controllerManagerPubTimer = new GuiPublishTimer(20);

    addPort(logDataOut);
    addPort(guiDataOut);

    controllerLoaded = false;

    cmOut.rtOpsStatus = (uint8_t) RtOpsCommand::NO_CONTROLLER_LOADED;
    //cmOut.medullasResetting = false;

    log(Info) << "[RTOps] constructed!" << endlog();
}

uint64_t RTOps::getTimestamp() {
    RTT::os::MutexLock lock(logTimestampLock);
    return logTimestamp;
}

Header RTOps::getRosHeader() {
    uint64_t time = getTimestamp();
    uint32_t nsec = time % SECOND_IN_NANOSECONDS;
    time /= SECOND_IN_NANOSECONDS;
    Header h;
    h.stamp.sec = time;
    h.stamp.nsec = nsec;
    return h;
}

Header RTOps::getInternalRosHeader() {
    uint64_t time = currentTimestamp;
    uint32_t nsec = time % SECOND_IN_NANOSECONDS;
    time /= SECOND_IN_NANOSECONDS;
    Header h;
    h.stamp.sec = time;
    h.stamp.nsec = nsec;
    return h;
}

void RTOps::newStateCallback() {
    cycleControlThread();
    /*if (cmOut.medullasResetting) {
     bool allReset = true;
     for (size_t t = 0; t < NUM_MEDULLAS_ON_ROBOT; t++) {
     if (cmOut.medullaStates[t] == medulla_state_error) {
     allReset = false;
     break;
     }
     }
     if (allReset) {
     cmOut.medullasResetting = false;
     cManagerDataOut.write(cmOut);
     }
     }*/
}

void RTOps::sendRobotState() {
    {
        RTT::os::MutexLock lock(robotStateLock);

        rtOpsCycle.header = getInternalRosHeader();

        rtOpsCycle.controllerOutput = controllerOutput;

        rtOpsCycle.commandedOutput.lLeg.motorCurrentA = CLAMP(controllerOutput.lLeg.motorCurrentA, MIN_MTR_TRQ_CMD, MAX_MTR_TRQ_CMD);
        rtOpsCycle.commandedOutput.lLeg.motorCurrentB = CLAMP(controllerOutput.lLeg.motorCurrentB, MIN_MTR_TRQ_CMD, MAX_MTR_TRQ_CMD);
        rtOpsCycle.commandedOutput.lLeg.motorCurrentHip = CLAMP(controllerOutput.lLeg.motorCurrentHip, MIN_HIP_MTR_TRQ_CMD, MAX_HIP_MTR_TRQ_CMD);

        // Send the 1 kHz stream for logging.
        logDataOut.write(rtOpsCycle);

        // Send the 50Hz stream to the ControllerManager
        if (controllerManagerPubTimer->readyToSend()) {
            cManagerDataOut.write(cmOut);
        }

        //Send a 20Hz stream to the GUI
        if (guiPubTimer->readyToSend()) {
            guiDataOut.write(rtOpsCycle);
        }
    }
    // Send RT Ops status here (I should probably add a mutex for that...)
}

bool RTOps::configureHook() {

    return true;
}

bool RTOps::startHook() {
    RTT::os::MutexLock lock(bigComponentLock);
    // Lock memory.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        perror("mlockall");
    }
    //this->getActivity()->setCpuAffinity(10);
    //RTT::Activity::setCpuAffinity(this->getActivity()->getCpuAffinity());
    comm = new ECatComm(this);

    if (comm->init()) {
        log(Info) << "[RTOps] EtherCAT initialized." << endlog();
        comm->start();
    	comm->setCpuAffinity(this->getActivity()->getCpuAffinity());

        stop = false;
        RTT::Activity::start();
    	RTT::Activity::setCpuAffinity(this->getActivity()->getCpuAffinity());
        //this->start();
        log(Info) << "[RTOps] started." << endlog();
        return true;
    }

    log(Info) << "[RTOps] EtherCAT failed to initialize, trying sim." << endlog();

    delete (comm);
    /*comm = new GazeboConnector(this);

     if (comm->init()) {
     log(Info) << "[RTOps] configured for Simulation link." << endlog();
     return true;
     }*/

    log(Info) << "[RTOps] Did not connect to simulation." << endlog();

    //delete(comm);
    comm = new NoopConnector(this);

    if (comm->init()) {
        log(Info) << "[RTOps] configured for execution with no I/O." << endlog();
        comm->start();

        stop = false;
        RTT::Activity::start();
        //this->start();
        log(Info) << "[RTOps] started." << endlog();
        return true;
    }

    log(Info) << "[RTOps] failed to configure communications!" << endlog();

    if (!comm) {
        log(Error) << "[RTOps] failed to start!" << endlog();
        return false;
    }

    return true;
}

void RTOps::updateHook() {
    RTT::os::MutexLock lock(bigComponentLock);
    if (NewData == cManagerDataIn.read(cmIn)) {
        //Have we received a new command?
        if (cmIn != cmOut.rtOpsStatus) {
            switch ((RtOpsCommand) cmIn) {
                case RtOpsCommand::NO_CONTROLLER_LOADED: {
                    if (comm) {
                        comm->disable();
                    }
                    cmOut.rtOpsStatus = (uint8_t) cmIn;
                    RTT::os::MutexLock lock(controllerLoadedLock);
                    controllerLoaded = false;
                    break;
                }
                case RtOpsCommand::DISABLE: {
                    if (comm) {
                        comm->disable();
                    }
                    zeroMotorTorques();
                    loadController();
                    cmOut.rtOpsStatus = (uint8_t) cmIn;
                    break;
                }
                case RtOpsCommand::ENABLE: {
                    if (comm) {
                        comm->enable();
                    }
                    loadController();
                    cmOut.rtOpsStatus = (uint8_t) cmIn;
                    break;
                }
                case RtOpsCommand::RESET: {
                    if (comm) {
                        comm->disable();
                        comm->leaveEStop();
                    }
                    zeroMotorTorques();
                    RTT::os::MutexLock lock(controllerLoadedLock);
                    cmOut.rtOpsStatus = (uint8_t) RtOpsCommand::NO_CONTROLLER_LOADED;
                    //cmOut.medullasResetting = true;
                    controllerLoaded = false;
                    break;
                }
                case RtOpsCommand::E_STOP: {
                    if (comm) {
                        comm->eStop();
                    }
                    zeroMotorTorques();
                    //cmOut.guiEStopCount++;
                    //cmOut.guiEStopCount++;
                    cmOut.rtOpsStatus = (uint8_t) cmIn;
                    break;
                }
                default: {
                    if (comm) {
                        comm->eStop();
                    }
                    zeroMotorTorques();
                    controllerLoaded = false;
                    //cmOut.badCommandCount++;
                    break;
                }
            }
            cManagerDataOut.write(cmOut);
        }
    }
    return;
}

void RTOps::loop() {
    while (!stop) {
        //log(Info) << "aoeu" << endlog();
        {
            // Update timestamp for distribution to components that do logging.
            RTT::os::MutexLock lock(logTimestampLock);
            logTimestamp = currentTimestamp;
        }
        {
            RTT::os::MutexLock lock(controllerLoadedLock);
            // Only run the controller if it's loaded.
            if (controllerLoaded) {
                atrias_msgs::rt_ops_cycle tempCycle;
                {
                    RTT::os::MutexLock lock(robotStateLock);
                    tempCycle = rtOpsCycle;
                }
                tempCycle.robotState.cmState = cmOut.rtOpsStatus;
                controllerOutput = runController(tempCycle.robotState);
            }
        }

        {
            RTT::os::MutexLock lock(bigComponentLock);
            comm->transmitHook();
        }
        signal.wait();
    }
}

void RTOps::loadController() {
    TaskContext *peer = this->getPeer("controller");

    if (peer) {
        runController = peer->provides("atc")->getOperation("runController");
        controllerLoaded = true;
    }
}

void RTOps::cycleControlThread() {
    if (!signal.value())
        signal.signal();
}

void RTOps::zeroMotorTorques() {
    controllerOutput.lLeg.motorCurrentA = 0.;
    controllerOutput.lLeg.motorCurrentB = 0.;
    controllerOutput.lLeg.motorCurrentHip = 0.;
}

bool RTOps::breakLoop() {
    stop = true;
    cycleControlThread();
    return true;
}

void RTOps::stopHook() {
    if (comm)
        comm->stop();

    RTT::Activity::stop();

    // Unlock memory.
    if (munlockall() == -1) {
        perror("munlockall");
    }

    log(Info) << "[RTOps] stopped!" << endlog();
}

void RTOps::cleanupHook() {
    RTT::os::MutexLock lock(bigComponentLock);
    delete (comm);
    log(Info) << "[RTOps] cleaned up!" << endlog();
}

ORO_CREATE_COMPONENT(RTOps)
