// Devin Koepl

#include <signal.h>
#include <stddef.h>
#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <ecatslaveapi.h>

#include <atrias/simulation_ecat_interface.h>

#include <drl_library/discretize.h>

#define MODULE_NAME             "ecs_atrias_simulation"

//#define DEBUG_ECAT
#undef DEBUG_ECAT

#define MODULE_NAME             "ecs_atrias_simulation"
#define PRINT_MSG               printf

#define TRUE	1
#define FALSE 0

//#define ECATSLAVE_APPCYCLE_IS_VERBOSE   1

/*==========================================================================*/
/* Definitions of constants to be used with setEcatSlaveSyncManagerMS():    */

/* Definitions for SyncManager used for "MBoxOut" (Mailbox data Master --> Slave) */
#define SM_MBOXOUT_ID               0           /* Select an ID of SyncManager. */
#define SM_MBOXOUT_MIN_LEN          4           /* Minimum lenght(in bytes) of SyncManager. */
#define SM_MBOXOUT_MAX_LEN          512         /* Maximum lenght(in bytes) of SyncManager. */
#define SM_MBOXOUT_MIN_ADR          0x1000      /* Minimum allowed physical start address for SyncManager. */
#define SM_MBOXOUT_MAX_ADR          0x1200      /* Maximum allowed physical start address for SyncManager. */

/* Definitions for SyncManager used for "MBoxIn"  (Mailbox data Slave  --> Master) */
#define SM_MBOXIN_ID                1           /* Select an ID of SyncManager. */
#define SM_MBOXIN_MIN_LEN           4           /* Minimum lenght(in bytes) of SyncManager. */
#define SM_MBOXIN_MAX_LEN           512         /* Maximum lenght(in bytes) of SyncManager. */
#define SM_MBOXIN_MIN_ADR           0x1200      /* Minimum allowed physical start address for SyncManager. */
#define SM_MBOXIN_MAX_ADR           0x1400      /* Maximum allowed physical start address for SyncManager. */

/* Definitions for SyncManager used for "Outputs" (Process Data Master --> Slave) */
#define SM_OUTPUTS_1_ID             2           /* Select an ID of SyncManager. */
#define SM_OUTPUTS_1_MIN_LEN        0           /* Minimum lenght(in bytes) of SyncManager. */
#define SM_OUTPUTS_1_MAX_LEN        512         /* Maximum lenght(in bytes) of SyncManager. */
#define SM_OUTPUTS_1_MIN_ADR        0x1400      /* Minimum allowed physical start address for SyncManager. */
#define SM_OUTPUTS_1_MAX_ADR        0x1400      /* Maximum allowed physical start address for SyncManager. */

/* Definitions for SyncManager used for "Inputs" (Process Data Slave  --> Master) */
#define SM_INPUTS_1_ID              3           /* Select an ID of SyncManager. */
#define SM_INPUTS_1_MIN_LEN         0           /* Minimum lenght(in bytes) of SyncManager. */
#define SM_INPUTS_1_MAX_LEN         512         /* Maximum lenght(in bytes) of SyncManager. */
#define SM_INPUTS_1_MIN_ADR         0x2000      /* Minimum allowed physical start address for SyncManager. */
#define SM_INPUTS_1_MAX_ADR         0x2000      /* Maximum allowed physical start address for SyncManager. */

/* Index(ID) of installed KPA EC Slave board.
   If only one board is installed then this ID is always 0. */
#define ECS_BOARD_ID                    0
/*==========================================================================*/

// #define TEST_DC_LATCH_ID                0

uControllerInput in;
uControllerOutput out;

static UINT32        g_EscPollPeriodMcs  = 10;   /* microseconds. 0 means no polling */
static UINT32        g_HeartBeatPeriodMs = 2000;    /* milliseconds. */
//static unsigned long g_AppCyclePeriodMcs = 1000000; /* microseconds. */
static unsigned long g_AppCyclePeriodMcs = 0; /* microseconds. */

static UINT16       g_IsExitRequested    = 0;
static UINT16       g_IsAppRunning       = 0;
static UINT16       g_InputsSize         = 0;
static UINT16       g_OutputsSize        = 0;
// static UINT16    start_read_dc = 0;

static UINT8 *      g_pOutputsBuf;
static UINT8 *      g_pInputsBuf;

/* Slave callback context data (just to demonstrate it).
   Both setEcatSlaveCallbackFunctionMS()
   and  removeEcatSlaveCallbackFunctionMS()
   MUST use the same pointer for pContextData otherwise
   removeEcatSlaveCallbackFunctionMS() will fail! */
static UINT8        g_SlaveEventCallbackContextData[32];

/*--------------------------------------------------------------------------*/
/* This function is an implementation of slave's callback function
   that is called when any not SDO-related slave stack event for which it was
   registered occurs.
   See setEcatSlaveCallbackFunctionMS() */
ECAT_RESULT OnSlaveEventCallbackFunction(
    IN UINT32    EventId,
    IN void*     pEventData,
    IN void*     pContextData,    /* pointer to data which was passed while registering this callback function */
    IN ECAT_BOOL bNeedWaitReturn  /* whether to wait for completion of this callback by user/kernel space wrapper function */
    )
{
    /* This function is an implementation of slave's callback function
    that is called when any not SDO-related slave stack event for which it was
    registered occurs. */

    //===== TO IMPLEMENT BY DEVELOPER: =========

    ECAT_RESULT Result;
    UINT16 wSmId;
    int j;

    switch (EventId) {

        case AL_EVENT_MAILBOX_START_REQ:
            Result = ECAT_S_OK;     //if you ready to start inputs update
            //Result = ECAT_E_FAIL; //else
            PRINT_MSG("AL_EVENT_MAILBOX_START_REQ (Wahhooo!)\n");
	    break;

        case AL_EVENT_MAILBOX_STOP_REQ:
            Result = ECAT_S_OK;     //if you ready to start inputs update
            //Result = ECAT_E_FAIL; //else
            PRINT_MSG("AL_EVENT_MAILBOX_STOP_REQ\n");
            break;

        case AL_EVENT_INPUTS_START_REQ:
            Result = ECAT_S_OK;     //if you ready to start inputs update
            //Result = ECAT_E_FAIL; //else
            PRINT_MSG("AL_EVENT_INPUTS_START_REQ\n");
            break;

        case AL_EVENT_OUTPUTS_START_REQ:
            Result = ECAT_S_OK;     //if you ready to start outputs update
            //Result = ECAT_E_FAIL; //else
            PRINT_MSG("AL_EVENT_OUTPUTS_START_REQ\n");
            break;

        case AL_EVENT_INPUTS_STOP_REQ:
            Result = ECAT_S_OK;
            PRINT_MSG("AL_EVENT_INPUTS_STOP_REQ\n");
            break;

        case AL_EVENT_OUTPUTS_STOP_REQ:
            Result = ECAT_S_OK;
            PRINT_MSG("AL_EVENT_OUTPUTS_STOP_REQ\n");
            break;

        case AL_EVENT_INPUTS_STARTED:
            /* Get current sizes of each "Inputs" SM used by the slave.
               This is needed because the slave application must allocate
               enough space for "inputs" buffers. */
            Result = getEcatSlaveConfiguredIoSize(SM_INPUTS_1_ID, &g_InputsSize);
            if (ECAT_FAILED(Result))
                PRINT_MSG("Can't get inputs size\n");
//             start_read_dc = 1;
            Result = ECAT_S_OK;
            PRINT_MSG("Inputs size: %d(0x%X hex)\n", g_InputsSize, g_InputsSize);
            break;

        case AL_EVENT_OUTPUTS_STARTED:
            /* Get current sizes of each "Outputs" SM used by the slave.
               This is needed because the slave application must allocate
               enough space for "outputs" buffers. */
            Result = getEcatSlaveConfiguredIoSize(SM_OUTPUTS_1_ID, &g_OutputsSize);
            if (ECAT_FAILED(Result))
                PRINT_MSG("Can't get outputs size\n");
            Result = ECAT_S_OK;
			PRINT_MSG("Outputs size: %d(0x%X hex)\n", g_OutputsSize, g_OutputsSize);
            break;

        case AL_EVENT_INPUTS_STOPPED:
            g_InputsSize = 0;
//             start_read_dc = 0;
            Result = ECAT_S_OK;
            PRINT_MSG("AL_EVENT_INPUTS_STOPPED\n");
            break;

        case AL_EVENT_OUTPUTS_STOPPED:
            g_OutputsSize = 0;
            Result = ECAT_S_OK;
            PRINT_MSG("AL_EVENT_OUTPUTS_STOPPED\n");
            break;

        case AL_EVENT_INPUTS_TRANSMITTED:
            //master has read some inputs
            wSmId = *((ECAT_WORD *)pEventData);

	    //PRINT_MSG("SM %d: AL_EVENT_INPUTS_TRANSMITTED\n", wSmId);
            Result = ECAT_S_OK;
            break;

        case AL_EVENT_OUTPUTS_RECEIVED:
            //master has written some outputs
            wSmId = *((ECAT_WORD *)pEventData);          
	   
		    //new torque here	  
		    Result = updateEcatSlaveRxOutputsMS(ECS_BOARD_ID, SM_OUTPUTS_1_ID,  g_pOutputsBuf);


	    // PRINT_MSG("SM %d: AL_EVENT_OUTPUTS_RECEIVED\n", wSmId);
            Result = ECAT_S_OK;
            break;

        case AL_EVENT_HEARTBEAT:
            PRINT_MSG("AL_EVENT_HEARTBEAT\n");
            // do something, for example reset any counter
            Result = ECAT_S_OK;
            break;
        default:
            PRINT_MSG("\nEPIC FAIL!!!\n");
	    Result = ECAT_E_FAIL;
    }

    return Result;
}

/*--------------------------------------------------------------------------*/
/* Sync managers configuration, Object dictionary creation,
   slave callbacks settings, app. loop, etc. */
void MainAppThreadFun(int data)
{
	PRINT_MSG("In MainAppThread.\n");

	ECAT_RESULT Result;
	UINT32      AlMask = 0;
	UINT64      app_cycle_counter = 0;

	UINT8 ALSt;
	UINT8 RQSt;

	g_IsAppRunning = 1;
	
	int tempf;
	int i;
	int counter = 0;
	
  /* Initialize slave stack: */
  printf("(>\")> Init Slave\n");
  Result = initEcatSlaveMS(ECS_BOARD_ID);
  if (ECAT_FAILED(Result))
  {
      PRINT_MSG("MainAppThreadFun: initEcatSlave returned error: 0x%X(%s)\n",
                Result, getEcatSlaveResultCodeDescription(Result));
      return;
  }

  changeEcatSlaveFoePassword(ECS_BOARD_ID, 0xF0EACCECULL/*old*/, 0x00000000/*new*/);

  //===== TO IMPLEMENT BY DEVELOPER: =========
  // do setEcatSlaveSyncManagerMS() for your SMs here:
  //========================================================================
  // Sets properties of slave's SyncManager.
  Result = setEcatSlaveSyncManagerMS(ECS_BOARD_ID,
		SM_MBOXOUT_ID,
		EcatSmTypeMbxOut,
		SM_MBOXOUT_MIN_LEN,
		SM_MBOXOUT_MAX_LEN,
		SM_MBOXOUT_MIN_ADR,
		SM_MBOXOUT_MAX_ADR);

  if (ECAT_FAILED(Result))
  {
      PRINT_MSG(MODULE_NAME":MainAppThreadFun: SM #%d, setEcatSlaveSyncManager returned error: 0x%X(%s)\n",
                SM_MBOXOUT_ID,
                Result, getEcatSlaveResultCodeDescription(Result));
      goto out_releaseEcatSlave;
  }

  // Sets properties of slave's SyncManager.
  printf("(>\")> Set SM properties\n");
  Result = setEcatSlaveSyncManagerMS(ECS_BOARD_ID,
		SM_MBOXIN_ID,
		EcatSmTypeMbxIn,
		SM_MBOXIN_MIN_LEN,
		SM_MBOXIN_MAX_LEN,
		SM_MBOXIN_MIN_ADR,
		SM_MBOXIN_MAX_ADR);
  
	if (ECAT_FAILED(Result))
  {
      PRINT_MSG(MODULE_NAME": MainAppThreadFun: SM #%d, setEcatSlaveSyncManager returned error: 0x%X(%s)\n",
      	SM_MBOXIN_ID,
        Result, getEcatSlaveResultCodeDescription(Result));
      goto out_releaseEcatSlave;
  }

  // Sets properties of slave's SyncManager.
  Result = setEcatSlaveSyncManagerMS(ECS_BOARD_ID,
		SM_INPUTS_1_ID,
		EcatSmTypeInputs,
		SM_INPUTS_1_MIN_LEN,
		SM_INPUTS_1_MAX_LEN,
		SM_INPUTS_1_MIN_ADR,
		SM_INPUTS_1_MAX_ADR);
  
	if (ECAT_FAILED(Result))
  {
      PRINT_MSG(MODULE_NAME": MainAppThreadFun: SM #%d, setEcatSlaveSyncManager returned error: 0x%X(%s)\n",
                SM_INPUTS_1_ID,
                Result, getEcatSlaveResultCodeDescription(Result));
      goto out_releaseEcatSlave;
  }

	// Sets properties of slave's SyncManager.
	Result = setEcatSlaveSyncManagerMS(ECS_BOARD_ID,
		SM_OUTPUTS_1_ID,
		EcatSmTypeOutputs,
		SM_OUTPUTS_1_MIN_LEN,
		SM_OUTPUTS_1_MAX_LEN,
		SM_OUTPUTS_1_MIN_ADR,
		SM_OUTPUTS_1_MAX_ADR);

	if (ECAT_FAILED(Result)) {
		PRINT_MSG(MODULE_NAME": MainAppThreadFun: SM #%d, setEcatSlaveSyncManager returned error: 0x%X(%s)\n",
					SM_OUTPUTS_1_ID,
					Result, getEcatSlaveResultCodeDescription(Result));
		goto out_releaseEcatSlave;
	}

	// Set polltime cycle period. Slave stack will poll AL event register periodically and update mailbox data.
	Result = setEcatSlavePollTimeMS(ECS_BOARD_ID, g_EscPollPeriodMcs); // specified as module parameter
	if (ECAT_FAILED(Result)) {
		PRINT_MSG(MODULE_NAME": MainAppThreadFun: setEcatSlavePollTime returned error: 0x%X(%s)\n",
					Result, getEcatSlaveResultCodeDescription(Result));
		goto out_releaseEcatSlave;
	}

	// Set period of Heart-beat event generation, See #AL_EVENT_HEARTBEAT
	// It returns ECAT_FAIL if poll period equyals 0
	Result = setEcatSlaveHeartbeatIntervalMS(ECS_BOARD_ID, g_HeartBeatPeriodMs);
    
	if (ECAT_FAILED(Result)) {
		PRINT_MSG(MODULE_NAME": MainAppThreadFun: setEcatSlaveHeartbeatInterval returned error: 0x%X(%s)\n",
					Result, getEcatSlaveResultCodeDescription(Result));
		goto out_releaseEcatSlave;
	}

	// Sets slave's event callback function.
	// See description of OnSlaveEventCallbackFunction(), tEcatSlaveDriverEvent 
	printf("(>\")> Set event callback function\n");
	Result = setEcatSlaveCallbackFunctionMS(

		ECS_BOARD_ID,
		OnSlaveEventCallbackFunction,
		g_SlaveEventCallbackContextData,
		AL_EVENT_MAILBOX_START_REQ  | AL_EVENT_MAILBOX_STOP_REQ  |
		AL_EVENT_INPUTS_START_REQ   | AL_EVENT_OUTPUTS_START_REQ |
		AL_EVENT_INPUTS_STOP_REQ    | AL_EVENT_OUTPUTS_STOP_REQ  |
		AL_EVENT_INPUTS_STARTED     | AL_EVENT_OUTPUTS_STARTED   |
		AL_EVENT_INPUTS_STOPPED     | AL_EVENT_OUTPUTS_STOPPED   |
		AL_EVENT_INPUTS_TRANSMITTED | AL_EVENT_OUTPUTS_RECEIVED);
		
	if (ECAT_FAILED(Result)) {
		PRINT_MSG(MODULE_NAME": MainAppThreadFun: setEcatSlaveCallbackFunction returned error: 0x%X(%s)\n",
				Result, getEcatSlaveResultCodeDescription(Result));
		goto out_removeEcatSlaveCallbackFunction;
	}

    /* Start slave:
        - sets proper internal variables,
        - starts slave's internal threads(if present, otherwise KPAEcatSlaveMainLoopStep() should be called explicitly)
          that poll ESC and update mailboxes. */
	printf("(>\")> Start Slave\n");
	Result = startEcatSlaveMS(ECS_BOARD_ID);
	if (ECAT_FAILED(Result)) {
		PRINT_MSG(MODULE_NAME": MainAppThreadFun: startEcatSlave returned error: 0x%X(%s)\n",
			Result, getEcatSlaveResultCodeDescription(Result));
		goto out_removeEcatSlaveCallbackFunction;
	}


    /* check periodically whether shutdown is requested and if it is requested then
       stop Poll thread, stop App cycle, release hardware resourses and then completely exit. */

	/* Start cyclic operation of our custom application algorithm */
	while (!g_IsExitRequested) 
	{
		// If needed: call one-step update(slave state machine update, etc.)
		KPAEcatSlaveMainLoopStep(0);  // commented if slave stack has built-in thread to do it himself in a cyclic manner
		updateEcatSlaveTxInputsMS(ECS_BOARD_ID, SM_INPUTS_1_ID, g_pInputsBuf);
		
		if (counter > 10000)
		{
			printf("command byte: %02X,\tmotor_torque: %u\n", in.command_byte, in.motor_torque);
	
			counter = 0;
		}
		else
		{
			counter++;
		}
	}

out_stopEcatSlave:
    stopEcatSlaveMS(ECS_BOARD_ID);

out_removeEcatSlaveCallbackFunction:
    removeEcatSlaveCallbackFunctionMS(
                ECS_BOARD_ID,
                OnSlaveEventCallbackFunction,
                g_SlaveEventCallbackContextData);

out_releaseEcatSlave:
    releaseEcatSlaveMS(ECS_BOARD_ID);

    g_IsAppRunning = 0;
}

/*============================================================================
 * COMMON PART FOR SLAVE APPLICATIONS
/*==========================================================================*/

static void endme(int dummy) { g_IsExitRequested = 1; }

#ifdef __cplusplus
extern "C" {
#endif
/* See "ecatslave_osal_basicio.c" for sample implementation
    of the following functions */
extern int CustomECS_InitBasicio(
    UINT8 BoardNumber,
    const char* devname,
    unsigned long offset/*only 0 is supported*/,
    unsigned long size);
extern int CustomECS_ReleaseBasicio(UINT8 BoardNumber);
#ifdef __cplusplus
}
#endif

/*--------------------------------------------------------------------------*/
int main(int argc, char** argv) {

	int ret;
	ECAT_RESULT Result;
	
	//signal(SIGUSR1, get_sim_data);

	PRINT_MSG("=== Loading %s module (built on %s, %s) ===\n", MODULE_NAME, __DATE__, __TIME__);
	PRINT_MSG("g_EscPollPeriodMcs  = %ld\n", g_EscPollPeriodMcs);
	PRINT_MSG("g_HeartBeatPeriodMs = %ld\n", g_HeartBeatPeriodMs);
	PRINT_MSG("g_AppCyclePeriodMcs = %ld\n", g_AppCyclePeriodMcs);
	
	signal(SIGINT,  endme);

	/*==========================================================================*/

	g_pOutputsBuf = (UINT8 *)&in;
	g_pInputsBuf = (UINT8 *)&out;

	out.id = MEDULLA_BOOM_ID;

	/*========================================================================
	*  PreInitialization. Most probably you do not need to modify this.
	*  See ecatslave_osal_basicio.c for sample implementation.
	*/
	ret = CustomECS_InitBasicio(
					ECS_BOARD_ID,
					"/dev/kpaecs0",		// "/dev/kpaecs{ECS_BOARD_ID}"
					0,			// offset
					KPA_PCISLV_IOMEM_SIZE);

	if (ret != 0) {
		PRINT_MSG("CustomECS_InitBasicio() failed! ret = %d\n", ret);
	}

	/*========================================================================
	*  Start Slave application main cycle (runs permanently till 'Ctrl+C')
	*  See also 'g_AppCyclePeriodMcs' above.
	*/
	MainAppThreadFun((int)NULL);

	/*  Here Slave application main cycle exited. Release resources.
	*  See ecatslave_osal_basicio.c for sample implementation.
	*/
	CustomECS_ReleaseBasicio(ECS_BOARD_ID);

	/*========================================================================
	* The whole application exits.
	*/
	PRINT_MSG("%s exited.\n", MODULE_NAME);

	return 0;
}

