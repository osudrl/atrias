
/**
 * @file HelloWorld.cpp
 * This file demonstrates the Orocos Provides/Requires primitives with
 * a 'hello world' example.
 */

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

/**
 * Include this header in order to use methods.
 */
#include <rtt/OperationCaller.hpp>

#include <ocl/OCL.hpp>
#include <ocl/TaskBrowser.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;

/**
 * Exercise 5: Read Orocos Component Builder's Manual, Chap 2 sect 5
 *
 * In this exercise, we want to put our operations in a provided service
 * and our methods in a required service with the name 'Robot'.
 *
 * We will do this explicitly by creating a provides and a requires service
 * class.
 *   - Create a RobotService class which inherits from ServiceProvider and
 *     add the mymethod operation (cut & paste from Hello).
 *   - Create a Robot class which inherits from ServiceRequester and
 *     add the Method with the correct signature and name (cut & paste from World).
 *
 * In both cases, take care of proper constructor inintialisations.
 * Next inherit in Hello from the RobotService and in World from the Robot class.
 *
 * Extend World::configureHook() such that it connects its required service to
 * the provided service of Hello.
 *
 * Use sayIt and mymethod in the updateHook() of World and log the results.
 *
 * Finally test this all in the TaskBrowser, how did the interface of Hello and
 * World change compared to previous exercises ?
 *
 * Optional: rewrite this exercise, with the exact same functionality,
 * but with the least amount of code and classes.
 * What effects has this on re-usability ?
 */
namespace Example
{

	/**
     * Every component inherits from the 'TaskContext' class.  This base
     * class allow a user to add a primitive to the interface and contain
     * an ExecutionEngine which executes application code.
     */
    class Hello
        : public TaskContext
    {
    protected:
        /**
         * @name Operations
         * @{
         */
        /**
         * Returns a string.
         */
        string mymethod() {
            return "Hello World";
        }

        bool sayIt(string sentence, string& answer) {
            log(Info) <<"  Saying: Hello " << sentence << "!" <<endlog();
            if (sentence == "Orocos") {
                answer="Hello Friend!";
                return true;
            }
            return false;
        }
        /** @} */

    public:
        /**
         * This example sets the interface up in the Constructor
         * of the component.
         */
        Hello(std::string name)
            : TaskContext(name)
        {
            this->provides("robot")->addOperation("mymethod", &Hello::mymethod, this).doc("Returns a friendly word.");
            this->provides("robot")->addOperation("sayIt", &Hello::sayIt, this).doc("Returns a friendly answer.")
                    .arg("sentence", "That's what I'll say.")
                    .arg("answer", "That's the answer you'll get if you let me say the right thing.");
        }

    };

    /**
     * World is the component that shows how to use the interface
     * of the Hello component.
     */
    class World
		: public TaskContext
    {
    protected:
    	/**
    	 * This method object serves to store the
    	 * call to the Hello component.
    	 * It is best practice to have this object as
    	 * a member variable of your class.
    	 */
    	OperationCaller< string(void) > mymethod;

        OperationCaller< bool(string, string&) > sayIt;
    	/** @} */

    public:
    	World(std::string name)
			: TaskContext(name, PreOperational),
			  mymethod("mymethod"), sayIt("sayIt")
    	{
    	    this->requires("robot")->addOperationCaller(mymethod);
    	    this->requires("robot")->addOperationCaller(sayIt);
    	}

    	bool configureHook() {
    	    // Check for the service being ready here and if not, connect to the provided service
    	    // from peer 'Hello'.
    	    return requires("robot")->connectTo( getPeer("Hello")->provides("robot"));
    	}

    	void updateHook() {
    		log(Info) << "Receiving from 'Hello': " << mymethod() <<endlog();
            // Log the results of sayIt here too.
    	}
    };
}

using namespace Example;

int ORO_main(int argc, char** argv)
{
    Logger::In in("main()");

    // Set log level more verbose than default,
    // such that we can see output :
    if ( log().getLogLevel() < Logger::Info ) {
        log().setLogLevel( Logger::Info );
        log(Info) << argv[0] << " manually raises LogLevel to 'Info' (5). See also file 'orocos.log'."<<endlog();
    }

    log(Info) << "**** Creating the 'Hello' component ****" <<endlog();
    // Create the task:
    Hello hello("Hello");
    // Create the activity which runs the task's engine:
    // 1: Priority
    // 0.5: Period (2Hz)
    hello.setActivity( new Activity(1, 0.5 ) );
    log(Info) << "**** Starting the 'hello' component ****" <<endlog();
    // Start the component:
    hello.start();

    log(Info) << "**** Creating the 'World' component ****" <<endlog();
    World world("World");
    // Create the activity which runs the task's engine:
    // 1: Priority
    // 0.5: Period (2Hz)
    world.setActivity( new Activity(1, 0.5 ) );

    log(Info) << "**** Creating the 'Peer' connection ****" <<endlog();
    // This is a bidirectional connection.
    connectPeers(&world, &hello );

    log(Info) << "**** Starting the TaskBrowser       ****" <<endlog();
    // Switch to user-interactive mode.
    TaskBrowser browser( &hello );

    // Accept user commands from console.
    browser.loop();

    return 0;
}
