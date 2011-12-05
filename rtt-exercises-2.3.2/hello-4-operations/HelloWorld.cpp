
/**
 * @file HelloWorld.cpp
 * This file demonstrates the Orocos 'Operation/OperationCaller' primitive with
 * a 'hello world' example.
 */

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

/**
 * Include this header in order to call component operations.
 */
#include <rtt/OperationCaller.hpp>

#include <ocl/OCL.hpp>
#include <ocl/TaskBrowser.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;

/**
 * Exercise 4: Read Orocos Component Builder's Manual, Chap 2 sect 3.5
 *
 * First, compile and run this application and use 'the_method'.
 * Configure and start the World component ('World.start()') and see
 * how it uses the_method. Fix any bugs :-)
 *
 * Next, add to Hello a second method 'bool sayIt(string sentence, string& answer)'
 * which uses log(Info) to display a sentence in the thread of the Hello component.
 * When sentence is "Orocos", the answer is "Hello Friend!" and true is returned. Otherwise,
 * false is returned and answer remains untouched.
 * Add this function to the operation interface of this class and document it
 * and its arguments. Test sending and collecting arguments with the TaskBrowser.
 *
 * Next do the same in C++. Send sayIt to Hello in updateHook of the World component, but only
 * check for the results in the next iteration of updateHook.
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
        /** @} */

    public:
        /**
         * This example sets the interface up in the Constructor
         * of the component.
         */
        Hello(std::string name)
            : TaskContext(name)
        {
            this->addOperation("the_method", &Hello::mymethod, this).doc("Returns a friendly word.");
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
    	OperationCaller< string(void) > hello_method;

    	/** @} */

    public:
    	World(std::string name)
			: TaskContext(name, PreOperational)
    	{
    	}

    	bool configureHook() {

    		// Lookup the Hello component.
    	    TaskContext* peer = this->getPeer("Hello");
    	    if ( !peer ) {
    	    	log(Error) << "Could not find Hello component!"<<endlog();
    	    	return false;
    	    }

    	    // It is best practice to lookup methods of peers in
    	    // your configureHook.
    	    hello_method = peer->getOperation("themethod");
    	    if ( !hello_method.ready() ) {
    	    	log(Error) << "Could not find Hello.the_method Operation!"<<endlog();
    	    	return false;
    	    }
    	    return true;
    	}

    	void updateHook() {
    		log(Info) << "Receiving from 'Hello': " << hello_method() <<endlog();
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
    // hello.engine(): is being executed.
    hello.setActivity( new Activity(1, 0.5 ) );
    log(Info) << "**** Starting the 'hello' component ****" <<endlog();
    // Start the component:
    hello.start();

    log(Info) << "**** Creating the 'World' component ****" <<endlog();
    World world("World");
    // Create the activity which runs the task's engine:
    // 1: Priority
    // 0.5: Period (2Hz)
    // world.engine(): is being executed.
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
