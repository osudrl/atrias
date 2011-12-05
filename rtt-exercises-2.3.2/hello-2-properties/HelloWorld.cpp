
/**
 * @file HelloWorld.cpp
 * This file demonstrates the Orocos 'Property' and 'Attribute' primitives with
 * a 'hello world' example.
 */

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>

/**
 * Include this header in order to use properties or attributes.
 */
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>

#include <ocl/OCL.hpp>
#include <ocl/TaskBrowser.hpp>

using namespace std;
using namespace RTT;
using namespace Orocos;

/**
 * Exercise 2: Read Orocos Component Builder's Manual, Chap 2 sect. 3.6
 *
 * First, compile and run this application and use 'the_property' and 'the_attribute':
 * Change and print their values.
 *
 * Next save the properties of this component to a hello.xml file:
 * You will need to install the 'marshalling' service using the TaskBrowser
 * at runtime:
 * ** TaskBrowser: type '.provides marshalling'
 *
 * (note: To make this permanent for your component:
 *    In C++ you need to #include <rtt/marsh/Marshalling.hpp> 
 *    and add to the constructor: this->getProvider<Marshalling>("marshalling");
 *
 *    In the CMakeLists.txt: add rtt-marshalling to the list of components to look for in
 *    find_package(Orocos-RTT REQUIRED) macro.
 *    See : http://www.orocos.org/wiki/orocos/toolchain/getting-started/cmake-and-building
 *    and use that syntax in the CMakeLists.txt file to link 'HelloWorld' with marshalling.
 * endnote)
 *
 * When marshalling is loaded:
 * In order to find out how to write the property to a file using 'marshalling',
 * type 'marshalling' (without quotes) to see the interface of the marshalling
 * task object.
 *
 * Next Open and modify the XML file and read it back in using the marshalling object.
 *
 * For the optional exercises, reead Chap 2, sect 6.1
 * Optional : read the property file from configureHook() and log it's value. You need
 * to make the modifications detailed above in the note.
 * Optional : write the property file in cleanupHook().
 */
namespace Example
{

    /**
     * Every component inherits from the 'TaskContext' class.  This base
     * class allow a user to add a primitive to the interface and contain
     * an ExecutionEngine which executes application code.
     */
    class Hello
        : public RTT::TaskContext
    {
    protected:
        /**
         * @name Name-Value parameters
         * @{
         */
        /**
         * Properties take a name, a value and a description
         * and are suitable for XML.
         */
        std::string property;
        /**
         * Attributes are C++ variables exported to the interface.
         */
        double attribute;
        /**
         * Constants are not changeable by an outsider, only by us.
         */
        std::string constant;
        /** @} */

    public:
        /**
         * This example sets the interface up in the Constructor
         * of the component.
         */
        Hello(std::string name)
            : TaskContext(name),
              // Name, description, value
              property("Hello World")
        {
            // Now add it to the interface:
            this->addProperty("the_property", property).doc("This property can contain any friendly string.");

            this->addAttribute("the_attribute", attribute);
            this->addConstant("the_constant", constant);
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
    hello.setActivity( new Activity(1, 0.5) );

    log(Info) << "**** Starting the 'Hello' component ****" <<endlog();
    // Start the component:
    hello.start();

    log(Info) << "**** Using the 'Hello' component    ****" <<endlog();

    // Do some 'client' calls :
    log(Info) << "**** Reading a Property:            ****" <<endlog();
    Property<std::string> p = hello.provides()->getProperty("the_property");
    if ( p.ready() )
        log(Info) << "     "<<p.getName() << " = " << p.value() <<endlog();
    else
        log(Error) << "Property 'the_property' was not found !" <<endlog();

    log(Info) << "**** Starting the TaskBrowser       ****" <<endlog();
    // Switch to user-interactive mode.
    TaskBrowser browser( &hello );

    // Accept user commands from console.
    browser.loop();

    return 0;
}
