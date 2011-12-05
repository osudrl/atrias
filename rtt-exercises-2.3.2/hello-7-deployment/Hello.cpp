
/**
 * @file Hello.cpp
 * This file demonstratess the Orocos component loading and XML with
 * a 'hello world' example.
 */

/*
 * Exercise 7: This file contains a fully functional Orocos component.
 * It's your task to extend the hello.xml file in this directory such
 * that you can load, configure and start this component from the deployer
 * application.
 *
 * NOTE: See the bottom of this file for the first exercise.
 * NOTE: See the application.xml file for the rest of the exercise.
 * If you're stuck, you can look at solution.xml to see the expected
 * result.
 */

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Activity.hpp>
#include <rtt/Property.hpp>
#include <rtt/Attribute.hpp>
#include <rtt/OperationCaller.hpp>
#include <rtt/Port.hpp>

using namespace std;
using namespace RTT;

namespace Example
{

    /**
     * This component will be dynamically loaded in your application.
     */
    class Hello
        : public TaskContext
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
         * Attributes are aliased to class variables.
         */
        std::string attribute;
        /**
         * Constants are aliased, but can only be changed
         * from the component itself.
         */
        std::string constant;
        /** @} */

        /**
         * @name Input-Output ports
         * @{
         */
        /**
         * We publish our data through this OutputPort
         *
         */
        OutputPort<std::string> outport;
        /**
         * This InputPort buffers incoming data.
         */
        InputPort<std::string> inport;
        /** @} */

        /**
         * An operation we want to add to our interface.
         */
        std::string mymethod() {
            return "Hello World";
        }

        /**
         * This one is executed in our own thread.
         */
        bool sayWorld( std::string word) {
            cout <<"Saying Hello '"<<word<<"' in own thread." <<endl;
            if (word == "World")
                return true;
            return false;
        }

        void updateHook() {
            std::string value;
            // repeat the last read value
            while (inport.read(value) == NewData) {
                outport.write("last was:" + value);
            }
        }
    public:
        /**
         * This example sets the interface up in the Constructor
         * of the component.
         */
        Hello(std::string name)
            : TaskContext(name),
              // initial values
              property("Hello World"),
              attribute("Hello World"),
              constant("Hello World"),
              // Name, keep_last_written_value
              outport("outport",true),
              // Name, policy
              inport("inport",ConnPolicy::buffer(13,ConnPolicy::LOCK_FREE,true) )
        {
            // Now add it to the interface:
            this->addProperty("the_property", property).doc("Nice Description");
            this->addAttribute("the_attribute", attribute);
            this->addConstant("the_constant", constant);

            this->ports()->addPort( outport );
            this->ports()->addPort( inport );

            this->addOperation( "the_method", &Hello::mymethod, this, ClientThread ).doc("'the_method' Description");

            this->addOperation( "the_command", &Hello::sayWorld, this, OwnThread).doc("'the_command' Description").arg("the_arg", "Use 'World' as argument to make the command succeed.");
        }
    };
}

/**
 * See Orocos Deployment Component Manual (An OCL component).
 * Exercise: Include the necessary RTT header and add the C macro such that the
 * Example::Hello class becomes a loadable Orocos component. See World.cpp
 * for inspiration.
 */

