
/**
 * @file World.cpp
 * This file demonstratess the Orocos component loading and XML with
 * a 'hello world' example.
 *
 * Exercise: This file (World.cpp) serves to demonstrate a fully
 * functional, loadable Orocos component. Extend the Hello.cpp file
 * in order to make the Example::Hello component loadable as well.
 *
 */

#include <rtt/os/main.h>

#include <rtt/Logger.hpp>
#include <rtt/TaskContext.hpp>

#include <rtt/Property.hpp>
#include <rtt/Port.hpp>

using namespace std;
using namespace RTT;

namespace Example
{

    /**
     * This component will be dynamically loaded in your application.
     */
    class World
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
        /** @} */

        /**
         * @name Input-Output ports
         * @{
         */
        /**
         * OutputPorts produce data
         */
        OutputPort<std::string> outport;
        /**
         * InputPorts consume data.
         */
        InputPort<std::string> inport;
        /** @} */

        OperationCaller<string(void)> mymethod;
        OperationCaller<bool(string)> sayWorld;

        int counter;
    public:
        /**
         * This example sets the interface up in the Constructor
         * of the component.
         */
        World(std::string name)
            : TaskContext(name, PreOperational),
              // Name, description, value
              property("Example"),
              // Name, initial value
              outport("outport"),
              // Name, buffer size, initial value
              inport("inport"),
              mymethod("the_method"),
              sayWorld("the_command"),
              counter(0)
        {
            // Now add it to the interface:
            this->addProperty("world_property", property);
            this->addAttribute("counter",counter);

            this->ports()->addPort( outport );
            this->ports()->addEventPort( inport );

            this->requires()->addOperationCaller(mymethod);
            this->requires()->addOperationCaller(sayWorld);
        }

        bool configureHook() {
            Logger::In in("World");
            // Lookup the Hello component.
            TaskContext* peer = this->getPeer("Hello");
            if ( !peer ) {
                log(Error) << "Could not find Hello component!"<<endlog();
                return false;
            }
            return true;
        }

        /**
         * This function may be called for multiple samples in inport
         */
        void updateHook() {
            std::string data;
            // for each sample we consume, produce 2 new ones.
            while ( inport.read(data) == NewData ) {
                std::stringstream ss;
                ss << ++counter;
                outport.write( ss.str() );
                ss << ", " << (counter + 1);
                outport.write( ss.str() );
            }
        }
    };
}

#include <rtt/Component.hpp>
ORO_CREATE_COMPONENT( Example::World );
