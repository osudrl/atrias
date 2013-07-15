/**
 * @file ATCDemo.cpp
 * @brief
 * @author Mikhail Jones
 */

#include "atc_demo/ATCDemo.hpp"

// The namespaces this controller resides in
namespace atrias {
namespace controller {

ATCDemo::ATCDemo(string name) :
    ATC(name),
    ascCommonToolkit(this, "ascCommonToolkit"),
    ascInterpolation(this, "ascInterpolation"),
    ascPDLmA(this, "ascPDLmA"),
    ascPDLmB(this, "ascPDLmB"),
    ascPDRmA(this, "ascPDRmA"),
    ascPDRmB(this, "ascPDRmB"),
    ascPDLh(this, "ascPDLh"),
    ascPDRh(this, "ascPDRh"),
    ascRateLimitLmA(this, "ascRateLimitLmA"),
    ascRateLimitLmB(this, "ascRateLimitLmB"),
    ascRateLimitRmA(this, "ascRateLimitRmA"),
    ascRateLimitRmB(this, "ascRateLimitRmB"),
    ascRateLimitLh(this, "ascRateLimitLh"),
    ascRateLimitRh(this, "ascRateLimitRh")
{
    // Startup is handled by the ATC class
    setStartupEnabled(true);

    // Set leg motor rate limit
    legRateLimit = 0.5;
    hipRateLimit = 0.5;
}

/**
 * @brief Top-level controller.
 * 
 * This is the main function for the top-level controller.
 * The ATC class automatically handles startup and shutdown,
 * if they are not disabled.
 */
void ATCDemo::controller() {
	// Implement your controller here. The robot state is in rs -- put the controller
	// output in co
}

ORO_CREATE_COMPONENT(ATCDemo)

}
}
