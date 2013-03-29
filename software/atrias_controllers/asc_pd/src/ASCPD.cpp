#include "asc_pd/ASCPD.hpp"

// Again, we need to put our code inside the appropriate namespaces.
namespace atrias {
namespace controller {

/* Breakdown of the next few lines:
 * We need to call our parent class's constructor,
 * then we can call the LogPort's constructor. The second parameter
 * is the name for this log port, which controls how it appears in the
 * bagfiles.
 */
ASCPD::ASCPD(AtriasController *parent, string name) :
        AtriasController(parent, name),
        log_out(this, "log")
{
        // Initialize our gains to something safe.
        P = 0.0;
        D = 0.0;
}

double ASCPD::operator()(double desPos, double curPos, double desVel, double curVel) {
        // Log our input data
        log_out.data.P          = P;
        log_out.data.D          = D;
        log_out.data.targetPos  = desPos;
        log_out.data.currentPos = curPos;
        log_out.data.targetVel  = desVel;
        log_out.data.currentVel = curVel;

        // Compute the actual output command
        // I'm just placing this right in the log data for convenience.
        log_out.data.output = P * (desPos - curPos) + D * (desVel - curVel);

        // Transmit the log data
        log_out.send();

        // Return our output command -- sending the log data does not change
        // this value.
        return log_out.data.output;
}

}
}

// vim: noexpandtab
