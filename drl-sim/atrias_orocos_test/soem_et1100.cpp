#include "soem_et1100.h"
#include <soem_master/soem_driver_factory.h>

using namespace RTT;

namespace soem_beckhoff_drivers
{

SoemET1100::SoemET1100(ec_slavet* mem_loc) :
    soem_master::SoemDriver(mem_loc), port_out("atrias_port_out")
{
    m_size = mem_loc->Ibits;
    m_service->doc(std::string("Services for Beckhoff ") + std::string(
            m_datap->name) + std::string(" (ATRIAS Medulla)"));
    m_service->addOperation("read", &SoemET1100::read, this,
            RTT::OwnThread).doc("Read in a value.");
    m_service->addConstant("size", m_size);
    //m_msg.values.resize(m_size);
    //port_out.setDataSample(m_msg);
    m_service->addPort("Port_out", port_out).doc("Data port to communicate full bitsets");
}

void SoemET1100::update()
{
    //m_msg.value = read();
    //log(Info) << "Read: " << m_msg.value << endlog();
    in = ((in_et1100t*) (m_datap->inputs));
    out = ((out_et1100t*) (m_datap->outputs));

    log(Info) << "  " << sizeof(out_et1100t)
              << "  " << out->enc32
              << "  " << (out->enc16)[0]
              << "  " << (out->enc16)[1]
              << "  " << (out->enc16)[2]
              << "  " << (out->enc16)[3]
              << "  " << out->timestep
              << "  " << out->id
              << "  " << out->status
              << endlog();
}

uint32_t SoemET1100::read(void) {
    return ((out_et1100t*) (m_datap->outputs))->enc32;
}


namespace
{
soem_master::SoemDriver* createSoemET1100(ec_slavet* mem_loc)
{
    return new SoemET1100(mem_loc);
}

// This is a.. hack? The EtherCAT slave cards used on ATRIAS are ET1100, but
// SOEM master sees them as "GeneralModules". Registering both for now.
const bool registered0 =
        soem_master::SoemDriverFactory::Instance().registerDriver("ET1100",
                createSoemET1100);
const bool registered1 =
        soem_master::SoemDriverFactory::Instance().registerDriver("GeneralModules",
                createSoemET1100);
}
}

