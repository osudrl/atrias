#ifndef SOEM_MEDULLA_H
#define SOEM_MEDULLA_H

#include <soem_master/soem_driver.h>
#include <soem_beckhoff_drivers/AtriasMsg.h>
#include <rtt/Port.hpp>
#include <bitset>

namespace soem_beckhoff_drivers {

    class SoemMedulla : public soem_master::SoemDriver {
        typedef struct {
            uint16_t motor;
            uint8_t command;
        } in_medulla;
        
        typedef struct {
            uint32_t enc32;
            uint16_t enc16[4];
            uint16_t timestep;

            uint8_t id;
            uint8_t status;

            uint8_t therm1;
            uint8_t therm2;
            uint8_t therm3;
        } out_medulla;

        in_medulla* in;
        out_medulla* out;
      
    public:
        SoemMedulla(ec_slavet* mem_loc);
        ~SoemMedulla(){};
     
        void update();
        uint32_t read();
        bool configure(){return true;}


    private:
        unsigned int m_size;
        AtriasMsg m_msg;
        std::bitset<8> m_bits;
        RTT::OutputPort<AtriasMsg> port_out;

};
 
}
#endif // SOEM_MEDULLA_H

