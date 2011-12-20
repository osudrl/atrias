#ifndef SOEM_ET1100_H
#define SOEM_ET1100_H

#include <soem_master/soem_driver.h>
#include <soem_beckhoff_drivers/EncoderMsg.h>
#include <rtt/Port.hpp>
#include <bitset>

namespace soem_beckhoff_drivers{

  class SoemET1100 : public soem_master::SoemDriver
  {
    
    typedef struct PACKED
    {
      uint8 bits;
    } out_et1100t;
    
  public:
    SoemET1100(ec_slavet* mem_loc);
    ~SoemET1100(){};
   
    bool isOn( unsigned int bit = 0) const;
    bool isOff( unsigned int bit = 0) const;
    bool readBit( unsigned int bit = 0) const;
    
    void update();
    uint32_t read();
    bool configure(){return true;}


  private:
    unsigned int m_size;
    EncoderMsg m_msg;
    std::bitset<8> m_bits;
    RTT::OutputPort<EncoderMsg> m_port;


};
 
}
#endif // SOEM_ET1100_H

