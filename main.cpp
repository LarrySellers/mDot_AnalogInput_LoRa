#include "mbed.h"
#include "dot_util.h"
#include "RadioEvent.h"

#define CHANNEL_PLAN CP_US915

//
//LoRa connection parameter variables
//
static std::string network_name = "MultiTech";
static std::string network_passphrase = "hondacr125";
static uint8_t frequency_sub_band = 1;
static bool public_network = false;
static uint8_t join_delay = 1;
static uint8_t ack = 0;
static bool adr = true;

// deepsleep consumes slightly less current than sleep
// in sleep mode, IO state is maintained, RAM is retained, and application will resume after waking up
// in deepsleep mode, IOs float, RAM is lost, and application will start from beginning after waking up
// if deep_sleep == true, device will enter deepsleep mode
//static bool deep_sleep = true;

mDot* dot = NULL;
lora::ChannelPlan* plan = NULL;

Serial pc(USBTX, USBRX); 
 
// 
// Initialize a pins to perform analog input and digital output fucntions
//
AnalogIn   ain(A0);
DigitalOut dout(XBEE_RSSI);
 
int main(void)
{
    /*
    * union for converting from 16- bit to 2 8-bit values
    */
    union convert32 {
        int16_t f_s;        // convert from signed 16 bit int
        uint16_t f_u;       // convert from unsigned 16 bit int
        uint8_t t_u[2];     // convert to 8 bit unsigned array
    } convertS;
        
    // Custom event handler for automatically displaying RX data
    RadioEvent events;

    pc.baud(115200);

#if defined(TARGET_XDOT_L151CC)
    i2c.frequency(400000);
#endif

    mts::MTSLog::setLogLevel(mts::MTSLog::TRACE_LEVEL);
    
    #if CHANNEL_PLAN == CP_US915
    plan = new lora::ChannelPlan_US915();
#elif CHANNEL_PLAN == CP_AU915
    plan = new lora::ChannelPlan_AU915();
#elif CHANNEL_PLAN == CP_EU868
    plan = new lora::ChannelPlan_EU868();
#elif CHANNEL_PLAN == CP_KR920
    plan = new lora::ChannelPlan_KR920();
#elif CHANNEL_PLAN == CP_AS923
    plan = new lora::ChannelPlan_AS923();
#elif CHANNEL_PLAN == CP_AS923_JAPAN
    plan = new lora::ChannelPlan_AS923_Japan();
#elif CHANNEL_PLAN == CP_IN865
    plan = new lora::ChannelPlan_IN865();
#endif
    assert(plan);

    dot = mDot::getInstance(plan);
    assert(dot);

    // attach the custom events handler
    dot->setEvents(&events);

    if (!dot->getStandbyFlag()) {
        logInfo("mbed-os library version: %d", MBED_LIBRARY_VERSION);

        // start from a well-known state
        logInfo("defaulting Dot configuration");
        dot->resetConfig();
        dot->resetNetworkSession();

        // make sure library logging is turned on
        dot->setLogLevel(mts::MTSLog::INFO_LEVEL);

        // update configuration if necessary
        // in AUTO_OTA mode the session is automatically saved, so saveNetworkSession and restoreNetworkSession are not needed
        if (dot->getJoinMode() != mDot::AUTO_OTA) {
            logInfo("changing network join mode to AUTO_OTA");
            if (dot->setJoinMode(mDot::AUTO_OTA) != mDot::MDOT_OK) {
                logError("failed to set network join mode to AUTO_OTA");
            }
        }
        // in OTA and AUTO_OTA join modes, the credentials can be passed to the library as a name and passphrase or an ID and KEY
        // only one method or the other should be used!
        // network ID = crc64(network name)
        // network KEY = cmac(network passphrase)
        update_ota_config_name_phrase(network_name, network_passphrase, frequency_sub_band, public_network, ack);
        //update_ota_config_id_key(network_id, network_key, frequency_sub_band, public_network, ack);
    
        // configure network link checks
        // network link checks are a good alternative to requiring the gateway to ACK every packet and should allow a single gateway to handle more Dots
        // check the link every count packets
        // declare the Dot disconnected after threshold failed link checks
        // for count = 3 and threshold = 5, the Dot will ask for a link check response every 5 packets and will consider the connection lost if it fails to receive 3 responses in a row
        update_network_link_check_config(3, 5);

        // enable or disable Adaptive Data Rate
        dot->setAdr(adr);

        // Configure the join delay
        dot->setJoinDelay(join_delay);
    
        // save changes to configuration
        logInfo("saving configuration");
        if (!dot->saveConfig()) {
            logError("failed to save configuration");
        }
    
        // display configuration
        display_config();
    }
    
    
    
    while (true) {
        
        uint8_t index = 5;
        uint16_t rawVoltage;
        float rawRead;
        std::vector<uint8_t> tx_data;

        // join network if not joined
        if (!dot->getNetworkJoinStatus()) {
            join_network();
        }
             
        /*
        // test the voltage on the initialized analog pin
        //  and if greater than 0.3 * VCC set the digital pin
        //  to a logic 1 otherwise a logic 0
        if(ain > 0.5f) {
            dout = 1;
        } else {
            dout = 0;
        }
         */
                        
        // print the percentage and 16 bit normalized values
        rawVoltage = ain.read_u16();
        rawRead = ain.read();
        convertS.f_u = rawVoltage;
        //convertS.f_float = rawRead;
        
        tx_data.push_back(index);
        tx_data.push_back(convertS.t_u[1]);
        tx_data.push_back(convertS.t_u[0]);
        
        //print results        
        printf("Raw Float Reading: %f \n\r", rawRead);
        printf("Raw Analog Reading(u_16): %lu\n\r", rawVoltage);
        printf("t_u[0]: %lu\n\r", convertS.t_u[0]);
        printf("t_u[1]: %lu\n\r", convertS.t_u[1]);
        
        send_data(tx_data);
        
        printf("\n\r\n\r***** All done *****\n\r\n\r");
        
        //wait(60.0f);
        
        // get data and send it to the gateway        
        //tx_data.push_back((rawVoltage >> 8) & 0xFF);
        //tx_data.push_back(rawVoltage & 0xFF);
        
        //logInfo("Payload: %lu [0x%04X]", rawVoltage, rawVoltage);
        
        /*
        printf("Payload Array: ");
        
        for(int i = 0; i < tx_data.size(); i++)
        {
            printf("%lu, ", tx_data[i]);
        }
        */
        
        

        // ONLY ONE of the three functions below should be uncommented depending on the desired wakeup method
        //sleep_wake_rtc_only(deep_sleep);
        //sleep_wake_interrupt_only(deep_sleep);
        //sleep_wake_rtc_or_interrupt(deep_sleep);
    }
}