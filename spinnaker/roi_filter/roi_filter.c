
//! imports
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "spin1_api.h"
#include "common-typedefs.h"
#include <data_specification.h>
#include <recording.h>
#include <simulation.h>
#include <debug.h>
#include <circular_buffer.h>

#define X_MASK(x) (x>>1)&0x1FF
#define Y_MASK(y) (y>>12)&0xFF

//! control value, which says how many timer ticks to run for before exiting
static uint32_t simulation_ticks = 0;
static uint32_t infinite_run = 0;
static uint32_t time = 0;
static uint32_t received_count = 0;
static uint32_t events_processed = 0;

//! timer period
static uint32_t timer_period;
static uint32_t log_counter = 0;

//! parameters for this c code
uint32_t example_key;
static uint32_t row_number;
static uint32_t number_of_cols;
unsigned char *LUT;

//! transmission key
static uint32_t i_has_key;
static uint32_t base_key;

//! human readable definitions of each region in SDRAM
typedef enum regions_e {
    SYSTEM_REGION,
    TRANSMISSION_DATA_REGION,
    CONFIG_REGION
} regions_e;

//! values for the priority for each callback
typedef enum callback_priorities {
    MC_PACKET = -1, MCPL_PACKET = 0, SDP_DMA = 1, TIMER = 3
} callback_priorities;

//! human readable definitions of each element in the transmission region
typedef enum transmission_region_elements {
    HAS_KEY = 0, MY_KEY = 1
} transmission_region_elements;

//! human readable definitions of each element in the config region
typedef enum config_region_elements {
    ROW_NUMBER = 0, NUMB_COLS = 1
} config_region_elements;


void update_LUT(int x, int y, int r) {

    memset(LUT, 0, sizeof(unsigned char) * number_of_cols);

    if((int)row_number > y - r && (int)row_number < y + r) {
        int x_start = x-r;
        if(x_start < 0)
            x_start = 0;
        int x_end = x + r;
        if(x_end > (int)(number_of_cols-1))
            x_end = (int)(number_of_cols-1);
        for(int i = x_start; i < x_end; i++)
            LUT[i] = 1;
    }

}

//! \brief callback for when packet has payload (agg)
//! \param[in] key: the key received
//! \param[in] payload: the payload received
void receive_data_payload(uint key, uint payload)
{
    //here we receive the region of interest and need to set the values
    //static int i = 0;
//    if(i++ % 30 == 0) {
//        log_debug("Received new ROI %d %d %d", X_MASK(key), Y_MASK(key), payload);
//    }
    update_LUT(X_MASK(key), Y_MASK(key), payload);

}


//! \brief callback when packet with no payload is received (retina)
//! \param[in] key: the key received
//! \param[in] payload: unused. is set to 0
void receive_data_no_payload(uint key, uint payload) {

    use(payload);
    received_count++;
    //here we need to filter the events based on the LUT

    if(!LUT[X_MASK(key)])
        return;

    events_processed++;
    //send on the data (masking out the flag bit
    example_key = key;
    while (!spin1_send_mc_packet(key | base_key, 0, NO_PAYLOAD)) {
            spin1_delay_us(1);
    }

}

//! \brief callback for when resuming
void resume_callback() {
    time = UINT32_MAX;
}

//! \brief timer tick callback
//! \param[in] ticks the number of tiemr tick callbacks (not accurate)
//! \param[in] b: unknown
void update(uint ticks, uint b) {
    use(b);
    use(ticks);

    time++;

    //log_debug("on tick %d of %d", time, simulation_ticks);

    // check that the run time hasn't already elapsed and thus needs to be
    // killed
    if ((infinite_run != TRUE) && (time >= simulation_ticks)) {
        log_debug("Simulation complete.\n");

        // falls into the pause resume mode of operating
        simulation_handle_pause_resume(resume_callback);

        return;

    }

    if(time*timer_period  >= log_counter) {
        log_counter += 1000000;
        log_debug("Received = %d | Processed = %d | Period 1s | 0x%08x | 0x%08x",
            received_count, events_processed, example_key, base_key);
        events_processed = 0;
        received_count = 0;
    }

}

//! \brief reads the transmission keys data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_transmission_keys(address_t address){
    i_has_key = address[HAS_KEY];
    base_key = address[MY_KEY];
    return true;
}

//! \brief reads the column data region data items
//! \param[in] address: dsg address in sdram memory space
//! \return bool which is successful if read correctly, false otherwise
bool read_config(address_t address){
    row_number = address[ROW_NUMBER];
    number_of_cols = address[NUMB_COLS];
    return true;
}


//! \brief main initisation method
//! \param[in] timer_period. the time set for the timer
//! \return bool true if successful, false otherwise
static bool initialize(uint32_t *timer_period) {
    log_info("Initialise: started\n");

    // Get the address this core's DTCM data starts at from SRAM
    address_t address = data_specification_get_data_address();

    // Read the header
    if (!data_specification_read_header(address)) {
        log_error("failed to read the data spec header");
        return false;
    }

    // Get the timing details and set up the simulation interface
    if (!simulation_initialise(
            data_specification_get_region(SYSTEM_REGION, address),
            APPLICATION_NAME_HASH, timer_period, &simulation_ticks,
            &infinite_run, SDP_DMA, SDP_DMA)) {
        return false;
    }

    // get key data
    if (!read_transmission_keys(data_specification_get_region(
            TRANSMISSION_DATA_REGION, address))){
        return false;
    }


    // get config data
    if (!read_config(data_specification_get_region(CONFIG_REGION, address))){
        return false;
    }

    // initialise my input_buffer for receiving packets
    log_info("build buffer");
    log_info("number of columns: %d", number_of_cols);
    LUT = spin1_malloc(number_of_cols * sizeof(unsigned char));
    if (LUT == 0){
        log_info("Could not allocate LUT");
        return false;
    }
    update_LUT(152, 120, 30);

    return true;
}


//! \brief main entrance method
void c_main() {
    log_info("starting roi filter\n");

    // initialise the model
    if (!initialize(&timer_period)) {
        log_error("failed to init");
        rt_error(RTE_SWERR);
    }

    // set timer tick value to configured value
    log_info("setting timer to execute every %d microseconds", timer_period);
    spin1_set_timer_tick(timer_period);

    // register callbacks
    spin1_callback_on(MCPL_PACKET_RECEIVED, receive_data_payload, MCPL_PACKET);
    spin1_callback_on(MC_PACKET_RECEIVED, receive_data_no_payload, MC_PACKET);
    spin1_callback_on(TIMER_TICK, update, TIMER);
    //spin1_callback_on(USER_EVENT, user_callback, USER);

    // start execution
    log_info("Starting\n");

    // Start the time at "-1" so that the first tick will be 0
    time = UINT32_MAX;

    simulation_run();
}
