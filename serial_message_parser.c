/*
 * Serial Message Parser 
 *
 * Written by: 
 * David Kooi
 * dkooi@ucsc.edu
 * davidkooi.pythonanywhere.com
 *
 * 10/31/20
 * */


#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>

// Check where the MSB is placed.
const int isBigEnd=1;
#define SYS_BIGENDIAN() ((*(char*)&isBigEnd) == 0)

// Define the message structure generally
// to make it easier to change message format in the future
#define MAX_MESSAGE_BYTES    100

#define NUM_FLAG_UNITS       5
#define FLAG_BYTES           5
#define FLAG_TERM            0xFF

#define TOTAL_MAX_MESSAGE_BYTES MAX_MESSAGE_BYTES + FLAG_BYTES

#define HEADER_CONTENT_BYTES    4
#define HEADER_BYTES            HEADER_CONTENT_BYTES + 1 

#define XSUM_BYTES              4
#define FOOTER_BYTES            XSUM_BYTES + 1 

#define PAYLOAD_DATA_BYTES      4
#define PAYLOAD_UNIT_BYTES      PAYLOAD_DATA_BYTES + 1 

#define MAX_PAYLOAD_BYTES       MAX_MESSAGE_BYTES - HEADER_BYTES - FOOTER_BYTES


// The pre-processed message will be zero-padded to ensure 32bit alignment of the 
// header and payload data. This way the 32bit XOR checksum can be computed
// without worrying about intersecting with the footer. 
//
// The padding will be removed for the post-processed message.  
#define MISALIGNED_BYTES ((HEADER_BYTES + MAX_PAYLOAD_BYTES) % sizeof(uint32_t))
#define XSUM_PAD_BYTES   (sizeof(uint32_t) - MISALIGNED_BYTES)
#define NUM_XSUM_UNITS   ((HEADER_BYTES + MAX_PAYLOAD_BYTES + XSUM_PAD_BYTES) / sizeof(uint32_t)) 


typedef struct message_s{
    uint8_t* buffer;

    uint8_t* flag_buffer;
    uint8_t* header_buffer;
    uint8_t* payload_buffer;
    uint8_t* footer_buffer;

    int      id;
    uint8_t  payload_bytes;
}MessageData;

typedef enum state_def{
    STATE_RESET,
    STATE_START, 
    STATE_CHECK,
    STATE_COMPLETE
}State;

// Forward Declerations

/* Run the START state.*/
State exec_start_state(MessageData* pre_message_data);
/*Run the CHECK state */
State exec_check_state(MessageData* pre_message_data);
/* Run the COMPLETE state*/
State exec_complete_state(MessageData* final_message_data, MessageData* pre_message_data);

/* Allocate a buffer for a message containing a payload length of
 * payload_bytes*/
void allocate_message_data(MessageData* message_data, uint16_t payload_bytes);
/* Remove all references within message_data*/
void remove_message_references(MessageData* message_data);
/* Perform a blocking read of the serial port. Continue trying until all
 * requested bytes have been recieved. */
int pending_read(uint8_t* buffer, uint16_t bytes_requested);


/* Provided functions */
int  do_read(char *buffer, unsigned int count);
void process_message(char *buffer, unsigned int message_id);
int  get_message_size_from_message_id(int message_id);


int main(int argc, const char* argv[]){
    // Setup the interrupt signal to default action
    signal(SIGINT, SIG_DFL);

    State curr_state = STATE_RESET;
    State next_state = STATE_RESET;

    MessageData pre_message_data;   // Preprocessed and pre-verified message data
    MessageData final_message_data; // Processed and verified message data

    // Allocate a message for the maximum possible message size.
    // (With a 32 bit aligned XSUM field)
    //
    // Once we know the payload size then the essential data of pre_message_data 
    // will be copied into final_message_data. 
    //
    // The pointers in MessageData are also initialzied.  
    allocate_message_data(&pre_message_data, MAX_PAYLOAD_BYTES + XSUM_PAD_BYTES); 
    
    while(1){

        // Always update the current state
        curr_state = next_state;

        switch(curr_state){
            case STATE_RESET:  

                // Zero out to prepare for new message 
                memset(pre_message_data.buffer, 0, 
                        TOTAL_MAX_MESSAGE_BYTES + XSUM_PAD_BYTES);
                
                // Remove any references in final_message to make
                // sure we don't mess with any data we have finished working
                // with.
                remove_message_references(&final_message_data);

                next_state = STATE_START;

                break;
            case STATE_START:

                next_state = exec_start_state(&pre_message_data);

                break;
            case STATE_CHECK:

                next_state = exec_check_state(&pre_message_data);

                break;
            case STATE_COMPLETE:

                next_state = exec_complete_state(&final_message_data, &pre_message_data);

                process_message((char*)final_message_data.buffer, 
                               (unsigned int)final_message_data.id);

                break;
            default:

                next_state = STATE_RESET;
                break;
        }        
    }
   return 0; 
}

/* A blocking serial read function. 
 * Keep requesting bytes until the number of
 * bytes requested has been recieved.
 *
 * Returns 0 if succssful
 * Returns -1 if unsuccessful
 * */
int pending_read(uint8_t* buffer, uint16_t bytes_requested){

    uint16_t total_bytes_read = 0;
    uint16_t bytes_read       = 0;

    while(total_bytes_read < bytes_requested){
        bytes_read = do_read(buffer + total_bytes_read, 1);
        if(bytes_read == 1){
            total_bytes_read++;
        }else if(bytes_read == 0){
            continue;
        }else{
            // There was an error with the do_read function. 
            return -1;
        }
    }
    return 0;
}

/**
 * Execute the START state
 *
 * Returns the next state
 * */
State exec_start_state(MessageData* pre_message_data){

    uint8_t  flag_unit       = 0;
    uint8_t  num_flags       = 0;
    int      status          = 0;

    while(num_flags < NUM_FLAG_UNITS){

        status = pending_read(&flag_unit, 1);
        if(status != 0){ return STATE_RESET; }
 
        if(flag_unit != FLAG_TERM){
            // There is garbage
            // Restart the search
            num_flags = 0;
            continue;   
        }else{
            pre_message_data->flag_buffer[num_flags] = flag_unit;
            num_flags++;
        }
    }
    
    // Start flag verified, continue to checking the message
    return STATE_CHECK;
}

/**
 * Execute the CHECK state
 *
 * Returns the next state
 * */
State exec_check_state(MessageData* pre_message_data){

    uint8_t*  header_buffer; 
    uint8_t*  payload_buffer;
    uint8_t*  footer_buffer;
    uint32_t* xsum_buffer;

    uint8_t  i                  = 0;    
    int      status             = 0;
    
    int      msg_id             = 0;
    uint8_t  le_msg_id          = 0; // Little endian ID
    uint8_t  msg_size_bytes     = 0;

    uint8_t  payload_bytes      = 0;

    uint32_t xsum_calc          = 0;
    uint32_t xsum_msg           = 0;

    // Set specific pointers into the message buffer
    header_buffer  = pre_message_data->header_buffer; 
    payload_buffer = pre_message_data->payload_buffer;
    footer_buffer  = pre_message_data->footer_buffer; 
    xsum_buffer    = (uint32_t*)header_buffer; 

    // Get the message ID so we can retrieve the payload size 
    status = pending_read(header_buffer, HEADER_BYTES); 
    if(status != 0){ return STATE_RESET; }

    le_msg_id = header_buffer[0]; 
    // The message is in little-endian. 
    // If we need to, flip the bytes so we get the correct
    // value. 
    if(SYS_BIGENDIAN()){
        msg_id = (le_msg_id>>4) | (le_msg_id<<4);
    }else{
        msg_id = le_msg_id;
    }

    msg_size_bytes = get_message_size_from_message_id(msg_id); 

    // Compute the payload size 
    payload_bytes = msg_size_bytes - HEADER_BYTES - FOOTER_BYTES;

    // Set the data we have just computed 
    pre_message_data->id = msg_id;
    pre_message_data->payload_bytes = payload_bytes;
  
    // Read payload and footer data
    status = do_read(payload_buffer, payload_bytes);
    if(status != 0){ return STATE_RESET; }

    status = do_read(footer_buffer, FOOTER_BYTES); 
    if(status != 0){ return STATE_RESET; }

    // Calculate XOR Checksum
    xsum_msg = ((uint32_t*)footer_buffer)[0]; 

    xsum_calc = xsum_buffer[0];
    for(i = 1; i < NUM_XSUM_UNITS; i++){
        xsum_calc ^= xsum_buffer[i];
    }

    // Let's check if the message is okay.
    if(xsum_calc == xsum_msg){
        // Okay, continue to complete the processing
        return STATE_COMPLETE;
    }else{
        // Not okay, do a reset.
        return STATE_RESET;
    }
}

State exec_complete_state(MessageData* final_message_data, MessageData* pre_message_data){
   
    // Allocate the memory we will return
    // This also initialized the pointers of final_message_data to the right
    // place.
    allocate_message_data(final_message_data, pre_message_data->payload_bytes);

    // Set the data fields of the final message 
    final_message_data->payload_bytes = pre_message_data->payload_bytes; 
    final_message_data->id = pre_message_data->id;


    // Assuming that handling the endianness of the message data is
    // not this message parser's responsibility.
    
    // Copy flag
    memcpy(final_message_data->flag_buffer, 
          pre_message_data->flag_buffer, 
          FLAG_BYTES);

    // Copy Header
    memcpy(final_message_data->header_buffer, 
           pre_message_data->header_buffer, 
           HEADER_BYTES); 

    // Copy payload
    memcpy(final_message_data->payload_buffer, 
           pre_message_data->payload_buffer, 
           pre_message_data->payload_bytes); 

    // Copy footer
    memcpy(final_message_data->footer_buffer,
           pre_message_data->footer_buffer,
           FOOTER_BYTES);

   return STATE_RESET;
}

/**
 * Allocate a message containing payload_bytes of data within the payload
 * field. 
 *
 * Given the number of payload_bytes the whole message structure can be
 * created.
 *
 * Returns void.
 * */
void allocate_message_data(MessageData* message_data, uint16_t payload_bytes){

    uint16_t total_final_size = 0;

    total_final_size = FLAG_BYTES + HEADER_BYTES + payload_bytes + FOOTER_BYTES; 

    message_data->buffer = (uint8_t*)malloc(total_final_size);

    message_data->flag_buffer    = message_data->buffer;
    message_data->header_buffer  = message_data->flag_buffer    + FLAG_BYTES;
    message_data->payload_buffer = message_data->header_buffer  + HEADER_BYTES;  
    message_data->footer_buffer  = message_data->payload_buffer + payload_bytes; 
}

/**
 * Set all pointers to NULL. 
 * Used after the final message is sent out. 
 * */
void remove_message_references(MessageData* message_data){
   message_data->buffer         = NULL;
   message_data->flag_buffer    = NULL; 
   message_data->payload_buffer = NULL;
   message_data->footer_buffer  = NULL;
}

/**
* Attempts to read up to count number of bytes from the serial
* port into the buffer.
*
* Returns the number of bytes actually do_read.
*/
int do_read(char *buffer, unsigned int count){

}

/**
* This function should only be called when a complete message
* has been parsed. The complete message should be at the start
* of the input buffer.
*/
void process_message(char *buffer, unsigned int message_id){

}


/**
* Returns the size (in bytes) of the message with the specified
* message_id. The message size includes the separator bytes
* (0xFE) and the checksum but does not include the start flag.
*/
int get_message_size_from_message_id(int message_id){

}
