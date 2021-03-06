#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>

// char: 8 bit, short: 16 bit, long: 32 bit

#define NUM_CHANNELS 16
#define NUM_SAMPLES 250 // N
#define BUF_SIZE 8050 // 8 + N*16 + 1 words (16 bits / 2 bytes)
#define JSON_BUF_SIZE 10000 // Arbitrary for now, will make closer to what's needed later.

/*
 Mimics incoming data packet in C types.
 Note that data packets start with 0xA1FA and end with 0x0E6A.
 (Alpha: start, Omega: end) put these back in
*/
struct Data_Packet {

    unsigned short i2c_address; // 3 bits
    unsigned short conf_address; // 4 bits
    unsigned short bank; // 1 bit, A or B
    unsigned short fine_time; // 8 bits, sample number when trigger arrives
    unsigned long coarse_time; // 32 bits
    unsigned short trigger_number; // 16 bits
    unsigned short samples_after_trigger; // 8 bits
    unsigned short look_back_samples; // 8 bits
    unsigned short samples_to_be_read; // 8 bits
    unsigned short starting_sample_number; // 8 bits
    unsigned short number_of_missed_triggers; // 8 bits
    unsigned short state_machine_status; // 8 bits
    unsigned short samples[NUM_SAMPLES][NUM_CHANNELS]; // Variable size?
    // Looks like N samples for 16 channels (so 1 ASIC)
};

void add_to_json(char * json_ptr, char * field, unsigned long value, short is_first, short is_last){
    char * value_ptr;
    if (is_first) {
        strcpy(json_ptr, '{ "');
        json_ptr += 3;
    }
    strcpy(json_ptr, field);
    json_ptr += strlen(field);
    strcpy(json_ptr, '": ');
    json_ptr += 3;
    ltoa(value, value_ptr, 10);
    strcpy(json_ptr, value_ptr);
    json_ptr += strlen(value_ptr);
    if (!is_last) {
        strcpy(json_ptr, ', "');
        json_ptr += 3;
    }
    else {
        strcpy(json_ptr, '}');
    }
}

int struct_to_json(struct Data_Packet * struct_ptr, char * json_ptr){
    char * json_buff[JSON_BUF_SIZE];
    char * buff_ptr = json_buff;
    add_to_json(buff_ptr, "i2c_address", struct_ptr->i2c_address, 1, 0);
    add_to_json(buff_ptr, "conf_address", struct_ptr->conf_address, 0, 0);
    add_to_json(buff_ptr, "bank", struct_ptr->bank, 0, 0);
    add_to_json(buff_ptr, "fine_time", struct_ptr->fine_time, 0, 0);
    add_to_json(buff_ptr, "coarse_time", struct_ptr->coarse_time, 0, 0);
    add_to_json(buff_ptr, "trigger_number", struct_ptr->trigger_number, 0, 0);
    add_to_json(buff_ptr, "samples_after_trigger", struct_ptr->samples_after_trigger, 0, 0);
    add_to_json(buff_ptr, "look_back_samples", struct_ptr->look_back_samples, 0, 0);
    add_to_json(buff_ptr, "samples_to_be_read", struct_ptr->samples_to_be_read, 0, 0);
    add_to_json(buff_ptr, "starting_sample_number", struct_ptr->starting_sample_number, 0, 0);
    add_to_json(buff_ptr, "number_of_missed_triggers", struct_ptr->number_of_missed_triggers, 0, 0);
    add_to_json(buff_ptr, "state_machine_status", struct_ptr->state_machine_status, 0, 0);
    // Need to figure out best way to rep samples
    json_ptr = json_buff;
}

int json_to_struct(void * json_ptr, struct Data_Packet * struct_ptr){

}

int main(int argc, char *argv[]){
    
    if (argc != 2) {
        printf("Usage: %s <pathname>\n", argv[0]);
        return -1;
    }
    
    int fd = open(argv[1], 0, "r");
    if (fd == -1) {
        perror("open");
    }

    unsigned short buf[BUF_SIZE / 2];

    if (read(fd, buf, BUF_SIZE) == -1) {
        perror("read");
    }

    // First short should be 0xA1FA
    if (buf[0] != 0xa1fa) {
        printf("File must start with word 0xA1FA.\n");
        return -2;
    }

    struct Data_Packet data_packet;
    data_packet.i2c_address = 0b111 & (buf[1] >> 13);
    data_packet.conf_address = 0b1111 & (buf[1] >> 9);
    data_packet.bank = 0b1 & (buf[1] >> 8);
    data_packet.fine_time = 0xff & buf[1];
    data_packet.coarse_time = (((unsigned long) buf[2]) << 16) & (((unsigned long) buf[3]) & 0xffff);
    data_packet.trigger_number = buf[4];
    data_packet.samples_after_trigger = (buf[5] >> 8) & 0xff;
    data_packet.look_back_samples = buf[5] & 0xff;
    data_packet.samples_to_be_read = (buf[6] >> 8) & 0xff;
    data_packet.starting_sample_number = buf[6] & 0xff;
    data_packet.number_of_missed_triggers = (buf[7] >> 8) & 0xff;
    data_packet.state_machine_status = buf[7] & 0xff;
    int buf_idx = 8;
    for (int i = 0; i < NUM_SAMPLES; i++) {
        for (int j = 0; j < NUM_CHANNELS; j++) {
            data_packet.samples[i][j] = buf[buf_idx] & 0xfff;
            buf_idx++;
        }
    }

    // Last short should be 0x0E6A
    if (buf[buf_idx] != 0x0e6a) {
        printf("File must end with word 0x0E6A.\n");
        return -3;
    }

    return 0;
}
