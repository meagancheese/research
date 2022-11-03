#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

// char: 8 bit, short: 16 bit, long: 32 bit

#define NUM_CHANNELS 16
#define NUM_SAMPLES 256 // N
#define BUF_SIZE 4105 // 8 + N*16 + 1 words (16 bits / 2 bytes per word)
#define JSON_BUF_SIZE 10000 // Arbitrary for now, will make closer to what's needed later.

/*
 Mimics incoming data packet in C types.
*/
struct SW_Data_Packet {
    uint16_t alpha; // Start Constant 0xA1FA
    uint8_t i2c_address; // 3 bits
    uint8_t conf_address; // 4 bits
    uint8_t bank; // 1 bit, A or B
    uint8_t fine_time; // 8 bits, sample number when trigger arrives
    uint32_t coarse_time; // 32 bits
    uint16_t trigger_number; // 16 bits
    uint8_t samples_after_trigger; // 8 bits
    uint8_t look_back_samples; // 8 bits
    uint8_t samples_to_be_read; // 8 bits
    uint8_t starting_sample_number; // 8 bits
    uint8_t number_of_missed_triggers; // 8 bits
    uint8_t state_machine_status; // 8 bits
    uint16_t samples[NUM_SAMPLES][NUM_CHANNELS]; // Variable size?
    // Looks like N samples for 16 channels (so 1 ASIC)
    uint16_t omega; // End Constant 0x0E6A
};

struct SW_Data_Packet data_packet;

uint16_t peds_a[NUM_SAMPLES][NUM_CHANNELS];
uint16_t peds_b[NUM_SAMPLES][NUM_CHANNELS];


void add_to_json(int json_fd, char * field, uint32_t value, uint8_t is_first, uint8_t is_last){
    char value_ptr[10];
    if (is_first) {
        write(json_fd, "{ \"", 3);
    }
    write(json_fd, field, strlen(field));
    write(json_fd, "\": ", 3);
    sprintf(value_ptr, "%d", value);
    write(json_fd, value_ptr, strlen(value_ptr));
    if (!is_last) {
        write(json_fd, ", \"", 3);
    }
    else {
        write(json_fd, " }", 2);
    }
}

void add_samples_to_json(int json_fd){
    char value_ptr[10];
    write(json_fd, "samples", 7);
    write(json_fd, "\": [ ", 5);

    for (int i = 0; i < NUM_SAMPLES; i++) {
        for (int j = 0; j < NUM_CHANNELS; j++) {
            sprintf(value_ptr, "%d", data_packet.samples[i][j]);
            write(json_fd, value_ptr, strlen(value_ptr));
            write(json_fd, ", ", 2);
        }
    }
    lseek(json_fd, -2, SEEK_CUR);
    write(json_fd, " ], \"", 5);
}

int struct_to_json(int json_fd){
    add_to_json(json_fd, "alpha", data_packet.alpha, 1, 0);
    add_to_json(json_fd, "i2c_address", data_packet.i2c_address, 0, 0);
    add_to_json(json_fd, "conf_address", data_packet.conf_address, 0, 0);
    add_to_json(json_fd, "bank", data_packet.bank, 0, 0);
    add_to_json(json_fd, "fine_time", data_packet.fine_time, 0, 0);
    add_to_json(json_fd, "coarse_time", data_packet.coarse_time, 0, 0);
    add_to_json(json_fd, "trigger_number", data_packet.trigger_number, 0, 0);
    add_to_json(json_fd, "samples_after_trigger", data_packet.samples_after_trigger, 0, 0);
    add_to_json(json_fd, "look_back_samples", data_packet.look_back_samples, 0, 0);
    add_to_json(json_fd, "samples_to_be_read", data_packet.samples_to_be_read, 0, 0);
    add_to_json(json_fd, "starting_sample_number", data_packet.starting_sample_number, 0, 0);
    add_to_json(json_fd, "number_of_missed_triggers", data_packet.number_of_missed_triggers, 0, 0);
    add_to_json(json_fd, "state_machine_status", data_packet.state_machine_status, 0, 0);
    add_samples_to_json(json_fd);
    add_to_json(json_fd, "omega", data_packet.omega, 0, 1);
}

int data_packet_dat_to_struct(int fd){

    // Read data into a larger buffer and then strip 
    // all of the spaces because dat files are bit-space-delineated.

    // The times two is to account for the space-delineation
    int num_bits = BUF_SIZE * 16 * 2;
    uint8_t og_buf[num_bits];

    if (read(fd, og_buf, num_bits) == -1) {
        perror("read");
    }

    for (int i = 0; i < num_bits; i++) {
        og_buf[i] = og_buf[i] - 0x30; // 0 in ASCII
    }

    uint16_t buf[BUF_SIZE];
    uint8_t ms_half, ls_half;
    int base = 0;
    for (int i = 0; i < BUF_SIZE; i++) {
        base = i * 32;
        ms_half = (og_buf[base] << 7) | (og_buf[base + 2] << 6) | (og_buf[base + 4] << 5) | (og_buf[base + 6] << 4) | (og_buf[base + 8] << 3) | (og_buf[base + 10] << 2) | (og_buf[base + 12] << 1) | og_buf[base + 14];
        ls_half = (og_buf[base + 16] << 7) | (og_buf[base + 18] << 6) | (og_buf[base + 20] << 5) | (og_buf[base + 22] << 4) | (og_buf[base + 24] << 3) | (og_buf[base + 26] << 2) | (og_buf[base + 28] << 1) | og_buf[base + 30];
        buf[i] = (ms_half << 8) | ls_half;
    }

    // First short should be 0xA1FA
    if (buf[0] != 0xa1fa) {
        printf("File must start with word 0xA1FA.\n");
        return -2;
    }

    data_packet.alpha = buf[0];
    data_packet.i2c_address = 0b111 & (buf[1] >> 13);
    data_packet.conf_address = 0b1111 & (buf[1] >> 9);
    data_packet.bank = 0b1 & (buf[1] >> 8);
    data_packet.fine_time = 0xff & buf[1];
    data_packet.coarse_time = (((uint32_t) buf[2]) << 16) & (((uint32_t) buf[3]) & 0xffff);
    data_packet.trigger_number = buf[4];
    data_packet.samples_after_trigger = (buf[5] >> 8) & 0xff;
    data_packet.look_back_samples = buf[5] & 0xff;
    data_packet.samples_to_be_read = (buf[6] >> 8) & 0xff;
    data_packet.starting_sample_number = buf[6] & 0xff;
    data_packet.number_of_missed_triggers = (buf[7] >> 8) & 0xff;
    data_packet.state_machine_status = buf[7] & 0xff;

    int buf_idx = 8;
    int samples_idx = data_packet.starting_sample_number;
    for (int i = 0; i < data_packet.samples_to_be_read + 1; i++) {
        for (int j = 0; j < NUM_CHANNELS; j++) {
            data_packet.samples[samples_idx][j] = buf[buf_idx] & 0xfff;
            buf_idx++;
        }
        samples_idx++;
        if (samples_idx == NUM_SAMPLES) {
            samples_idx = 0;
        }
    }

    // Last short should be 0x0E6A
    if (buf[buf_idx] != 0x0e6a) {
        printf("File must end with word 0x0E6A.\n");
        return -3;
    }
    data_packet.omega = buf[buf_idx];

    return 0;
}

int peds_dat_to_arrays(int fd){
    FILE * fp = fdopen(fd, "r");
    if(fp == NULL) {
        perror("fdopen");
    }
    printf("After fdopen\n");
    fflush(stdout);

    uint16_t sample_num;
    uint16_t peds[NUM_CHANNELS];
    char line[100];
    for(int i = 0; i < NUM_SAMPLES; i++) {
        line[0] = '\0';
        //fscanf(fp, "%[^\n]%*c", line);
        printf("BEFORE FGETS\n");
        fflush(stdout);
        printf("LINE before fgets: %s\n", line);
        if(fgets(line, sizeof(line), fp) == NULL) {
            perror("fgets");
        }
        printf("LINE after fgets: %s", line);
        printf("After fgets\n");
        fflush(stdout);
        sscanf(line, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d", &sample_num, &peds[0], &peds[1], &peds[2], &peds[3], &peds[4], &peds[5], &peds[6], &peds[7], &peds[8], &peds[9], &peds[10], &peds[11], &peds[12], &peds[13], &peds[14], &peds[15]);
        printf("After sscanf\n");
        printf("Line after sscanf: %s", line);
        fflush(stdout);
        printf("Channel 1: %d\n", peds[0]);
        printf("Channel 2: %d\n", peds[1]);
        printf("Channel 3: %d\n", peds[2]);
        printf("Channel 4: %d\n", peds[3]);
        printf("Channel 5: %d\n", peds[4]);
        printf("Channel 6: %d\n", peds[5]);
        printf("Channel 7: %d\n", peds[6]);
        printf("Channel 8: %d\n", peds[7]);
        printf("Channel 9: %d\n", peds[8]);
        printf("Channel 10: %d\n", peds[9]);
        printf("Channel 11: %d\n", peds[10]);
        printf("Channel 12: %d\n", peds[11]);
        printf("Channel 13: %d\n", peds[12]);
        printf("Channel 14: %d\n", peds[13]);
        printf("Channel 15: %d\n", peds[14]);
        printf("Channel 16: %d\n", peds[15]);
        fflush(stdout);
        for(int j = 0; j < NUM_CHANNELS; j++) {
            peds_a[i][j] = peds[j];
        }
        printf("After assignment\n");
        fflush(stdout);
    }
    for(int i = 0; i < NUM_SAMPLES; i++) {
        line[0] = '\0';
        //fscanf(fp, "%[^\n]%*c", line);
        fgets(line, 100, fp);
        sscanf(line, "%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d%", &sample_num, &peds[0], &peds[1], &peds[2], &peds[3], &peds[4], &peds[5], &peds[6], &peds[7], &peds[8], &peds[9], &peds[10], &peds[11], &peds[12], &peds[13], &peds[14], &peds[15]);
        for(int j = 0; j < NUM_CHANNELS; j++) {
            peds_b[i][j] = peds[j];
        }
    }
    fclose(fp);
}

// Functions to make: ped_subtract, integrals, centroiding?, info mapping

int main(int argc, char *argv[]){
    
    //if (argc != 4) {
        // Need to add the interval limits to the count above (unless they're optional)
        // Should the Intervals be optional? Or else hard code a number of intervals?
        // Also maybe an optional name for a JSON debug file...
    //    printf("Usage: %s <EventStream.dat> <peds.dat> <trigger> <>\n", argv[0]);
    //    return -1;
    //}
    
    int data_packet_fd = open("EventStream.dat", 0, "r");
    if (data_packet_fd == -1) {
        perror("open");
    }

    data_packet_dat_to_struct(data_packet_fd);

    int json_fd = open("output.json", O_CREAT | O_RDWR);
    if (json_fd == -1) {
        perror("open");
    }

    struct_to_json(json_fd);

    int peds_fd = open("peds.dat", 0, "r");
    if (peds_fd == -1) {
        perror("open");
    }

    peds_dat_to_arrays(peds_fd);

    return 0;
}
