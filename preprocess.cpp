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

int16_t ped_sub_results[NUM_SAMPLES][NUM_CHANNELS] = {0}; // Really 13 bits
//int32_t integrals[4][NUM_CHANNELS] = {0}; // Really 21 bits

// FOR DEBUGGING
char value_ptr[10];


int ped_subtract(struct SW_Data_Packet * data_packet, uint16_t *all_peds) {
    int ped_sample_idx = data_packet->starting_sample_number;
    for (int i = 0; i < data_packet->samples_to_be_read + 1; i++) {
        for (int j = 0; j < NUM_CHANNELS; j++) {
            //ped_sub_results[i][j] = data_packet->samples[i][j] - all_peds[data_packet->bank][ped_sample_idx][j];
            ped_sub_results[i][j] = data_packet->samples[i][j] - *(((all_peds + (data_packet->bank)*(NUM_SAMPLES*NUM_CHANNELS)) + ped_sample_idx*NUM_CHANNELS) + j);
        }
        ped_sample_idx += 1;
        if (ped_sample_idx == NUM_SAMPLES) {
            ped_sample_idx = 0;
        }
    }
    printf("all_peds[1][3][5] from kernel: \n");
    sprintf(value_ptr, "%d", *(((all_peds + (1)*(NUM_SAMPLES*NUM_CHANNELS)) + 3*NUM_CHANNELS) + 5));
    printf(value_ptr);
    printf("\n");
    return 0;
}

int integral(struct SW_Data_Packet * data_packet, int rel_start, int rel_end, int integral_num, int32_t * integrals) {
    int start = data_packet->trigger_number + rel_start - data_packet->starting_sample_number;
    if (start < 0) {
        start = start + data_packet->samples_to_be_read;
    }
    int end = data_packet->trigger_number + rel_end - data_packet->starting_sample_number;
    if (end >= data_packet->samples_to_be_read) {
        end = end - data_packet->samples_to_be_read;
    }
    int32_t integral;
    if (end >= start) {
        for (int i = 0; i < NUM_CHANNELS; i++) {
            integral = 0;
            for (int j = start; j <= end; j++) {
                integral = integral + ped_sub_results[j][i];
            }
            integrals[integral_num*NUM_CHANNELS+i] = integral;
        }
    }
    else {
        for (int i = 0; i < NUM_CHANNELS; i++) {
            integral = 0;
            for (int j = start; j < NUM_SAMPLES; j++) {
                integral = integral + ped_sub_results[j][i];
            }
            for (int k = 0; k <= end; k ++) {
                integral = integral + ped_sub_results[k][i];
            }
            integrals[integral_num*NUM_CHANNELS+i] = integral;
        }
    }
    return 0;
}

// THESE METHODS ARE STRICTLY FOR DEBUGGING
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

void add_samples_to_json(int json_fd, struct SW_Data_Packet * data_packet){
    char value_ptr[10];
    write(json_fd, "samples", 7);
    write(json_fd, "\": [ ", 5);

    for (int i = 0; i < NUM_SAMPLES; i++) {
        for (int j = 0; j < NUM_CHANNELS; j++) {
            sprintf(value_ptr, "%d", data_packet->samples[i][j]);
            write(json_fd, value_ptr, strlen(value_ptr));
            write(json_fd, ", ", 2);
        }
    }
    lseek(json_fd, -2, SEEK_CUR);
    write(json_fd, " ], \"", 5);
}

int struct_to_json(int json_fd, struct SW_Data_Packet * data_packet){
    add_to_json(json_fd, "alpha", data_packet->alpha, 1, 0);
    add_to_json(json_fd, "i2c_address", data_packet->i2c_address, 0, 0);
    add_to_json(json_fd, "conf_address", data_packet->conf_address, 0, 0);
    add_to_json(json_fd, "bank", data_packet->bank, 0, 0);
    add_to_json(json_fd, "fine_time", data_packet->fine_time, 0, 0);
    add_to_json(json_fd, "coarse_time", data_packet->coarse_time, 0, 0);
    add_to_json(json_fd, "trigger_number", data_packet->trigger_number, 0, 0);
    add_to_json(json_fd, "samples_after_trigger", data_packet->samples_after_trigger, 0, 0);
    add_to_json(json_fd, "look_back_samples", data_packet->look_back_samples, 0, 0);
    add_to_json(json_fd, "samples_to_be_read", data_packet->samples_to_be_read, 0, 0);
    add_to_json(json_fd, "starting_sample_number", data_packet->starting_sample_number, 0, 0);
    add_to_json(json_fd, "number_of_missed_triggers", data_packet->number_of_missed_triggers, 0, 0);
    add_to_json(json_fd, "state_machine_status", data_packet->state_machine_status, 0, 0);
    add_samples_to_json(json_fd, data_packet);
    add_to_json(json_fd, "omega", data_packet->omega, 0, 1);
    return 0;
}

extern "C" {
    void preprocess(
	        struct SW_Data_Packet * input_data_packet, // Read-Only Data Packet Struct
	        uint16_t *input_all_peds, // Read-Only Pedestals
            int * bounds, // Read-Only Integral Bounds
	        int32_t *output_integrals       // Output Result (Integrals)
	        )
    {
#pragma HLS INTERFACE m_axi port=input_data_packet bundle=aximm1
#pragma HLS INTERFACE m_axi port=input_all_peds bundle=aximm2
#pragma HLS INTERFACE m_axi port=bounds bundle=aximm3
#pragma HLS INTERFACE m_axi port=output_integrals bundle=aximm1

        int json_fd = open("kernel_packet.json", O_CREAT | O_RDWR, 0666);
        if (json_fd == -1) {
            perror("open");
        }

        struct_to_json(json_fd, input_data_packet);

	    ped_subtract(input_data_packet, input_all_peds);

        integral(input_data_packet, bounds[0], bounds[1], 0, output_integrals);
        integral(input_data_packet, bounds[2], bounds[3], 1, output_integrals);
        integral(input_data_packet, bounds[4], bounds[5], 2, output_integrals);
        integral(input_data_packet, bounds[6], bounds[7], 3, output_integrals);

        printf("BOUNDS: \n");
        sprintf(value_ptr, "%d", bounds[0]);
        printf(value_ptr);
        printf("\n");
        sprintf(value_ptr, "%d", bounds[1]);
        printf(value_ptr);
        printf("\n");

        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < NUM_CHANNELS; j++) {
                sprintf(value_ptr, "%d", output_integrals[i*NUM_CHANNELS+j]);
                printf(value_ptr);
                printf("\n");
            }
        }
    }
}