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
int32_t integrals[4][NUM_CHANNELS] = {0}; // Really 21 bits


extern "C" {
    int ped_subtract(struct SW_Data_Packet * data_packet);
    int integral(struct SW_Data_Packet * data_packet, int rel_start, int rel_end, int integral_num);
    void preprocess(struct SW_Data_Packet *input_data_packet, uint16_t *input_all_peds, char ** bounds, int32_t *output_integrals);
}

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
    return 0;
}

int integral(struct SW_Data_Packet * data_packet, int rel_start, int rel_end, int integral_num) {
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
            integrals[integral_num][i] = integral;
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
            integrals[integral_num][i] = integral;
        }
    }
    return 0;
}


void preprocess(
	    struct SW_Data_Packet *input_data_packet, // Read-Only Data Packet Struct
	    uint16_t *input_all_peds, // Read-Only Pedestals
        char ** bounds, // Read-Only Integral Bounds
	    int32_t *output_integrals       // Output Result (Integrals)
	    )
{
#pragma HLS INTERFACE m_axi port=input_data_packet bundle=aximm1
#pragma HLS INTERFACE m_axi port=input_all_peds bundle=aximm2
#pragma HLS INTERFACE m_axi port=bounds bundle=aximm3
#pragma HLS INTERFACE m_axi port=output_integrals bundle=aximm1

	ped_subtract(input_data_packet, input_all_peds);

    int s1 = atoi(bounds[0]);
    int e1 = atoi(bounds[1]);
    int s2 = atoi(bounds[2]);
    int e2 = atoi(bounds[3]);
    int s3 = atoi(bounds[4]);
    int e3 = atoi(bounds[5]);
    int s4 = atoi(bounds[6]);
    int e4 = atoi(bounds[7]);

    integral(input_data_packet, s1, e1, 0);
    integral(input_data_packet, s2, e2, 1);
    integral(input_data_packet, s3, e3, 2);
    integral(input_data_packet, s4, e4, 3);

    output_integrals = &integrals[0][0];
}
