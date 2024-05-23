#include <mram.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <defs.h>
#include <barrier.h>
#include <perfcounter.h>

#include "common.h"
#include "types.h"

__host uint32_t initialization = 0;
__host uint32_t huffman_decoding = 0;
__host uint32_t dequantization = 0;
__host uint32_t inverse_dct = 0;
__host uint32_t color_space_conversion = 0;

__mram uint32_t buffer[BUFFER_SIZE];
__mram short component01[64 * 5000]; // Y or R
__mram short component02[64 * 5000]; // Cb or G
__mram short component03[64 * 5000]; // Cr or B

__dma_aligned MCU current_mcus[NR_TASKLETS];

huffman_table huffman_DC_tables[4];
huffman_table huffman_AC_tables[4];
quantization_table quantization_tables[4];
jpeg_info metadata;

int byte_cursor = 0,  bit_cursor = 0;
int total_mcus, mcu_per_tasklet;
uint32_t accumulated_cycles = 0;

void init();
void load_data();

int read_bit();
int read_bits(int length);
void align();

void decode_huffman_data();
int decode_mcu_component(short *component, short *pre_dc_values, int component_ID);
byte get_next_symbol(huffman_table *table);

void dequantize(int tasklet_ID);

void idct(int tasklet_ID);
void idct_component(int tasklet_ID, int component_ID);

void convert_colorspace(int tasklet_ID);

BARRIER_INIT(init_barrier, NR_TASKLETS);

int main(){
    int tasklet_ID = me();
    if(tasklet_ID == 0){
        perfcounter_config(COUNT_CYCLES, true);
        init();
        load_data();
        initialization = perfcounter_get();
        accumulated_cycles += initialization;
        total_mcus = ((metadata.height + 7) / 8) * ((metadata.width + 7) / 8);
        mcu_per_tasklet = total_mcus / NR_TASKLETS;
        decode_huffman_data();
        huffman_decoding = perfcounter_get() - accumulated_cycles;
        accumulated_cycles += huffman_decoding;
    }
    barrier_wait(&init_barrier);
    dequantize(me());
    if(tasklet_ID == 0) dequantization = perfcounter_get() - accumulated_cycles;
    accumulated_cycles += dequantization;
    idct(me());
    if(tasklet_ID == 0) inverse_dct = perfcounter_get() - accumulated_cycles;
    accumulated_cycles += inverse_dct;
    convert_colorspace(me());
    if(tasklet_ID == 0) color_space_conversion = perfcounter_get() - accumulated_cycles;
    
    return 0;
}

void init(){
  for(int i=0; i<4; i++){
    huffman_DC_tables[i].set = 0;
    huffman_AC_tables[i].set = 0;
    quantization_tables[i].set = 0;
  }
}

void load_data(){
  int cursor = 0;

  for(int i=0; i<4; i++){
    if(buffer[cursor++] == FALSE) break;
    for(int j=0; j<17; j++) huffman_DC_tables[i].offsets[j] = buffer[cursor++];
    for(int j=0; j<162; j++) huffman_DC_tables[i].symbols[j] = buffer[cursor++];      
    for(int j=0; j<162; j++) huffman_DC_tables[i].codes[j] = buffer[cursor++];
    huffman_DC_tables[i].set = TRUE;
  }

  for(int i=0; i<4; i++){
    if(buffer[cursor++] == FALSE) break;
    for(int j=0; j<17; j++) huffman_AC_tables[i].offsets[j] = buffer[cursor++];
    for(int j=0; j<162; j++) huffman_AC_tables[i].symbols[j] = buffer[cursor++];      
    for(int j=0; j<162; j++) huffman_AC_tables[i].codes[j] = buffer[cursor++];
    huffman_AC_tables[i].set = TRUE;
  }

  for(int i=0; i<4; i++){
    if(buffer[cursor++] == FALSE) break;
    for(int j=0; j<64; j++) quantization_tables[i].table[j] = buffer[cursor++];
    quantization_tables[i].set = TRUE;
  }

  metadata.width = buffer[cursor++];
  metadata.height = buffer[cursor++];
  metadata.num_components = buffer[cursor++];
  for(int i=0; i<metadata.num_components; i++){
    metadata.color_components[i].huffman_DC_table_ID = buffer[cursor++];
    metadata.color_components[i].huffman_AC_table_ID = buffer[cursor++];
    metadata.color_components[i].quantization_table_ID = buffer[cursor++];
  }
  metadata.restart_interval = buffer[cursor++];
  byte_cursor = cursor;
}

void load_mcu(int tasklet_ID, int offset){
    // offset: n-th MCU
    mram_read(component01 + offset * 64, current_mcus[tasklet_ID].component[0], sizeof(current_mcus[tasklet_ID].component[0]));
    mram_read(component02 + offset * 64, current_mcus[tasklet_ID].component[1], sizeof(current_mcus[tasklet_ID].component[1]));
    mram_read(component03 + offset * 64, current_mcus[tasklet_ID].component[2], sizeof(current_mcus[tasklet_ID].component[2]));
}

void store_mcu(int tasklet_ID, int offset){
    // offset: n-th MCU
    mram_write(current_mcus[tasklet_ID].component[0], component01 + offset * 64, sizeof(current_mcus[tasklet_ID].component[0]));
    mram_write(current_mcus[tasklet_ID].component[1], component02 + offset * 64, sizeof(current_mcus[tasklet_ID].component[1]));
    mram_write(current_mcus[tasklet_ID].component[2], component03 + offset * 64, sizeof(current_mcus[tasklet_ID].component[2]));
}

int read_bit(){
    if(byte_cursor >= BUFFER_SIZE) return FALSE;
    int bit = (buffer[byte_cursor] >> (7 - bit_cursor)) & 1;
    bit_cursor += 1;
    if(bit_cursor == 8){
        bit_cursor = 0;
        byte_cursor += 1;
    }
    return bit;
}

int read_bits(int length){
    int bits = 0;
    for(uint i=0; i<length; i++){
        int bit = read_bit();
        if(bit == -1) return FALSE;
        bits = (bits << 1) | bit;
    }
    return bits;
}

void align(){
    if(byte_cursor >= BUFFER_SIZE) return;
    if(bit_cursor != 0){
        bit_cursor = 0;
        byte_cursor += 1;
    }
}

byte get_next_symbol(huffman_table *table){
    uint code = 0;

    for(int i=0; i<16; i++){
        int bit = read_bit();
        code = (code << 1) | bit;
        for(uint j=table->offsets[i]; j<table->offsets[i+1]; j++){
            if(code == table->codes[j]) return table->symbols[j];
        }
    }
    return FALSE;
}

int decode_mcu_component(short *component, short *pre_dc_values, int component_ID){
    huffman_table *huffman_DC_table = &huffman_DC_tables[metadata.color_components[component_ID].huffman_DC_table_ID];
    huffman_table *huffman_AC_table = &huffman_AC_tables[metadata.color_components[component_ID].huffman_AC_table_ID];
    
    byte length = get_next_symbol(huffman_DC_table);
    
    if(length == (byte)-1){
        printf("Error - Invalid DC value (length ==  %d)\n", length);
        return -1;
    }
    if(length > 11){
        printf("Error - DC coefficient length greater than 11\n");
        return -1;
    }
    
    int coefficient = read_bits(length);
    if(coefficient == -1){
        printf("Error - Invalid DC value\n");
        return -1;
    }
    if(length != 0 && coefficient < (1 << (length - 1)))
        coefficient -= (1 << length) - 1;
    component[0] = coefficient + (*pre_dc_values);
    (*pre_dc_values) = component[0];
    
    uint i = 1;
    while(i < 64){
        byte symbol = get_next_symbol(huffman_AC_table);
        
        if(symbol == (byte)-1){
            printf("Error - Invalid AC value\n");
            return -1;
        }
        if(symbol == 0x00){
            for(; i<64; i++) component[zigzag_map[i]] = 0;
            return 1;
        }
        
        byte num_zeros = symbol >> 4;
        byte coefficient_length = symbol & 0x0F;
        coefficient = 0;

        if(symbol == 0xF0) num_zeros = 16;
        if(i + num_zeros >= 64){
            printf("Error - Zero run-length exceeded MCU (i=%d, num_zeros=%d)\n", i, num_zeros);
            return -1;
        }

        for(uint j=0; j<num_zeros; j++, i++){
            component[zigzag_map[i]] = 0;
        }
        if(coefficient_length > 10){
            printf("Error - AC coefficient length greater than 10\n");
            return -1;
        }
        if(coefficient_length != 0){
            coefficient = read_bits(coefficient_length);
            if(coefficient == -1){
                printf("Error - Invalid AC value\n");
                return -1;
            }
            if(coefficient < (1 << (coefficient_length - 1))) coefficient -= (1 << coefficient_length) - 1;
            component[zigzag_map[i]] = coefficient;
            i += 1;
        }
    }
    return 1;
}


void decode_huffman_data(){
    for(int u=0; u<4; u++){
        if(huffman_DC_tables[u].set == 1){
            unsigned int code = 0;
            for(int i=0; i<16; i++){
                for(int j=huffman_DC_tables[u].offsets[i]; j<huffman_DC_tables[u].offsets[i+1]; j++){
                    huffman_DC_tables[u].codes[j] = code;
                    code += 1;
                }
                code <<= 1;
            }
        }
        if(huffman_AC_tables[u].set == 1){
            unsigned int code = 0;
            for(int i=0; i<16; i++){
                for(int j=huffman_AC_tables[u].offsets[i]; j<huffman_AC_tables[u].offsets[i+1]; j++){
                    huffman_AC_tables[u].codes[j] = code;
                    code += 1;
                }
                code <<= 1;
            }        
        }
    }

    short pre_dc_values[3] = {0, 0, 0};
    for(int i=0; i<total_mcus; i++){
        if(metadata.restart_interval != 0 && i % metadata.restart_interval == 0){
            pre_dc_values[0] = 0; pre_dc_values[1] = 0; pre_dc_values[2] = 0;
            align();
        }
        for(uint j=0; j<metadata.num_components; j++){
            if(decode_mcu_component(current_mcus[0].component[j], &pre_dc_values[j], j) == -1){
                printf("ERROR! (%dth MCU component)\n", i);
                return;
            }
        }
        store_mcu(0, i);
        
    }
}

void dequantize(int tasklet_ID){
    for(uint i=tasklet_ID*mcu_per_tasklet; i<tasklet_ID*mcu_per_tasklet+mcu_per_tasklet; i++){
        load_mcu(tasklet_ID, i);
        for(uint j=0; j<64; j++)
            for(uint k=0; k<metadata.num_components; k++)
                current_mcus[tasklet_ID].component[k][j] *= quantization_tables[metadata.color_components[k].quantization_table_ID].table[j];
        store_mcu(tasklet_ID, i);
    }
    printf("tasklet %d: MCU %d ~ %d (%d MCUs)\n", tasklet_ID, tasklet_ID*mcu_per_tasklet, tasklet_ID*mcu_per_tasklet+mcu_per_tasklet, mcu_per_tasklet);
}

void idct_component(int tasklet_ID, int component_ID){
    float result[64];
    float sum;
    for(uint i=0; i<8; i++){
        for(uint y=0; y<8; y++){
            sum = 0;
            for(uint v=0; v<8; v++){
                sum += current_mcus[tasklet_ID].component[component_ID][8 * v + i] * idct_map[8 * v + y];
            }
            result[y * 8 + i] = sum;
        }
    }

    for(uint i=0; i<8; i++){
        for(uint x=0; x<8; x++){
            sum = 0;
            for(uint u=0; u<8; u++){
                sum += result[i * 8 + u] * idct_map[8 * u + x];
            }
            current_mcus[tasklet_ID].component[component_ID][i * 8 + x] = (int)sum;
        }
    }
}

void idct(int tasklet_ID){
    for(uint i=tasklet_ID*mcu_per_tasklet; i<tasklet_ID*mcu_per_tasklet+mcu_per_tasklet; i++){
        load_mcu(tasklet_ID, i);
        for(uint j=0; j<metadata.num_components; j++)
            idct_component(tasklet_ID, j);
        store_mcu(tasklet_ID, i);
    }
}

void convert_colorspace(int tasklet_ID){
    short r, g, b;
    for(uint i=tasklet_ID*mcu_per_tasklet; i<tasklet_ID*mcu_per_tasklet+mcu_per_tasklet; i++){
        load_mcu(tasklet_ID, i);
        for(int j=0; j<64; j++){
            r = current_mcus[tasklet_ID].component[0][j] + 1.402f * current_mcus[tasklet_ID].component[2][j] + 128;
            g = current_mcus[tasklet_ID].component[0][j] - 0.344f * current_mcus[tasklet_ID].component[1][j] - 0.714 * current_mcus[tasklet_ID].component[2][j] + 128;
            b = current_mcus[tasklet_ID].component[0][j] + 1.772f * current_mcus[tasklet_ID].component[1][j] + 128;
            if(r < 0) r = 0;
            if(r > 255) r = 255;
            if(g < 0) g = 0;
            if(g > 255) g = 255;
            if(b < 0) b = 0;
            if(b > 255) b = 255;
            current_mcus[tasklet_ID].component[0][j] = r;
            current_mcus[tasklet_ID].component[1][j] = g;
            current_mcus[tasklet_ID].component[2][j] = b;
        }
        store_mcu(tasklet_ID, i);
    }
}