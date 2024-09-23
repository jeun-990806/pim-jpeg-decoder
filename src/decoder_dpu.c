#include <mram.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <defs.h>
#include <barrier.h>
#include <perfcounter.h>

#include "headers/common.h"

typedef struct {
    short component[3][64];
} MCU;

typedef struct {
    MCU mcus[4];
} block;

typedef struct {
    byte offsets[17];
    byte symbols[162];
    unsigned int codes[162];
    byte set;
} huffman_table;

typedef struct {
    uint table[64];
    byte set;
} quantization_table;

typedef struct {
    byte h_sampling_factor;
    byte v_sampling_factor;
    byte quantization_table_ID;
    byte huffman_DC_table_ID;
    byte huffman_AC_table_ID;
} color_component;

typedef struct {
    int mcu_height;
    int mcu_width;
    int mcu_height_real;
    int mcu_width_real;

    int num_components;
    color_component color_components[3];

    byte v_sampling_factor;
    byte h_sampling_factor;
} jpeg_info;

__host uint32_t initialization = 0;
__host uint32_t dequantization = 0;
__host uint32_t inverse_dct = 0;
__host uint32_t color_space_conversion = 0;

__mram uint32_t metadata_buffer[20 + 4 * 64];
__mram short mcus[64 * MAX_MCU_PER_DPU * 3];

__dma_aligned block current_blocks[NR_TASKLETS];
__dma_aligned uint32_t quantization_tables_cached[4 * 64];

jpeg_info metadata;

int total_blocks;
int block_per_tasklet;
uint32_t accumulated_cycles = 0;

void init();
void load_data();

void dequantize(int tasklet_ID);

void idct(int tasklet_ID);
void idct_component(int tasklet_ID, int block_position, int component_ID);

void convert_colorspace(int tasklet_ID);
void convert_colorspace_component(int tasklet_ID, int cbcr_index, int y_index, int v, int h);

BARRIER_INIT(init_barrier, NR_TASKLETS);

int main(){
    int tasklet_ID = me();
    
    if(tasklet_ID == 0){
        perfcounter_config(COUNT_CYCLES, true);
        load_data();
        initialization = perfcounter_get();
        accumulated_cycles += initialization;
        block_per_tasklet = total_blocks / NR_TASKLETS;
    }
    barrier_wait(&init_barrier);

    dequantize(me());
    if(tasklet_ID == 0) {
        dequantization = perfcounter_get() - accumulated_cycles;
        accumulated_cycles += dequantization;
    }
    
    idct(me());
    if(tasklet_ID == 0){ 
        inverse_dct = perfcounter_get() - accumulated_cycles;
        accumulated_cycles += inverse_dct;
    }

    convert_colorspace(me());
    if(tasklet_ID == 0) color_space_conversion = perfcounter_get() - accumulated_cycles;

    return 0;
}

void load_data(){
    metadata.mcu_height = metadata_buffer[0];
    metadata.mcu_width = metadata_buffer[1];
    metadata.mcu_height_real = metadata_buffer[2];
    metadata.mcu_width_real = metadata_buffer[3];
    metadata.num_components = metadata_buffer[4];
    metadata.v_sampling_factor = metadata_buffer[5];
    metadata.h_sampling_factor = metadata_buffer[6];
    for(uint i=0; i<metadata.num_components; i++){
        metadata.color_components[i].quantization_table_ID = metadata_buffer[i + 7];
    }
    for(uint i=0; i<metadata.num_components; i++){
        metadata.color_components[i].h_sampling_factor = metadata_buffer[i + metadata.num_components + 7];
    }
    for(uint i=0; i<metadata.num_components; i++){
        metadata.color_components[i].v_sampling_factor = metadata_buffer[i + (metadata.num_components * 2) + 7];
    }
    
    total_blocks = metadata_buffer[19] / 4;
    mram_read(metadata_buffer + 20, quantization_tables_cached, sizeof(uint32_t) * 4 * 64);
}

void load_block(int tasklet_ID, int offset){
    // offset: n-th block
    int range_start = (offset * 3) << 8;

    for(uint i=0; i<3; i++){
        mram_read(mcus + range_start + (i << 8), current_blocks[tasklet_ID].mcus[0].component[i], 128);
        mram_read(mcus + range_start + (i << 8) + 64, current_blocks[tasklet_ID].mcus[1].component[i], 128);
        mram_read(mcus + range_start + (i << 8) + 128, current_blocks[tasklet_ID].mcus[2].component[i], 128);
        mram_read(mcus + range_start + (i << 8) + 192, current_blocks[tasklet_ID].mcus[3].component[i], 128);
    }
}

void store_block(int tasklet_ID, int offset){
    // offset: n-th block
    int range_start = (offset * 3) << 8;

    for(uint i=0; i<3; i++){
        mram_write(current_blocks[tasklet_ID].mcus[0].component[i], mcus + range_start + (i << 8), 128);
        mram_write(current_blocks[tasklet_ID].mcus[1].component[i], mcus + range_start + (i << 8) + 64, 128);
        mram_write(current_blocks[tasklet_ID].mcus[2].component[i], mcus + range_start + (i << 8) + 128, 128);
        mram_write(current_blocks[tasklet_ID].mcus[3].component[i], mcus + range_start + (i << 8) + 192, 128);
    }
}

void dequantize(int tasklet_ID) {
    int start_block = tasklet_ID * block_per_tasklet;
    int end_block = start_block + block_per_tasklet;
    if (tasklet_ID == NR_TASKLETS - 1 && total_blocks % NR_TASKLETS != 0) {
        end_block += total_blocks % NR_TASKLETS;
    }

    for(int block_index=start_block; block_index<end_block; block_index++){
        load_block(tasklet_ID, block_index);
        for(uint i=0; i<metadata.num_components; i++){
            for(uint j=0; j<64; j++){
                current_blocks[tasklet_ID].mcus[0].component[i][j] *= quantization_tables_cached[metadata.color_components[i].quantization_table_ID * 64 + j];
                current_blocks[tasklet_ID].mcus[1].component[i][j] *= quantization_tables_cached[metadata.color_components[i].quantization_table_ID * 64 + j];
                current_blocks[tasklet_ID].mcus[2].component[i][j] *= quantization_tables_cached[metadata.color_components[i].quantization_table_ID * 64 + j];
                current_blocks[tasklet_ID].mcus[3].component[i][j] *= quantization_tables_cached[metadata.color_components[i].quantization_table_ID * 64 + j];
            }
        }
        store_block(tasklet_ID, block_index);
    }
}

void idct(int tasklet_ID) {
    int start_block = tasklet_ID * block_per_tasklet;
    int end_block = start_block + block_per_tasklet;
    if (tasklet_ID == NR_TASKLETS - 1 && total_blocks % NR_TASKLETS != 0) {
        end_block += total_blocks % NR_TASKLETS;
    }

    for (int block_index=start_block; block_index<end_block; block_index++) {
        load_block(tasklet_ID, block_index);
        
        idct_component(tasklet_ID, 0, 0);
        idct_component(tasklet_ID, 0, 1);
        idct_component(tasklet_ID, 0, 2);

        idct_component(tasklet_ID, 1, 0);
        if(metadata.h_sampling_factor != 2 && metadata.v_sampling_factor == 2){
            idct_component(tasklet_ID, 1, 1);
            idct_component(tasklet_ID, 1, 2);
        }

        idct_component(tasklet_ID, 2, 0);
        if(metadata.h_sampling_factor == 2 && metadata.v_sampling_factor != 2){
            idct_component(tasklet_ID, 2, 1);
            idct_component(tasklet_ID, 2, 2);
        }

        idct_component(tasklet_ID, 3, 0);
        if(metadata.h_sampling_factor != 2 && metadata.v_sampling_factor != 2){
            idct_component(tasklet_ID, 3, 1);
            idct_component(tasklet_ID, 3, 2);
        }

        store_block(tasklet_ID, block_index);
    }
}

// https://github.com/UBC-ECE-Sasha/PIM-JPEG/blob/master/src/dpu/dpu-jpeg-decode.c inverse_dct_component function 참고
void idct_component(int tasklet_ID, int block_position, int component_ID){
    int g0, g1, g2, g3, g4, g5, g6, g7;
    int f4, f5, f6, f7;
    int e2, e3, e5, e7, e8;
    int d2, d4, d5, d6, d8;
    int c0, c1, c2, c4, c5, c6, c8;
    int b0, b1, b2, b3, b4, b6;

    for(uint i=0; i<8; i++) {
        g0 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 0] * 181) >> 5;
        g1 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 4] * 181) >> 5;
        g2 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 2] * 59) >> 3;
        g3 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 6] * 49) >> 4;
        g4 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 5] * 71) >> 4;
        g5 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 1] * 251) >> 5;
        g6 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 7] * 25) >> 4;
        g7 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 3] * 213) >> 5;

        f4 = g4 - g7;
        f5 = g5 + g6;
        f6 = g5 - g6;
        f7 = g4 + g7;

        e2 = g2 - g3;
        e3 = g2 + g3;
        e5 = f5 - f7;
        e7 = f5 + f7;
        e8 = f4 + f6;

        d2 = (e2 * 181) >> 7;
        d4 = (f4 * 277) >> 8;
        d5 = (e5 * 181) >> 7;
        d6 = (f6 * 669) >> 8;
        d8 = (e8 * 49) >> 6;

        c0 = g0 + g1;
        c1 = g0 - g1;
        c2 = d2 - e3;
        c4 = d4 + d8;
        c5 = d5 + e7;
        c6 = d6 - d8;
        c8 = c5 - c6;

        b0 = c0 + e3;
        b1 = c1 + c2;
        b2 = c1 - c2;
        b3 = c0 - e3;
        b4 = c4 - c8;
        b6 = c6 - e7;

        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 0] = (b0 + e7) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 1] = (b1 + b6) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 2] = (b2 + c8) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 3] = (b3 + b4) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 4] = (b3 - b4) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 5] = (b2 - c8) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 6] = (b1 - b6) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][i * 8 + 7] = (b0 - e7) >> 4;
    }

    for (int i=0; i<8; i++) {
        g0 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][0 * 8 + i] * 181) >> 5;
        g1 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][4 * 8 + i] * 181) >> 5;
        g2 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][2 * 8 + i] * 59) >> 3;
        g3 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][6 * 8 + i] * 49) >> 4;
        g4 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][5 * 8 + i] * 71) >> 4;
        g5 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][1 * 8 + i] * 251) >> 5;
        g6 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][7 * 8 + i] * 25) >> 4;
        g7 = (current_blocks[tasklet_ID].mcus[block_position].component[component_ID][3 * 8 + i] * 213) >> 5;

        f4 = g4 - g7;
        f5 = g5 + g6;
        f6 = g5 - g6;
        f7 = g4 + g7;

        e2 = g2 - g3;
        e3 = g2 + g3;
        e5 = f5 - f7;
        e7 = f5 + f7;
        e8 = f4 + f6;

        d2 = (e2 * 181) >> 7;
        d4 = (f4 * 277) >> 8;
        d5 = (e5 * 181) >> 7;
        d6 = (f6 * 669) >> 8;
        d8 = (e8 * 49) >> 6;

        c0 = g0 + g1;
        c1 = g0 - g1;
        c2 = d2 - e3;
        c4 = d4 + d8;
        c5 = d5 + e7;
        c6 = d6 - d8;
        c8 = c5 - c6;

        b0 = c0 + e3;
        b1 = c1 + c2;
        b2 = c1 - c2;
        b3 = c0 - e3;
        b4 = c4 - c8;
        b6 = c6 - e7;

        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][0 * 8 + i] = (b0 + e7) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][1 * 8 + i] = (b1 + b6) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][2 * 8 + i] = (b2 + c8) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][3 * 8 + i] = (b3 + b4) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][4 * 8 + i] = (b3 - b4) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][5 * 8 + i] = (b2 - c8) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][6 * 8 + i] = (b1 - b6) >> 4;
        current_blocks[tasklet_ID].mcus[block_position].component[component_ID][7 * 8 + i] = (b0 - e7) >> 4;
    }
}

void convert_colorspace(int tasklet_ID){
    int start_block = tasklet_ID * block_per_tasklet;
    int end_block = start_block + block_per_tasklet;
    if(tasklet_ID == NR_TASKLETS-1 && total_blocks % NR_TASKLETS != 0)
        end_block += total_blocks % NR_TASKLETS;

    for(int block_index=start_block; block_index<end_block; block_index++) {
        load_block(tasklet_ID, block_index);

        if(metadata.v_sampling_factor == 1 && metadata.h_sampling_factor == 1){
            convert_colorspace_component(tasklet_ID, 0, 0, 0, 0);
            convert_colorspace_component(tasklet_ID, 1, 1, 0, 0);
            convert_colorspace_component(tasklet_ID, 2, 2, 0, 0);
            convert_colorspace_component(tasklet_ID, 3, 3, 0, 0);
        }
        if(metadata.v_sampling_factor == 2 && metadata.h_sampling_factor == 1){
            convert_colorspace_component(tasklet_ID, 0, 2, 1, 0);
            convert_colorspace_component(tasklet_ID, 0, 0, 0, 0);
            convert_colorspace_component(tasklet_ID, 1, 3, 1, 0);
            convert_colorspace_component(tasklet_ID, 1, 1, 0, 0);
        }
        if(metadata.v_sampling_factor == 1 && metadata.h_sampling_factor == 2){
            convert_colorspace_component(tasklet_ID, 0, 1, 0, 1);
            convert_colorspace_component(tasklet_ID, 0, 0, 0, 0);
            convert_colorspace_component(tasklet_ID, 2, 3, 0, 1);
            convert_colorspace_component(tasklet_ID, 2, 2, 0, 0);
        }
        if(metadata.v_sampling_factor == 2 && metadata.h_sampling_factor == 2){
            convert_colorspace_component(tasklet_ID, 0, 3, 1, 1);
            convert_colorspace_component(tasklet_ID, 0, 2, 1, 0);
            convert_colorspace_component(tasklet_ID, 0, 1, 0, 1);
            convert_colorspace_component(tasklet_ID, 0, 0, 0, 0);
        }

        store_block(tasklet_ID, block_index);
    }
}

void convert_colorspace_component(int tasklet_ID, int cbcr_index, int y_index, int v, int h){
    int scaling_factor = 22;    
    int pixel, cbcr_pixel;
    int y_val, cb_val, cr_val;
    int r, g, b;

    for(uint y=7; y<8; y--){
        for(uint x=7; x<8; x--){
            pixel = (y << 3) + x;
            cbcr_pixel = (((y / metadata.v_sampling_factor) + 4 * v) << 3) + ((x / metadata.h_sampling_factor) + 4 * h);
            
            y_val = current_blocks[tasklet_ID].mcus[y_index].component[0][pixel];
            cb_val = current_blocks[tasklet_ID].mcus[cbcr_index].component[1][cbcr_pixel];
            cr_val = current_blocks[tasklet_ID].mcus[cbcr_index].component[2][cbcr_pixel];
            
            r = y_val + ((5880414 * cr_val) >> scaling_factor) + 128;
            g = y_val - ((1442840 * cb_val) >> scaling_factor) - ((2994733 * cr_val) >> scaling_factor) + 128;
            b = y_val + ((7432306 * cb_val) >> scaling_factor) + 128;

            r = r < 0 ? 0 : (r > 255 ? 255 : r);
            g = g < 0 ? 0 : (g > 255 ? 255 : g);
            b = b < 0 ? 0 : (b > 255 ? 255 : b);
                            
            current_blocks[tasklet_ID].mcus[y_index].component[0][pixel] = r;
            current_blocks[tasklet_ID].mcus[y_index].component[1][pixel] = g;
            current_blocks[tasklet_ID].mcus[y_index].component[2][pixel] = b;
        }
    }

}