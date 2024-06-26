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
__host uint32_t dequantization = 0;
__host uint32_t inverse_dct = 0;
__host uint32_t color_space_conversion = 0;

__mram uint32_t quantization_tables[4 * 64];
__mram uint32_t metadata_buffer[16];
__mram uint32_t mcu_num[4];
/* 
 * metadata_buffer[0] = mcuHeight
 * metadata_buffer[1] = mcuWidth
 * metadata_buffer[2] = mcuHeightReal
 * metadata_buffer[3] = mcuWidthReal
 * metadata_buffer[4] = numComponent
 * metadata_buffer[5] = verticalSamplingFactor
 * metadata_buffer[6] = horizontalSamplingFactor
 * metadata_buffer[7-9] = quantization table ID of component01-03
 * metadata_buffer[10-12] = vertical sampling factors of component01-03
 * metadata_buffer[13-15] = horizontal sampling factors of component01-03
 */
__mram short component01[64 * MAX_MCU_PER_DPU]; // Y or R
__mram short component02[64 * MAX_MCU_PER_DPU]; // Cb or G
__mram short component03[64 * MAX_MCU_PER_DPU]; // Cr or B

__dma_aligned MCU current_mcus[NR_TASKLETS];
__dma_aligned uint32_t quantization_tables_cached[4 * 64];
__dma_aligned MCU pre_cbcr_mcu[NR_TASKLETS];

jpeg_info metadata;

int mcu_per_tasklet;
uint32_t accumulated_cycles = 0;

void init();
void load_data();

void dequantize(int tasklet_ID);

void idct(int tasklet_ID);
void idct_component(int tasklet_ID, int component_ID);

void convert_colorspace(int tasklet_ID);
void convert_colorspace_shift_and_add(int tasklet_ID);

BARRIER_INIT(init_barrier, NR_TASKLETS);

int main(){
    int tasklet_ID = me();
    
    if(tasklet_ID == 0){
        perfcounter_config(COUNT_CYCLES, true);
        load_data();
        initialization = perfcounter_get();
        accumulated_cycles += initialization;
        mcu_per_tasklet = mcu_num[0] / NR_TASKLETS;
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
    
    convert_colorspace_shift_and_add(me());
    if(tasklet_ID == 0) color_space_conversion = perfcounter_get() - accumulated_cycles;

    return 0;
}

void load_data(){
    metadata.mcu_height = metadata_buffer[0];
    metadata.mcu_width = metadata_buffer[1];
    metadata.mcu_height_real = metadata_buffer[2];
    metadata.mcu_width_real = metadata_buffer[3];
    metadata.num_components = metadata_buffer[4];
    metadata.vertical_sampling_factor = metadata_buffer[5];
    metadata.horizontal_sampling_factor = metadata_buffer[6];
    for(uint i=0; i<metadata.num_components; i++){
        metadata.color_components[i].quantization_table_ID = metadata_buffer[i + 7];
    }
    for(uint i=0; i<metadata.num_components; i++){
        metadata.color_components[i].horizontal_sampling_factor = metadata_buffer[i + metadata.num_components + 7];
    }
    for(uint i=0; i<metadata.num_components; i++){
        metadata.color_components[i].vertical_sampling_factor = metadata_buffer[i + (metadata.num_components * 2) + 7];
    }
    mram_read(quantization_tables, quantization_tables_cached, sizeof(uint32_t) * 4 * 64);
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

void dequantize(int tasklet_ID) {
    int start_mcu = tasklet_ID * mcu_per_tasklet;
    int end_mcu = start_mcu + mcu_per_tasklet;
    if(tasklet_ID == NR_TASKLETS - 1 && mcu_num[0] % NR_TASKLETS != 0)
        end_mcu += mcu_num[0] % NR_TASKLETS;

    for(int mcu_index=start_mcu; mcu_index<end_mcu; mcu_index++) {
        int row = mcu_index / metadata.mcu_width_real;
        int column = mcu_index % metadata.mcu_width_real;
        
        for (uint i=0; i<metadata.num_components; i++) {
            for (uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++) {
                for (uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++) {
                    int current_mcu_index = (row + v) * metadata.mcu_width_real + (column + h);
                    load_mcu(tasklet_ID, current_mcu_index);
                    for (uint j=0; j<64; j++) {
                        current_mcus[tasklet_ID].component[i][j] *= quantization_tables_cached[metadata.color_components[i].quantization_table_ID * 64 + j];
                    }
                    store_mcu(tasklet_ID, current_mcu_index);
                }
            }
        }
    }
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

void idct_component_shift_and_add(int tasklet_ID, int component_ID){
    int result[64];
    int sum, scaling_factor = 17;
    for(uint i=0; i<8; i++){
        for(uint y=0; y<8; y++){
            sum = 0;
            for(uint v=0; v<8; v++){
                sum += (current_mcus[tasklet_ID].component[component_ID][8 * v + i] * idct_map_int[8 * v + y]) >> scaling_factor;
            }
            result[y * 8 + i] = sum;
        }
    }

    for(uint i=0; i<8; i++){
        for(uint x=0; x<8; x++){
            sum = 0;
            for(uint u=0; u<8; u++){
                sum += (result[i * 8 + u] * idct_map_int[8 * u + x]) >> scaling_factor;
            }
            current_mcus[tasklet_ID].component[component_ID][i * 8 + x] = sum;
        }
    }
}

void idct(int tasklet_ID){
    int start_mcu = tasklet_ID * mcu_per_tasklet;
    int end_mcu = start_mcu + mcu_per_tasklet;
    if(tasklet_ID == NR_TASKLETS - 1 && mcu_num[0] % NR_TASKLETS != 0)
        end_mcu += mcu_num[0] % NR_TASKLETS;

    for(int mcu_index=start_mcu; mcu_index<end_mcu; mcu_index++) {
        int row = mcu_index / metadata.mcu_width_real;
        int column = mcu_index % metadata.mcu_width_real;
        
        for (uint i=0; i<metadata.num_components; i++) {
            for (uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++) {
                for (uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++) {
                    int current_mcu_index = (row + v) * metadata.mcu_width_real + (column + h);
                    load_mcu(tasklet_ID, current_mcu_index);
                    idct_component_shift_and_add(tasklet_ID, i);
                    store_mcu(tasklet_ID, current_mcu_index);
                }
            }
        }
    }
}

void convert_colorspace_shift_and_add(int tasklet_ID){
    int start_mcu = tasklet_ID * mcu_per_tasklet;
    int end_mcu = start_mcu + mcu_per_tasklet;
    if(tasklet_ID == NR_TASKLETS-1 && mcu_num[0] % NR_TASKLETS != 0)
        end_mcu += mcu_num[0] % NR_TASKLETS;
    int start_row = start_mcu / metadata.mcu_width_real;
    int start_column = start_mcu % metadata.mcu_width_real;
    int end_row = end_mcu / metadata.mcu_width_real;
    int end_column = (end_mcu-1) % metadata.mcu_width_real;

    int pixel, cbcr_pixel_row, cbcr_pixel_column, cbcr_pixel;
    int r, g, b, tmp_cb, tmp_cr, scaling_factor = 17;

    for(uint i=start_mcu; i<end_mcu; i++){
        load_mcu(tasklet_ID, i);
        for(int j=0; j<64; j++){
            tmp_cr = (183762 * current_mcus[tasklet_ID].component[2][j]) >> scaling_factor;
            r = current_mcus[tasklet_ID].component[0][j] + tmp_cr + 128;
            tmp_cb = (45088 * current_mcus[tasklet_ID].component[1][j]) >> scaling_factor;
            tmp_cr = (93585 * current_mcus[tasklet_ID].component[2][j]) >> scaling_factor;
            g = current_mcus[tasklet_ID].component[0][j] - tmp_cb - tmp_cr + 128;
            tmp_cb = (232259 * current_mcus[tasklet_ID].component[1][j]) >> scaling_factor;
            b = current_mcus[tasklet_ID].component[0][j] + tmp_cb + 128;
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

void convert_colorspace(int tasklet_ID){
    int start_mcu = tasklet_ID * mcu_per_tasklet;
    int end_mcu = start_mcu + mcu_per_tasklet;
    if(tasklet_ID == NR_TASKLETS-1 && mcu_num[0] % NR_TASKLETS != 0)
        end_mcu += mcu_num[0] % NR_TASKLETS;
    int start_row = start_mcu / metadata.mcu_width_real;
    int start_column = start_mcu % metadata.mcu_width_real;
    int end_row = end_mcu / metadata.mcu_width_real;
    int end_column = (end_mcu-1) % metadata.mcu_width_real;

    int pixel, cbcr_pixel_row, cbcr_pixel_column, cbcr_pixel;
    int r, g, b;
    /*
    for(uint x=start_column; x<metadata.mcu_width_real; x+=metadata.horizontal_sampling_factor){
        for(uint v=metadata.vertical_sampling_factor-1; v<metadata.vertical_sampling_factor; v--){
            for(uint h=metadata.horizontal_sampling_factor-1; h<metadata.horizontal_sampling_factor; h--){
                load_mcu(tasklet_ID, (start_row + v) * metadata.mcu_width_real + (x + h));
                for(uint mcu_y=7; mcu_y<8; mcu_y--){
                    for(uint mcu_x=7; mcu_x<8; mcu_x--){
                        pixel = mcu_y * 8 + mcu_x;
                        cbcr_pixel_row = mcu_y / metadata.vertical_sampling_factor + 4 * v;
                        cbcr_pixel_column = mcu_x / metadata.horizontal_sampling_factor + 4 * h;
                        cbcr_pixel = cbcr_pixel_row * 8 + cbcr_pixel_column;
                        mram_read(component02 + (start_row * (metadata.mcu_width_real + x)) * 64, pre_cbcr_mcu[tasklet_ID].component[1], sizeof(pre_cbcr_mcu[tasklet_ID].component[1]));
                        mram_read(component03 + (start_row * (metadata.mcu_width_real + x)) * 64, pre_cbcr_mcu[tasklet_ID].component[2], sizeof(pre_cbcr_mcu[tasklet_ID].component[2]));
                        r = current_mcus[tasklet_ID].component[0][pixel] + 1.402f * pre_cbcr_mcu[tasklet_ID].component[2][cbcr_pixel] + 128;
                        g = current_mcus[tasklet_ID].component[0][pixel] - 0.344f * pre_cbcr_mcu[tasklet_ID].component[1][cbcr_pixel] - 0.714 * pre_cbcr_mcu[tasklet_ID].component[2][cbcr_pixel] + 128;
                        b = current_mcus[tasklet_ID].component[0][pixel] + 1.772f * pre_cbcr_mcu[tasklet_ID].component[1][cbcr_pixel] + 128;
                        if(r < 0) r = 0;
                        if(r > 255) r = 255;
                        if(g < 0) g = 0;
                        if(g > 255) g = 255;
                        if(b < 0) b = 0;
                        if(b > 255) b = 255;
                        current_mcus[tasklet_ID].component[0][pixel] = r;
                        current_mcus[tasklet_ID].component[1][pixel] = g;
                        current_mcus[tasklet_ID].component[2][pixel] = b;
                    }
                }
                store_mcu(tasklet_ID, (start_row + v) * metadata.mcu_width_real + (x + h));
            }
        }
    }

    for(uint y=start_row+1; y<end_row-1; y+=metadata.vertical_sampling_factor){
        for(uint x=0; x<metadata.mcu_width_real; x+=metadata.horizontal_sampling_factor){
            mram_read(component02 + (y * (metadata.mcu_width_real + x)) * 64, pre_cbcr_mcu[tasklet_ID].component[1], sizeof(pre_cbcr_mcu[tasklet_ID].component[1]));
            mram_read(component03 + (y * (metadata.mcu_width_real + x)) * 64, pre_cbcr_mcu[tasklet_ID].component[2], sizeof(pre_cbcr_mcu[tasklet_ID].component[2]));
            for(uint v=metadata.vertical_sampling_factor-1; v<metadata.vertical_sampling_factor; v--){
                for(uint h=metadata.horizontal_sampling_factor-1; h<metadata.horizontal_sampling_factor; h--){
                    load_mcu(tasklet_ID, (y + v) * metadata.mcu_width_real + (x + h));
                    for(uint mcu_y=7; mcu_y<8; mcu_y--){
                        for(uint mcu_x=7; mcu_x<8; mcu_x--){
                            pixel = mcu_y * 8 + mcu_x;
                            cbcr_pixel_row = mcu_y / metadata.vertical_sampling_factor + 4 * v;
                            cbcr_pixel_column = mcu_x / metadata.horizontal_sampling_factor + 4 * h;
                            cbcr_pixel = cbcr_pixel_row * 8 + cbcr_pixel_column;
                            r = current_mcus[tasklet_ID].component[0][pixel] + 1.402f * pre_cbcr_mcu[tasklet_ID].component[2][cbcr_pixel] + 128;
                            g = current_mcus[tasklet_ID].component[0][pixel] - 0.344f * pre_cbcr_mcu[tasklet_ID].component[1][cbcr_pixel] - 0.714 * pre_cbcr_mcu[tasklet_ID].component[2][cbcr_pixel] + 128;
                            b = current_mcus[tasklet_ID].component[0][pixel] + 1.772f * pre_cbcr_mcu[tasklet_ID].component[1][cbcr_pixel] + 128;
                            if(r < 0) r = 0;
                            if(r > 255) r = 255;
                            if(g < 0) g = 0;
                            if(g > 255) g = 255;
                            if(b < 0) b = 0;
                            if(b > 255) b = 255;
                            current_mcus[tasklet_ID].component[0][pixel] = r;
                            current_mcus[tasklet_ID].component[1][pixel] = g;
                            current_mcus[tasklet_ID].component[2][pixel] = b;
                        }
                    }
                    store_mcu(tasklet_ID, (y + v) * metadata.mcu_width_real + (x + h));
                }
            }
        }
    }

    if(start_row != end_row - 1){
      for(uint x=0; x<end_column+1; x+=metadata.horizontal_sampling_factor){
            for(uint v=metadata.vertical_sampling_factor-1; v<metadata.vertical_sampling_factor; v--){
                for(uint h=metadata.horizontal_sampling_factor-1; h<metadata.horizontal_sampling_factor; h--){
                    load_mcu(tasklet_ID, (end_row - 1 + v) * metadata.mcu_width_real + (x + h));
                    for(uint mcu_y=7; mcu_y<8; mcu_y--){
                        for(uint mcu_x=7; mcu_x<8; mcu_x--){
                            pixel = mcu_y * 8 + mcu_x;
                            cbcr_pixel_row = mcu_y / metadata.vertical_sampling_factor + 4 * v;
                            cbcr_pixel_column = mcu_x / metadata.horizontal_sampling_factor + 4 * h;
                            cbcr_pixel = cbcr_pixel_row * 8 + cbcr_pixel_column;
                            mram_read(component02 + (end_row - 1 * (metadata.mcu_width_real + x)) * 64, pre_cbcr_mcu[tasklet_ID].component[1], sizeof(pre_cbcr_mcu[tasklet_ID].component[1]));
                            mram_read(component03 + (end_row - 1 * (metadata.mcu_width_real + x)) * 64, pre_cbcr_mcu[tasklet_ID].component[2], sizeof(pre_cbcr_mcu[tasklet_ID].component[2]));
                            r = current_mcus[tasklet_ID].component[0][pixel] + 1.402f * pre_cbcr_mcu[tasklet_ID].component[2][cbcr_pixel] + 128;
                            g = current_mcus[tasklet_ID].component[0][pixel] - 0.344f * pre_cbcr_mcu[tasklet_ID].component[1][cbcr_pixel] - 0.714 * pre_cbcr_mcu[tasklet_ID].component[2][cbcr_pixel] + 128;
                            b = current_mcus[tasklet_ID].component[0][pixel] + 1.772f * pre_cbcr_mcu[tasklet_ID].component[1][cbcr_pixel] + 128;
                            if(r < 0) r = 0;
                            if(r > 255) r = 255;
                            if(g < 0) g = 0;
                            if(g > 255) g = 255;
                            if(b < 0) b = 0;
                            if(b > 255) b = 255;
                            current_mcus[tasklet_ID].component[0][pixel] = r;
                            current_mcus[tasklet_ID].component[1][pixel] = g;
                            current_mcus[tasklet_ID].component[2][pixel] = b;
                        }
                    }
                    store_mcu(tasklet_ID, (end_row - 1 + v) * metadata.mcu_width_real + (x + h));
                }
            }
      }
    } */
    
    for(uint i=start_mcu; i<end_mcu; i++){
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