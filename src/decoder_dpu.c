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

jpeg_info metadata;

int mcu_per_tasklet;
uint32_t accumulated_cycles = 0;

void init();
void load_data();

void dequantize(int tasklet_ID);

void idct(int tasklet_ID);
void idct_component(int tasklet_ID, int component_ID);

void convert_colorspace(int tasklet_ID);

BARRIER_INIT(init_barrier, NR_TASKLETS);

int main(){
    int tasklet_ID = me();
    
    if(tasklet_ID == 0){
        perfcounter_config(COUNT_CYCLES, true);
        load_data();
        initialization = perfcounter_get();
        accumulated_cycles += initialization;
        mcu_per_tasklet = (metadata.mcu_height * metadata.mcu_width) / NR_TASKLETS;
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
    metadata.vertical_sampling_factor = metadata_buffer[5];
    metadata.horizontal_sampling_factor = metadata_buffer[6];
    for(uint i=0; i<metadata.num_components; i++){
        metadata.color_components[i].quantization_table_ID = metadata_buffer[i + 7];
    }
    for(uint i=0; i<metadata.num_components; i++){
        metadata.color_components[i].vertical_sampling_factor = metadata_buffer[i + metadata.num_components + 7];
    }
    for(uint i=0; i<metadata.num_components; i++){
        metadata.color_components[i].horizontal_sampling_factor = metadata_buffer[i + (metadata.num_components * 2) + 7];
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

void dequantize(int tasklet_ID){
    int start_mcu = tasklet_ID * mcu_per_tasklet;
    int end_mcu = tasklet_ID * mcu_per_tasklet + mcu_per_tasklet;
    if(tasklet_ID == NR_TASKLETS-1 && (metadata.mcu_height_real * metadata.mcu_width_real) % NR_TASKLETS != 0)
        end_mcu += (metadata.mcu_height_real * metadata.mcu_width_real) % NR_TASKLETS;
    int start_row = start_mcu / metadata.mcu_width;
    int start_column = start_mcu % metadata.mcu_width;
    int end_row = end_mcu / metadata.mcu_width;
    int end_column = end_mcu % metadata.mcu_width;

    for(uint x=start_column; x<metadata.mcu_width; x += metadata.horizontal_sampling_factor){
      for(uint i=0; i<metadata.num_components; i++){
        for(uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++){
          for(uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++){
            load_mcu(tasklet_ID, (start_row + v) * metadata.mcu_width_real + (x + h));
            for(uint j=0; j<64; j++)
              current_mcus[tasklet_ID].component[i][j] *= quantization_tables_cached[metadata.color_components[i].quantization_table_ID * 64 + j];
            store_mcu(tasklet_ID, (start_row + v) * metadata.mcu_width_real + (x + h));
          }
        }
      }
    }

    for(uint y=start_row+1; y<end_row - 1; y+=metadata.vertical_sampling_factor){
      for(uint x=0; x<metadata.mcu_width; x += metadata.horizontal_sampling_factor){
        for(uint i=0; i<metadata.num_components; i++){
          for(uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++){
            for(uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++){
              load_mcu(tasklet_ID, (y + v) * metadata.mcu_width_real + (x + h));
              for(uint j=0; j<64; j++)
                current_mcus[tasklet_ID].component[i][j] *= quantization_tables_cached[metadata.color_components[i].quantization_table_ID * 64 + j];
              store_mcu(tasklet_ID, (y + v) * metadata.mcu_width_real + (x + h));
            }
          }
        }
      }
    }

    for(uint x=0; x<end_mcu % metadata.mcu_width; x += metadata.horizontal_sampling_factor){
      for(uint i=0; i<metadata.num_components; i++){
        for(uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++){
          for(uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++){
            load_mcu(tasklet_ID, (end_row + v) * metadata.mcu_width_real + (x + h));
            for(uint j=0; j<64; j++)
              current_mcus[tasklet_ID].component[i][j] *= quantization_tables_cached[metadata.color_components[i].quantization_table_ID * 64 + j];
            store_mcu(tasklet_ID, (end_row + v) * metadata.mcu_width_real + (x + h));
          }
        }
      }
    }

    // printf("tasklet %d: MCU %d ~ %d (%d MCUs)\n", tasklet_ID, start_mcu, end_mcu, end_mcu-start_mcu);
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

/*
void idct_component_shift_and_add(int tasklet_ID, int component_ID){
    for (int i=0; i<8; i++) {
        int g0 = (current_mcus[tasklet_ID][(0 << 3) + i] * 181) >> 5;
        int g1 = (current_mcus[tasklet_ID][(4 << 3) + i] * 181) >> 5;
        int g2 = (current_mcus[tasklet_ID][(2 << 3) + i] * 59) >> 3;
        int g3 = (current_mcus[tasklet_ID][(6 << 3) + i] * 49) >> 4;
        int g4 = (current_mcus[tasklet_ID][(5 << 3) + i] * 71) >> 4;
        int g5 = (current_mcus[tasklet_ID][(1 << 3) + i] * 251) >> 5;
        int g6 = (current_mcus[tasklet_ID][(7 << 3) + i] * 25) >> 4;
        int g7 = (current_mcus[tasklet_ID][(3 << 3) + i] * 213) >> 5;

        int f4 = g4 - g7;
        int f5 = g5 + g6;
        int f6 = g5 - g6;
        int f7 = g4 + g7;

        int e2 = g2 - g3;
        int e3 = g2 + g3;
        int e5 = f5 - f7;
        int e7 = f5 + f7;
        int e8 = f4 + f6;

        int d2 = (e2 * 181) >> 7;
        int d4 = (f4 * 277) >> 8;
        int d5 = (e5 * 181) >> 7;
        int d6 = (f6 * 669) >> 8;
        int d8 = (e8 * 49) >> 6;

        int c0 = g0 + g1;
        int c1 = g0 - g1;
        int c2 = d2 - e3;
        int c4 = d4 + d8;
        int c5 = d5 + e7;
        int c6 = d6 - d8;
        int c8 = c5 - c6;

        int b0 = c0 + e3;
        int b1 = c1 + c2;
        int b2 = c1 - c2;
        int b3 = c0 - e3;
        int b4 = c4 - c8;
        int b6 = c6 - e7;

        current_mcus[tasklet_ID][(0 << 3) + i] = (b0 + e7) >> 4;
        current_mcus[tasklet_ID][(1 << 3) + i] = (b1 + b6) >> 4;
        current_mcus[tasklet_ID][(2 << 3) + i] = (b2 + c8) >> 4;
        current_mcus[tasklet_ID][(3 << 3) + i] = (b3 + b4) >> 4;
        current_mcus[tasklet_ID][(4 << 3) + i] = (b3 - b4) >> 4;
        current_mcus[tasklet_ID][(5 << 3) + i] = (b2 - c8) >> 4;
        current_mcus[tasklet_ID][(6 << 3) + i] = (b1 - b6) >> 4;
        current_mcus[tasklet_ID][(7 << 3) + i] = (b0 - e7) >> 4;
    }

    for (int i=0; i<8; i++) {
        int g0 = (current_mcus[tasklet_ID][(i << 3) + 0] * 181) >> 5;
        int g1 = (current_mcus[tasklet_ID][(i << 3) + 4] * 181) >> 5;
        int g2 = (current_mcus[tasklet_ID][(i << 3) + 2] * 59) >> 3;
        int g3 = (current_mcus[tasklet_ID][(i << 3) + 6] * 49) >> 4;
        int g4 = (current_mcus[tasklet_ID][(i << 3) + 5] * 71) >> 4;
        int g5 = (current_mcus[tasklet_ID][(i << 3) + 1] * 251) >> 5;
        int g6 = (current_mcus[tasklet_ID][(i << 3) + 7] * 25) >> 4;
        int g7 = (current_mcus[tasklet_ID][(i << 3) + 3] * 213) >> 5;

        int f4 = g4 - g7;
        int f5 = g5 + g6;
        int f6 = g5 - g6;
        int f7 = g4 + g7;

        int e2 = g2 - g3;
        int e3 = g2 + g3;
        int e5 = f5 - f7;
        int e7 = f5 + f7;
        int e8 = f4 + f6;

        int d2 = (e2 * 181) >> 7;
        int d4 = (f4 * 277) >> 8;
        int d5 = (e5 * 181) >> 7;
        int d6 = (f6 * 669) >> 8;
        int d8 = (e8 * 49) >> 6;

        int c0 = g0 + g1;
        int c1 = g0 - g1;
        int c2 = d2 - e3;
        int c4 = d4 + d8;
        int c5 = d5 + e7;
        int c6 = d6 - d8;
        int c8 = c5 - c6;

        int b0 = c0 + e3;
        int b1 = c1 + c2;
        int b2 = c1 - c2;
        int b3 = c0 - e3;
        int b4 = c4 - c8;
        int b6 = c6 - e7;

        current_mcus[tasklet_ID][(i << 3) + 0] = (b0 + e7) >> 4;
        current_mcus[tasklet_ID][(i << 3) + 1] = (b1 + b6) >> 4;
        current_mcus[tasklet_ID][(i << 3) + 2] = (b2 + c8) >> 4;
        current_mcus[tasklet_ID][(i << 3) + 3] = (b3 + b4) >> 4;
        current_mcus[tasklet_ID][(i << 3) + 4] = (b3 - b4) >> 4;
        current_mcus[tasklet_ID][(i << 3) + 5] = (b2 - c8) >> 4;
        current_mcus[tasklet_ID][(i << 3) + 6] = (b1 - b6) >> 4;
        current_mcus[tasklet_ID][(i << 3) + 7] = (b0 - e7) >> 4;
    }
}
*/

void idct(int tasklet_ID){
    int start_mcu = tasklet_ID * mcu_per_tasklet;
    int end_mcu = tasklet_ID * mcu_per_tasklet + mcu_per_tasklet;
    if(tasklet_ID == NR_TASKLETS-1 && (metadata.mcu_height_real * metadata.mcu_width_real) % NR_TASKLETS != 0)
        end_mcu += (metadata.mcu_height_real * metadata.mcu_width_real) % NR_TASKLETS;
    
    int start_row = start_mcu / metadata.mcu_width;
    int start_column = start_mcu % metadata.mcu_width;
    int end_row = end_mcu / metadata.mcu_width;
    int end_column = end_mcu % metadata.mcu_width;

    for(uint x=start_column; x<metadata.mcu_width; x += metadata.horizontal_sampling_factor){
      for(uint i=0; i<metadata.num_components; i++){
        for(uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++){
          for(uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++){
            load_mcu(tasklet_ID, (start_row + v) * metadata.mcu_width_real + (x + h));
            idct_component(tasklet_ID, i);
            store_mcu(tasklet_ID, (start_row + v) * metadata.mcu_width_real + (x + h));
          }
        }
      }
    }

    for(uint y=start_row+1; y<end_row - 1; y+=metadata.vertical_sampling_factor){
      for(uint x=0; x<metadata.mcu_width; x += metadata.horizontal_sampling_factor){
        for(uint i=0; i<metadata.num_components; i++){
          for(uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++){
            for(uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++){
              load_mcu(tasklet_ID, (y + v) * metadata.mcu_width_real + (x + h));
              idct_component(tasklet_ID, i);
              store_mcu(tasklet_ID, (y + v) * metadata.mcu_width_real + (x + h));
            }
          }
        }
      }
    }

    for(uint x=0; x<end_mcu % metadata.mcu_width; x += metadata.horizontal_sampling_factor){
      for(uint i=0; i<metadata.num_components; i++){
        for(uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++){
          for(uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++){
            load_mcu(tasklet_ID, (end_row + v) * metadata.mcu_width_real + (x + h));
            idct_component(tasklet_ID, i);
            store_mcu(tasklet_ID, (end_row + v) * metadata.mcu_width_real + (x + h));
          }
        }
      }
    }
}

void convert_colorspace(int tasklet_ID){
    int start_mcu = tasklet_ID * mcu_per_tasklet;
    int end_mcu = tasklet_ID * mcu_per_tasklet + mcu_per_tasklet;
    if(tasklet_ID == NR_TASKLETS-1 && (metadata.mcu_height_real * metadata.mcu_width_real) % NR_TASKLETS != 0)
        end_mcu += (metadata.mcu_height_real * metadata.mcu_width_real) % NR_TASKLETS;
    
    int start_row = start_mcu / metadata.mcu_width;
    int start_column = start_mcu % metadata.mcu_width;
    int end_row = end_mcu / metadata.mcu_width;
    int end_column = end_mcu % metadata.mcu_width;

    short r, g, b;

    for(uint x=start_column; x<metadata.mcu_width; x += metadata.horizontal_sampling_factor){
      for(uint i=0; i<metadata.num_components; i++){
        for(uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++){
          for(uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++){
            load_mcu(tasklet_ID, (start_row + v) * metadata.mcu_width_real + (x + h));
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
            store_mcu(tasklet_ID, (start_row + v) * metadata.mcu_width_real + (x + h));
          }
        }
      }
    }

    for(uint y=start_row+1; y<end_row - 1; y+=metadata.vertical_sampling_factor){
      for(uint x=0; x<metadata.mcu_width; x += metadata.horizontal_sampling_factor){
        for(uint i=0; i<metadata.num_components; i++){
          for(uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++){
            for(uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++){
              load_mcu(tasklet_ID, (y + v) * metadata.mcu_width_real + (x + h));
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
              store_mcu(tasklet_ID, (y + v) * metadata.mcu_width_real + (x + h));
            }
          }
        }
      }
    }

    for(uint x=0; x<end_mcu % metadata.mcu_width; x += metadata.horizontal_sampling_factor){
      for(uint i=0; i<metadata.num_components; i++){
        for(uint v=0; v<metadata.color_components[i].vertical_sampling_factor; v++){
          for(uint h=0; h<metadata.color_components[i].horizontal_sampling_factor; h++){
            load_mcu(tasklet_ID, (end_row + v) * metadata.mcu_width_real + (x + h));
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
            store_mcu(tasklet_ID, (end_row + v) * metadata.mcu_width_real + (x + h));
          }
        }
      }
    }
}