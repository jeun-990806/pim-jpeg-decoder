#include <iostream>
#include <fstream>

#include "headers/bmp.h"
#include "headers/common.h"

void put_integer(std::ofstream& output, const uint v){
    output.put((v >> 0) & 0xFF);
    output.put((v >> 8) & 0xFF);
    output.put((v >> 16) & 0xFF);
    output.put((v >> 24) & 0xFF);
}

void put_short(std::ofstream &output, const uint v){
    output.put((v >> 0) & 0xFF);
    output.put((v >> 8) & 0xFF);
}

void write_BMP(std::vector<uint32_t>& metadata, std::vector<std::vector<short>>& mcus, int start_dpu_index, const std::string& filename){
    std::ofstream output = std::ofstream(filename, std::ios::out | std::ios::binary);
    if(!output.is_open()){
        std::cout << filename << ": Error - Unable to create BMP file" << std::endl;
        return;
    }

    const int width = metadata[18];
    const int height = metadata[17];
    const uint padding = width % 4;
    const uint size = 14 + 12 + height * width * 3 + padding * height;
    const int max_blk_per_dpu = MAX_MCU_PER_DPU / 4;

    output.put('B');
    output.put('M');
    put_integer(output, size);
    put_integer(output, 0);
    put_integer(output, 0x1A);
    put_integer(output, 12);
    put_short(output, width);
    put_short(output, height);
    put_short(output, 1);
    put_short(output, 24);

    for(uint y=height-1; y<height; y--){
        const uint mcu_row = y / 8;
        const uint pixel_row = y % 8;
        for(uint x=0; x<width; x++){
            const uint mcu_column = x / 8;
            const uint pixel_column = x % 8;
            const uint pixel_index = pixel_row * 8 + pixel_column;

            int mcu_index = mcu_row * metadata[3] + mcu_column;
            int block_index = (mcu_index / (metadata[3] * 2)) * ((metadata[3] + 1) / 2) + ((mcu_index % metadata[3]) / 2);
            int block_position = ((mcu_index / metadata[3]) % 2) * 2 + ((mcu_index % metadata[3]) % 2);
            int dpu_index = block_index / max_blk_per_dpu;

            block_index %= max_blk_per_dpu;

            output.put(mcus[start_dpu_index + dpu_index][(block_index * 768) + 512 + (block_position * 64) + pixel_index]);    // 2
            output.put(mcus[start_dpu_index + dpu_index][(block_index * 768) + 256 + (block_position * 64) + pixel_index]);    // 1
            output.put(mcus[start_dpu_index + dpu_index][(block_index * 768) + (block_position * 64) + pixel_index]);    // 0
        }
        for(uint i=0; i<padding; i++){
            output.put(0);
        }
    }
    output.close();
}