#include <mram.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <defs.h>
#include <barrier.h>

#include "common.h"

typedef struct {
  int y_r[64];
  int cb_g[64];
  int cr_b[64];
} MCU;

typedef struct {
  byte offsets[17];
  byte symbols[162];
  unsigned int codes[162];
  byte set;
} HuffmanTable;

typedef struct {
    uint table[64];
    byte set;
} QuantizationTable;

typedef struct {
  byte horizontalSamplingFactor;
  byte verticalSamplingFactor;
  byte quantizationTableID;
  byte huffmanDCTableID;
  byte huffmanACTableID;
} ColorComponent;

typedef struct {
  int width;
  int height;
  ColorComponent colorComponents[3];
  int numComponents;
  int restartInterval;
} Metadata;

__mram uint32_t buffer[BUFFER_SIZE];
__mram int y_r[64 * 5000];
__mram int cb_g[64 * 5000];
__mram int cr_b[64 * 5000];

__dma_aligned MCU current_mcus[NR_TASKLETS];

HuffmanTable huffmanDCTables[4];
HuffmanTable huffmanACTables[4];
QuantizationTable quantizationTables[4];
Metadata metadata;

BARRIER_INIT(init_barrier, NR_TASKLETS);

/*
   --- byteCursor = 0 --- | --- byteCursor = 1 ---
   0  0  1  0  0  1  0  1   1  1  0  0  1  0  0  0
   ↑           ↑            ↑
   bit         bit          bit
   Cursor=0    Cursor=4     Cursor=0

*/
int byteCursor = 0;
int bitCursor = 0;

void init(){
  for(int i=0; i<4; i++){
    huffmanDCTables[i].set = 0;
    huffmanACTables[i].set = 0;
    quantizationTables[i].set = 0;
  }
}

int readBit(){
    if(byteCursor >= BUFFER_SIZE) return -1;
    int bit = (buffer[byteCursor] >> (7 - bitCursor)) & 1;
    bitCursor += 1;
    if(bitCursor == 8){
        bitCursor = 0;
        byteCursor += 1;
    }
    return bit;
}

int readBits(int length){
    int bits = 0;
    for(uint i=0; i<length; i++){
        int bit = readBit();
        if(bit == -1) return -1;
        bits = (bits << 1) | bit;
    }
    return bits;
}

void align(){
    if(byteCursor >= BUFFER_SIZE) return;
    if(bitCursor != 0){
        bitCursor = 0;
        byteCursor += 1;
    }
}

void load_data(){
  int cursor = 0;

  for(int i=0; i<4; i++){
    if(buffer[cursor++] != 1) break;
    for(int j=0; j<17; j++) huffmanDCTables[i].offsets[j] = buffer[cursor++];
    for(int j=0; j<162; j++) huffmanDCTables[i].symbols[j] = buffer[cursor++];      
    for(int j=0; j<162; j++) huffmanDCTables[i].codes[j] = buffer[cursor++];
    huffmanDCTables[i].set = 1;
  }

  for(int i=0; i<4; i++){
    if(buffer[cursor++] != 1) break;
    for(int j=0; j<17; j++) huffmanACTables[i].offsets[j] = buffer[cursor++];
    for(int j=0; j<162; j++) huffmanACTables[i].symbols[j] = buffer[cursor++];      
    for(int j=0; j<162; j++) huffmanACTables[i].codes[j] = buffer[cursor++];
    huffmanACTables[i].set = 1;
  }

  for(int i=0; i<4; i++){
    if(buffer[cursor++] != 1) break;
    for(int j=0; j<64; j++) quantizationTables[i].table[j] = buffer[cursor++];
    quantizationTables[i].set = 1;
  }

  metadata.width = buffer[cursor++];
  metadata.height = buffer[cursor++];
  metadata.numComponents = buffer[cursor++];
  for(int i=0; i<metadata.numComponents; i++){
    metadata.colorComponents[i].huffmanDCTableID = buffer[cursor++];
    metadata.colorComponents[i].huffmanACTableID = buffer[cursor++];
    metadata.colorComponents[i].quantizationTableID = buffer[cursor++];
  }
  metadata.restartInterval = buffer[cursor++];
  byteCursor = cursor;
}

byte getNextSymbol(HuffmanTable *hTable){
    uint code = 0;

    for(int i=0; i<16; i++){
        int bit = readBit();
        code = (code << 1) | bit;
        for(uint j=hTable->offsets[i]; j<hTable->offsets[i+1]; j++){
            if(code == hTable->codes[j]) return hTable->symbols[j];
        }
    }
    return -1;
}

int decodeMCUComponent(int *component, int *preDC, int componentID){
    HuffmanTable huffmanDCTable = huffmanDCTables[metadata.colorComponents[componentID].huffmanDCTableID];
    HuffmanTable huffmanACTable = huffmanACTables[metadata.colorComponents[componentID].huffmanACTableID];
    
    byte length = getNextSymbol(&huffmanDCTable);
    
    if(length == (byte)-1){
        printf("Error - Invalid DC value (length ==  %d)\n", length);
        return -1;
    }
    if(length > 11){
        printf("Error - DC coefficient length greater than 11\n");
        return -1;
    }

    int coefficient = readBits(length);
    if(coefficient == -1){
        printf("Error - Invalid DC value\n");
        return -1;
    }
    if(length != 0 && coefficient < (1 << (length - 1)))
        coefficient -= (1 << length) - 1;
    component[0] = coefficient + (*preDC);
    (*preDC) = component[0];

    uint i = 1;
    
    while(i < 64){
        byte symbol = getNextSymbol(&huffmanACTable);
        if(symbol == (byte)-1){
            printf("Error - Invalid AC value\n");
            return -1;
        }
        if(symbol == 0x00){
            for(; i<64; i++) component[zigzagMap[i]] = 0;
            return 1;
        }
        
        byte numZeros = symbol >> 4;
        byte coefficientLen = symbol & 0x0F;
        coefficient = 0;

        if(symbol == 0xF0) numZeros = 16;
        if(i + numZeros >= 64){
            printf("Error - Zero run-length exceeded MCU (i=%d, numZeros=%d)\n", i, numZeros);
            return -1;
        }

        for(uint j=0; j<numZeros; j++, i++){
            component[zigzagMap[i]] = 0;
        }
        if(coefficientLen > 10){
            printf("Error - AC coefficient length greater than 10\n");
            return -1;
        }

        if(coefficientLen != 0){
            coefficient = readBits(coefficientLen);
            if(coefficient == -1){
                printf("Error - Invalid AC value\n");
                return -1;
            }
            if(coefficient < (1 << (coefficientLen - 1))) coefficient -= (1 << coefficientLen) - 1;
            component[zigzagMap[i]] = coefficient;
            i += 1;
        }
    }
    return 1;
}


void decodeHuffmanData(){
  int mcuWidth = (metadata.width + 7) / 8;
  int mcuHeight = (metadata.height + 7) / 8;

  for(int u=0; u<4; u++){
    if(huffmanDCTables[u].set == 1){
        unsigned int code = 0;
        for(int i=0; i<16; i++){
            for(int j=huffmanDCTables[u].offsets[i]; j<huffmanDCTables[u].offsets[i+1]; j++){
                huffmanDCTables[u].codes[j] = code;
                code += 1;
            }
            code <<= 1;
        }
    }
    if(huffmanACTables[u].set == 1){
        unsigned int code = 0;
        for(int i=0; i<16; i++){
            for(int j=huffmanACTables[u].offsets[i]; j<huffmanACTables[u].offsets[i+1]; j++){
                huffmanACTables[u].codes[j] = code;
                code += 1;
            }
            code <<= 1;
        }        
    };
  }

  int preDCs[3] = {0, 0, 0};
  printf("Huffman Decoding\n");

  for(int i=0; i<mcuWidth*mcuHeight; i++){
    if(metadata.restartInterval != 0 && i % metadata.restartInterval == 0){
        preDCs[0] = 0; preDCs[1] = 0; preDCs[2] = 0;
        align();
    }
    if(metadata.numComponents == 3){
        if(decodeMCUComponent(current_mcus[0].y_r, &preDCs[0], 0) == -1){
            printf("ERROR! (%dth MCU component Y)\n", i);
            return;
        }
        if(decodeMCUComponent(current_mcus[0].cb_g, &preDCs[1], 1) == -1){
            printf("ERROR! (%dth MCU component Cb)\n", i);
            return;
        }
        if(decodeMCUComponent(current_mcus[0].cr_b, &preDCs[2], 2) == -1){
            printf("ERROR! (%dth MCU component Cr)\n", i);
            return;
        }
    }
    mram_write(current_mcus[0].y_r, y_r + 64 * i, sizeof(current_mcus[0].y_r));
    mram_write(current_mcus[0].cb_g, cb_g + 64 * i, sizeof(current_mcus[0].cb_g));
    mram_write(current_mcus[0].cr_b, cr_b + 64 * i, sizeof(current_mcus[0].cr_b));
  }
}

void dequantizeMCUComponent(int tasklet){
    for(uint i=0; i<64; i++){
        current_mcus[tasklet].y_r[i] *= quantizationTables[metadata.colorComponents[0].quantizationTableID].table[i];
        current_mcus[tasklet].cb_g[i] *= quantizationTables[metadata.colorComponents[1].quantizationTableID].table[i];
        current_mcus[tasklet].cr_b[i] *= quantizationTables[metadata.colorComponents[2].quantizationTableID].table[i];
    }
}

void dequantize(int tasklet){
    const uint mcuHeight = (metadata.height + 7) / 8;
    const uint mcuWidth = (metadata.width + 7) / 8;
    const uint mcuPerTasklet = mcuHeight*mcuWidth / NR_TASKLETS;
    for(uint i=tasklet*mcuPerTasklet; i<tasklet*mcuPerTasklet+mcuPerTasklet; i++){
        mram_read(y_r + i * 64, current_mcus[tasklet].y_r, sizeof(current_mcus[tasklet].y_r));
        mram_read(cb_g + i * 64, current_mcus[tasklet].cb_g, sizeof(current_mcus[tasklet].cb_g));
        mram_read(cr_b + i * 64, current_mcus[tasklet].cr_b, sizeof(current_mcus[tasklet].cr_b));
        dequantizeMCUComponent(tasklet);
        mram_write(current_mcus[tasklet].y_r, y_r + i * 64, sizeof(current_mcus[tasklet].y_r));
        mram_write(current_mcus[tasklet].cb_g, cb_g + i * 64, sizeof(current_mcus[tasklet].cb_g));
        mram_write(current_mcus[tasklet].cr_b, cr_b + i * 64, sizeof(current_mcus[tasklet].cr_b));
    }
    printf("tasklet %d: MCU %d ~ %d (%d MCUs)\n", tasklet, tasklet*mcuPerTasklet, tasklet*mcuPerTasklet+mcuPerTasklet, mcuPerTasklet);
}

void inverseDCTComponent(int tasklet){
    float result[64] = { 0 };
    for(uint i=0; i<8; i++){
        for(uint y=0; y<8; y++){
            float sum = 0.0;
            for(uint v=0; v<8; v++){
                sum += current_mcus[tasklet].y_r[v * 8 + i] * idctMap[v * 8 + y];
            }
            result[y * 8 + i] = sum;
        }
    }

    for(uint i=0; i<8; i++){
        for(uint x=0; x<8; x++){
            float sum = 0.0f;
            for(uint u=0; u<8; u++){
                sum += result[i * 8 + u] * idctMap[u * 8 + x];
            }
            current_mcus[tasklet].y_r[i * 8 + x] = (int)sum;
        }
    }

    for(uint i=0; i<8; i++){
        for(uint y=0; y<8; y++){
            float sum = 0.0;
            for(uint v=0; v<8; v++){
                sum += current_mcus[tasklet].cb_g[v * 8 + i] * idctMap[v * 8 + y];
            }
            result[y * 8 + i] = sum;
        }
    }

    for(uint i=0; i<8; i++){
        for(uint x=0; x<8; x++){
            float sum = 0.0f;
            for(uint u=0; u<8; u++){
                sum += result[i * 8 + u] * idctMap[u * 8 + x];
            }
            current_mcus[tasklet].cb_g[i * 8 + x] = (int)sum;
        }
    }

    for(uint i=0; i<8; i++){
        for(uint y=0; y<8; y++){
            float sum = 0.0;
            for(uint v=0; v<8; v++){
                sum += current_mcus[tasklet].cr_b[v * 8 + i] * idctMap[v * 8 + y];
            }
            result[y * 8 + i] = sum;
        }
    }

    for(uint i=0; i<8; i++){
        for(uint x=0; x<8; x++){
            float sum = 0.0f;
            for(uint u=0; u<8; u++){
                sum += result[i * 8 + u] * idctMap[u * 8 + x];
            }
            current_mcus[tasklet].cr_b[i * 8 + x] = (int)sum;
        }
    }
}

void inverseDCT(int tasklet){
    const uint mcuHeight = (metadata.height + 7) / 8;
    const uint mcuWidth = (metadata.width + 7) / 8;
    const uint mcuPerTasklet = mcuHeight*mcuWidth / NR_TASKLETS;
    for(uint i=tasklet*mcuPerTasklet; i<tasklet*mcuPerTasklet+mcuPerTasklet; i++){
        mram_read(y_r + i * 64, current_mcus[tasklet].y_r, sizeof(current_mcus[tasklet].y_r));
        mram_read(cb_g + i * 64, current_mcus[tasklet].cb_g, sizeof(current_mcus[tasklet].cb_g));
        mram_read(cr_b + i * 64, current_mcus[tasklet].cr_b, sizeof(current_mcus[tasklet].cr_b));
        inverseDCTComponent(tasklet);
        mram_write(current_mcus[tasklet].y_r, y_r + i * 64, sizeof(current_mcus[tasklet].y_r));
        mram_write(current_mcus[tasklet].cb_g, cb_g + i * 64, sizeof(current_mcus[tasklet].cb_g));
        mram_write(current_mcus[tasklet].cr_b, cr_b + i * 64, sizeof(current_mcus[tasklet].cr_b));
    }
}

void YCbCrToRGBMCU(int tasklet){
    int r, g, b;
    for(int i=0; i<64; i++){
        r = current_mcus[tasklet].y_r[i] + 1.402f * current_mcus[tasklet].cr_b[i] + 128;
        g = current_mcus[tasklet].y_r[i] - 0.344f * current_mcus[tasklet].cb_g[i] - 0.714 * current_mcus[tasklet].cr_b[i] + 128;
        b = current_mcus[tasklet].y_r[i] + 1.772f * current_mcus[tasklet].cb_g[i] + 128;
        if(r < 0) r = 0;
        if(r > 255) r = 255;
        if(g < 0) g = 0;
        if(g > 255) g = 255;
        if(b < 0) b = 0;
        if(b > 255) b = 255;
        current_mcus[tasklet].y_r[i] = r;
        current_mcus[tasklet].cb_g[i] = g;
        current_mcus[tasklet].cr_b[i] = b;
    }
}

void YCbCrToRGB(int tasklet){
    const uint mcuHeight = (metadata.height + 7) / 8;
    const uint mcuWidth = (metadata.width + 7) / 8;
    const uint mcuPerTasklet = mcuHeight*mcuWidth / NR_TASKLETS;
    for(uint i=tasklet*mcuPerTasklet; i<tasklet*mcuPerTasklet+mcuPerTasklet; i++){
        mram_read(y_r + i * 64, current_mcus[tasklet].y_r, sizeof(current_mcus[tasklet].y_r));
        mram_read(cb_g + i * 64, current_mcus[tasklet].cb_g, sizeof(current_mcus[tasklet].cb_g));
        mram_read(cr_b + i * 64, current_mcus[tasklet].cr_b, sizeof(current_mcus[tasklet].cr_b));
        YCbCrToRGBMCU(tasklet);
        mram_write(current_mcus[tasklet].y_r, y_r + i * 64, sizeof(current_mcus[tasklet].y_r));
        mram_write(current_mcus[tasklet].cb_g, cb_g + i * 64, sizeof(current_mcus[tasklet].cb_g));
        mram_write(current_mcus[tasklet].cr_b, cr_b + i * 64, sizeof(current_mcus[tasklet].cr_b));
    }
}

int main(){
    if(me() == 0){
        init();
        load_data();
        decodeHuffmanData();
        dequantize(me());
        inverseDCT(me());
        YCbCrToRGB(me());
    }else{
        barrier_wait(&init_barrier);
        dequantize(me());
        inverseDCT(me());
        YCbCrToRGB(me());
    }
    barrier_wait(&init_barrier);
    
    return 0;
}