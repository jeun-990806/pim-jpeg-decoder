#ifndef JPG_H
#define JPG_H

#include <vector>
#include "common.h"

const byte SOF0 = 0xC0; // Baseline DCT
const byte SOF1 = 0xC1; // Extended sequential DCT
const byte SOF2 = 0xC2; // Progressive DCT
const byte SOF3 = 0xC3; // Lossless (sequential)

// Start of Frame markers, differential, Huffman coding
const byte SOF5 = 0xC5; // Differential sequential DCT
const byte SOF6 = 0xC6; // Differential progressive DCT
const byte SOF7 = 0xC7; // Differential lossless (sequential)

// Start of Frame markers, non-differential, arithmetic coding
const byte SOF9 = 0xC9; // Extended sequential DCT
const byte SOF10 = 0xCA; // Progressive DCT
const byte SOF11 = 0xCB; // Lossless (sequential)

// Start of Frame markers, differential, arithmetic coding
const byte SOF13 = 0xCD; // Differential sequential DCT
const byte SOF14 = 0xCE; // Differential progressive DCT
const byte SOF15 = 0xCF; // Differential lossless (sequential)

// APPN Makers
const byte APP0 = 0xE0;
const byte APP1 = 0xE1;
const byte APP2 = 0xE2;
const byte APP3 = 0xE3;
const byte APP4 = 0xE4;
const byte APP5 = 0xE5;
const byte APP6 = 0xE6;
const byte APP7 = 0xE7;
const byte APP8 = 0xE8;
const byte APP9 = 0xE9;
const byte APP10 = 0xEA;
const byte APP11 = 0xEB;
const byte APP12 = 0xEC;
const byte APP13 = 0xED;
const byte APP14 = 0xEE;
const byte APP15 = 0xEF;

// Restart Markers
const byte RST0 = 0xD0;
const byte RST1 = 0xD1;
const byte RST2 = 0xD2;
const byte RST3 = 0xD3;
const byte RST4 = 0xD4;
const byte RST5 = 0xD5;
const byte RST6 = 0xD6;
const byte RST7 = 0xD7;

// JPEG extensions
const byte JPG0 = 0xF0;
const byte JPG1 = 0xF1;
const byte JPG2 = 0xF2;
const byte JPG3 = 0xF3;
const byte JPG4 = 0xF4;
const byte JPG5 = 0xF5;
const byte JPG6 = 0xF6;
const byte JPG7 = 0xF7;
const byte JPG8 = 0xF8;
const byte JPG9 = 0xF9;
const byte JPG10 = 0xFA;
const byte JPG11 = 0xFB;
const byte JPG12 = 0xFC;
const byte JPG13 = 0xFD;

const byte COM = 0xFE;
const byte TEM = 0x01;

const byte SOI = 0xD8;
const byte EOI = 0xD9;
const byte SOS = 0xDA;
const byte DQT = 0xDB;
const byte DNL = 0xDC;
const byte DRI = 0xDD;
const byte DHP = 0xDE;
const byte EXP = 0xDF;

const byte DHT = 0xC4;


struct QuantizationTable {
    uint table[64] = { 0 };
    bool set = false;

    QuantizationTable() {
        for(uint y=0; y<8; y++){
            for(uint x=0; x<8; x++){
                table[y * 8 + x];
            }
        }
    }
};

struct HuffmanTable {
    byte offsets[17] = { 0 };
    byte symbols[162] = { 0 };
    uint codes[162] = { 0 };
    bool set = false;
};

struct ColorComponent {
    byte horizontalSamplingFactor = 1;
    byte verticalSamplingFactor = 1;
    byte quantizationTableID = 0;
    byte huffmanDCTableID = 0;
    byte huffmanACTableID = 0;
    bool usedInFrame = false;
    bool usedInScan = false;
};

struct Header {
    QuantizationTable quantizationTables[4];
    HuffmanTable huffmanDCTables[4];
    HuffmanTable huffmanACTables[4];

    byte frameType = 0;
    uint height = 0;
    uint width = 0;
    byte numComponents = 0;
    ColorComponent colorComponents[3];
    bool zeroBased = false;

    byte componentsInScan = 0;
    byte startOfSelection = 0;
    byte endOfSelection = 63;
    byte successiveApproximationHigh = 0;
    byte successiveApproximationLow = 0;

    uint restartInterval = 0;

    std::vector<byte> huffmanData;

    bool valid = true;

    uint mcuHeight = 0;
    uint mcuWidth = 0;
    uint mcuHeightReal = 0;
    uint mcuWidthReal = 0;

    byte horizontalSamplingFactor = 1;
    byte verticalSamplingFactor = 1;
};

struct MCU {
    union {
        int y[64] = { 0 };
        int r[64];
    };
    union {
        int cb[64] = { 0 };
        int g[64];
    };
    union {
        int cr[64] = { 0 };
        int b[64];
    };   
    int *operator[](uint i){
        switch(i){
            case 0:
                return y;
            case 1:
                return cb;
            case 2:
                return cr;
            default:
                return nullptr;
        }
    }
};

struct JPEG {
    std::string filename;
    Header *header;
    MCU *mcus;
};

#endif
