#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <dpu>

#include "jpg.h"

using namespace dpu;

class BitReader {
    private:
        const std::vector<byte>& data;
        uint nextByte = 0;
        uint nextBit = 0;
    public:
        BitReader(const std::vector<byte>& d) :
        data(d)
        {}

        int readBit(){
            if(nextByte >= data.size()) return -1;
            int bit = (data[nextByte] >> (7-nextBit)) & 1;
            nextBit += 1;
            if(nextBit == 8){
                nextBit = 0;
                nextByte += 1;
            }
            return bit;
        }
        
        int readBits(const uint length){
            int bits = 0;
            for(uint i=0; i<length; i++){
                int bit = readBit();
                if(bit == -1){
                    bits = -1;
                    break;
                }
                bits = (bits << 1) | bit;
            }
            return bits;
        }

        void align(){
            if(nextByte >= data.size()) return;
            if(nextBit != 0){
                nextBit = 0;
                nextByte += 1;
            }
        }
};

void putInt(std::ofstream& outFile, const uint v){
    outFile.put((v >> 0) & 0xFF);
    outFile.put((v >> 8) & 0xFF);
    outFile.put((v >> 16) & 0xFF);
    outFile.put((v >> 24) & 0xFF);
}

void putShort(std::ofstream &outFile, const uint v){
    outFile.put((v >> 0) & 0xFF);
    outFile.put((v >> 8) & 0xFF);
}

void readStartOfScan(std::ifstream& inFile, Header *const header){
    std::cout << "Reading SOS Marker\n";
    if(header->numComponents == 0){
        std::cout << "Error - SOS detected before SOF\n";
        header->valid = false;
        return;
    }

    uint length = (inFile.get() << 8) + inFile.get();

    for(uint i=0; i<header->numComponents; i++){
        header->colorComponents[i].used = false;
    }

    byte numComponents = inFile.get();
    for(uint i=0; i<numComponents; i++){
        byte componentID = inFile.get();
        if(header->zeroBased) componentID += 1;
        if(componentID > header->numComponents){
            std::cout << "Error - Invalid color component ID: " << (uint)componentID << "\n";
            header->valid = false;
            return;
        }
        ColorComponent *component = &header->colorComponents[componentID - 1];
        if(component->used){
            std::cout << "Duplicate color component ID\n";
            header->valid = false;
            return;            
        }
        component->used = true;

        byte huffmanTableIDs = inFile.get();
        component->huffmanDCTableID = huffmanTableIDs >> 4;
        component->huffmanACTableID = huffmanTableIDs & 0x0F;
        if(component->huffmanDCTableID > 3){
            std::cout << "Error - Invalid Huffman DC table ID: " << (uint)component->huffmanDCTableID << "\n";
            header->valid = false;
            return;
        }
        if(component->huffmanACTableID > 3){
            std::cout << "Error - Invalid Huffman AC table ID: " << (uint)component->huffmanACTableID << "\n";
            header->valid = false;
            return;
        }
    }

    header->startOfSelection = inFile.get();
    header->endOfSelection = inFile.get();
    byte successiveApproximation = inFile.get();
    header->successiveApproximationHigh = successiveApproximation >> 4;
    header->successiveApproximationLow = successiveApproximation & 0x0F;

    if(header->startOfSelection !=0 || header->endOfSelection != 63){
        std::cout << "Error - Invalid spectral selection\n";
        header->valid = false;
        return;
    }
    if(header->successiveApproximationHigh != 0 || header->successiveApproximationLow != 0){
        std::cout << "Error - Invalid successive approximation\n";
        header->valid = false;
        return;
    }

    if(length - 6 - (2 * numComponents) != 0){
        std::cout << "Error - SOS invalid\n";
        header->valid = false;
        return;
    }
}

void readHuffmanTable(std::ifstream& inFile, Header *const header){
    std::cout << "Reading DHT Marker\n";
    int length = (inFile.get() << 8) + inFile.get();
    length -= 2;

    while(length > 0){
        byte tableInfo = inFile.get();
        byte tableID = tableInfo & 0x0F;
        bool ACTable = tableInfo >> 4;

        if(tableID > 3){
            std::cout << "Error - Invalid Huffman table ID: " << (uint)tableID << "\n";
            header->valid = false;
            return;
        }

        HuffmanTable *hTable;
        if(ACTable){
            hTable = &header->huffmanACTables[tableID];
        }else{
            hTable = &header->huffmanDCTables[tableID];
        }
        hTable->set = true;

        hTable->offsets[0] = 0;
        uint allSymbols = 0;
        for(uint i=1; i<=16; i++){
            allSymbols += inFile.get();
            hTable->offsets[i] = allSymbols;
        }
        if(allSymbols > 162){
            std::cout << "Error - Too many symbols in Huffman table\n";
            header->valid = false;
            return;
        }

        for(uint i=0; i<allSymbols; i++){
            hTable->symbols[i] = inFile.get();
        }

        length -= 17 + allSymbols;
    }
    if(length != 0){
        std::cout << "Error - DHT invalid\n";
        header->valid = false;
    }
}

void readStartOfFrame(std::ifstream& inFile, Header *const header){
    std::cout << "Reading SOF Marker\n";
    if(header->numComponents != 0){
        std::cout << "Error - Multiple SOFs detected\n";
        header->valid = false;
        return;
    }

    uint length = (inFile.get() << 8) + inFile.get();

    byte precision = inFile.get();
    if(precision != 8){
        std::cout << "Error - Invalid precision: " << (uint)precision << "\n";
        header->valid = false;
        return;
    }

    header->height = (inFile.get() << 8) + inFile.get();
    header->width = (inFile.get() << 8) + inFile.get();
    if(header->height == 0 || header->width == 0){
        std::cout << "Error - Invalid dimensions\n";
        header->valid = false;
        return;
    }

    header->numComponents = inFile.get();
    if(header->numComponents == 4){
        std::cout << "Error - CMYK color mode not supported\n";
        header->valid = false;
        return;
    }
    if(header->numComponents == 0){
        std::cout << "Error - Number of color components must not be zero\n";
        header->valid = false;
        return;
    }
    for(uint i=0; i<header->numComponents; i++){
        byte componentID = inFile.get();
        // Component IDs are usually 1-3 but rarely can be seen as 0-2
        // always force them into 1-3 for consistency.
        if(componentID == 0) header->zeroBased = true;
        if(header->zeroBased) componentID += 1;
        if(componentID == 4 || componentID == 5){
            std::cout << "Error - YIQ color mode not supported\n";
            header->valid = false;
            return;
        }
        if(componentID == 0 || componentID > 3){
            std::cout << "Error - Invalid component ID:" << (uint)componentID << "\n";
            header->valid = false;
            return;
        }
        ColorComponent *component = &header->colorComponents[componentID - 1];
        if(component->used){
            std::cout << "Duplicate color component ID\n";
            header->valid = false;
            return;
        }
        component->used = true;
        byte samplingFactor = inFile.get();
        component->horizontalSamplingFactor = samplingFactor >> 4;
        component->verticalSamplingFactor = samplingFactor & 0x0F;
        if(component->horizontalSamplingFactor != 1 || component->verticalSamplingFactor != 1){
            std::cout << "Error - Sampling factors not supported\n";
            header->valid = false;
            return;
        }
        component->quantizationTableID = inFile.get();
        if(component->quantizationTableID > 3){
            std::cout << "Error - Invalid quantization table ID in frame components\n";
            header->valid = false;
            return;
        }
    }

    if(length - 8 - (3 * header->numComponents) != 0){
        std::cout << "Error - SOF invalid\n";
        header->valid = false;
    }
}

void readQuantizationTable(std::ifstream& inFile, Header *const header){
    std::cout << "Reading DQT Marker\n";
    int length = (inFile.get() << 8) + inFile.get();
    length -= 2;

    while(length > 0){
        byte tableInfo = inFile.get();
        length -= 1;
        byte tableID = tableInfo & 0x0F;

        if(tableID > 3){
            std::cout << "Error Invalid quantization table ID: " << (uint)tableID << "\n";
            header->valid = false;
            return;
        }
        
        header->quantizationTables[tableID].set = true;

        if(tableInfo >> 4 != 0){
            for(uint i=0; i<64; i++){
                header->quantizationTables[tableID].table[zigzagMap[i]] = (inFile.get() << 8) + inFile.get();
            }
            length -= 128;
        }else{
            for(uint i=0; i<64; i++){
                header->quantizationTables[tableID].table[zigzagMap[i]] = inFile.get();
            }
            length -= 64;
        }
    }

    if(length != 0){
        std::cout << "Error - DQT invalid\n";
        header->valid = false;
    }
}

void readRestartInterval(std::ifstream& inFile, Header *const header){
    std::cout << "Reading DRI Marker\n";
    uint length = (inFile.get() << 8) + inFile.get();

    header->restartInterval = (inFile.get() << 8) + inFile.get();
    if(length - 4 != 0){
        std::cout << "Error - DRI invalid\n";
        header->valid = false;
    }
}

void readAPPN(std::ifstream& inFile, Header *const header){
    std::cout << "Reading APPN Marker\n";
    uint length = (inFile.get() << 8) + inFile.get();

    for(uint i=0; i<length-2; ++i) inFile.get();
}

void readComment(std::ifstream& inFile, Header *const header){
    std::cout << "Reading COM Marker\n";
    uint length = (inFile.get() << 8) + inFile.get();

    for(uint i=0; i<length-2; i++){
        inFile.get();
    }
}

Header *readJPG(const std::string& filename){
    std::ifstream inFile = std::ifstream(filename, std::ios::in | std::ios::binary);
    if(!inFile.is_open()){
        std::cout << "Error - Error opening input file\n";
        return nullptr;
    }

    Header *header = new (std::nothrow) Header;
    if(header == nullptr){
        std::cout << "Error - Memory error\n";
        inFile.close();
        return nullptr;
    }

    byte last = inFile.get();
    byte current = inFile.get();
    if(last != 0xFF || current != SOI){
        header->valid = false;
        inFile.close();
        return header;
    }
    last = inFile.get();
    current = inFile.get();
    while(header->valid){
        if(!inFile || last != 0xFF){
            if(!inFile) std::cout << "Error - File ended prematurely\n";
            if(last != 0xFF) std::cout << "Error - Expected a marker\n";
            header->valid = false;
            inFile.close();
            return header;
        }

        if(current == SOF0){
            header->frameType = SOF0;
            readStartOfFrame(inFile, header);
        }else if(current == DQT){
            readQuantizationTable(inFile, header);
        }else if(current == DHT){
            readHuffmanTable(inFile, header);
        }else if (current == SOS){
            readStartOfScan(inFile, header);
            break;
        }else if(current == DRI){
            readRestartInterval(inFile, header);
        }else if(current >= APP0 && current <= APP15){
            readAPPN(inFile, header);
        }else if(current == COM){
            readComment(inFile, header);
        }else if((current >= JPG0 && current <= JPG13) || current == DNL || current == DHP || current == EXP){
            readComment(inFile, header);
        }else if(current == TEM){
            // TEM has no size
        }else if(current == 0xFF){
            current = inFile.get();
            continue;
        }else{
            std::cout << "Error - Unknown marker: 0x" << std::hex << (uint)current << std::dec << "\n";
        }

        last = inFile.get();
        current = inFile.get();
    }
    
    if(header->valid){
        // Read compressed image data
        current = inFile.get();
        while(true){
            if(!inFile){
                std::cout << "Error - File ended prematurely\n";
                header->valid = false;
                inFile.close();
                return header;
            }
            last = current;
            current = inFile.get();
            if(last == 0xFF){
                if(current == EOI){
                    break;
                }else if(current == 0x00){
                    header->huffmanData.push_back(last);
                    current = inFile.get();
                }else if(current >= RST0 && current <= RST7){
                    current = inFile.get();
                }else if(current == 0xFF){
                    continue;
                }else{
                    std::cout << "Error - Invalid marker during compressed data scan: 0x" << std::hex << (uint)current << std::dec << "\n";
                    header->valid = false;
                    inFile.close();
                    return header;       
                }         
            }else{
                header->huffmanData.push_back(last);
            }
        }
    }
    inFile.close();
    return header;
}

void printHeader(const Header *const header){
    if(header == nullptr) return;
    std::cout << "DQT ===========\n";
    for(uint i=0; i<4; i++){
        if(header->quantizationTables[i].set){
            std::cout << "Table ID: " << i << "\n";
            std::cout << "Table Data: ";
            for(uint j=0; j<64; j++){
                if(j % 8 == 0) std::cout << "\n";
                std::cout << header->quantizationTables[i].table[j] << " ";
            }
            std::cout << "\n";
        }
    }

    std::cout << "SOF ===========\n";
    std::cout << "Frame Type: 0x" << std::hex << (uint)header->frameType << std::dec << "\n";
    std::cout << "Height: " << header->height << "\n";
    std::cout << "Width: " << header->width << "\n";
    std::cout << "Color Components:\n";
    for(uint i=0; i<header->numComponents; i++){
        std::cout << "Component ID: " << (i + 1) << "\n";
        std::cout << "Horizontal Sampling Factor: " << (uint)header->colorComponents[i].horizontalSamplingFactor << "\n";
        std::cout << "Vertical Sampling Factor: " << (uint)header->colorComponents[i].verticalSamplingFactor << "\n";
        std::cout << "Quantization Table ID: " << (uint)header->colorComponents[i].quantizationTableID << "\n";
    }
    std::cout << "DHT ===========\n";
    std::cout << "DC Tables:\n";
    for(uint i=0; i<4; i++){
        if(header->huffmanDCTables[i].set){
            std::cout << "Table ID: " << i << "\n";
            std::cout << "Symbols: \n";
            for(uint j=0; j<16; j++){
                std::cout << std::dec << (j+1) << ": ";
                for(uint k=header->huffmanDCTables[i].offsets[j]; k<header->huffmanDCTables[i].offsets[j+1]; k++){
                    std::cout << std::hex << (uint)header->huffmanDCTables[i].symbols[k] << " ";
                }
                std::cout << "\n";
            }
        }
    }
    std::cout << "AC Tables:\n";
    for(uint i=0; i<4; i++){
        if(header->huffmanACTables[i].set){
            std::cout << "Table ID: " << i << "\n";
            std::cout << "Symbols: \n";
            for(uint j=0; j<16; j++){
                std::cout << std::dec << (j+1) << ": ";
                for(uint k=header->huffmanACTables[i].offsets[j]; k<header->huffmanACTables[i].offsets[j+1]; k++){
                    std::cout << std::hex << (uint)header->huffmanACTables[i].symbols[k] << " ";
                }
                std::cout << std::dec << "\n";
            }
        }
    }

    std::cout << "SOS ===========\n";
    std::cout << "Start of Selection: " << (uint)header->startOfSelection << "\n";
    std::cout << "End of Selection: " << (uint)header->endOfSelection << "\n";
    std::cout << "Successive Approximation High: " << (uint)header->successiveApproximationHigh << "\n";
    std::cout << "Successive Approximation Low: " << (uint)header->successiveApproximationLow << "\n";
    std::cout << "Color Components:\n";
    for(uint i=0; i<header->numComponents; i++){
        std::cout << "Component ID: " << (i + 1) << "\n";
        std::cout << "Huffman DC Table ID: " << (uint)header->colorComponents[i].huffmanDCTableID << "\n";
        std::cout << "Huffman AC Table ID: " << (uint)header->colorComponents[i].huffmanACTableID << "\n";
    }
    std::cout << "Length of Huffman Data: " << header->huffmanData.size() << "\n";

    std::cout << "DRI ===========\n";
    std::cout << "Restart Interval: " << header->restartInterval << "\n";
}

void generateCodes(HuffmanTable& hTable){
    uint code = 0;
    for(uint i=0; i<16; i++){
        for(uint j=hTable.offsets[i]; j<hTable.offsets[i+1]; j++){
            hTable.codes[j] = code;
            code += 1;
        }
        code <<= 1;
    }
}

byte getNextSymbol(BitReader& b, const HuffmanTable& hTable, int& symbolSize){
    uint currentCode = 0;
    for(uint i=0; i<16; i++){
        int bit = b.readBit();
        if(bit == -1) {
            return -1;
        }
        currentCode = (currentCode << 1) | bit;
        for(uint j=hTable.offsets[i]; j<hTable.offsets[i+1]; j++){
            if(currentCode == hTable.codes[j]) {
                symbolSize = i;
                return hTable.symbols[j];
            }
        }
    }
    return -1;
}

bool decodeMCUComponent(BitReader& b, int *const component, int& previousDC, const HuffmanTable& dcTable, const HuffmanTable& acTable, int& intervalSize){
    int mcuSize = 0, symbolSize = 0;
    byte length = getNextSymbol(b, dcTable, symbolSize);
    mcuSize += symbolSize;
    if(length == (byte)-1){
        std::cout << "Error - Invalid DC value\n";
        return false;
    }
    if(length > 11){
        std::cout << "Error - DC coefficient length greater than 11\n";
        return false;
    }

    int coeff = b.readBits(length);
    mcuSize += length;
    if(coeff == -1){
        std::cout << "Error - Invalid DC value\n";
        return false;
    }
    if(length != 0 && coeff < (1 << (length - 1))){
        coeff -= (1 << length) - 1;
    }
    component[0] = coeff + previousDC;
    previousDC = component[0];

    uint i = 1;
    while(i < 64){
        byte symbol = getNextSymbol(b, acTable, symbolSize);
        mcuSize += symbolSize;
        if(symbol == (byte)-1){
            std::cout << "Error - Invalid AC value\n";
            return false;
        }
        if(symbol == 0x00){
            intervalSize += mcuSize;
            for(; i<64; i++) component[zigzagMap[i]] = 0;
            return true;
        }

        byte numZeros = symbol >> 4;
        byte coeffLength = symbol & 0x0F;
        coeff = 0;

        // Symbol 0xF0 means skip 16 0's
        if(symbol == 0xF0) numZeros = 16;
        
        if(i + numZeros >= 64){
            std::cout << "Error - Zero run-length exceeded MCU\n";
            return false;
        }else{
            for(uint j=0; j<numZeros; j++, i++){
                component[zigzagMap[i]] = 0;
            }
            if(coeffLength > 10) {
                std::cout << "Error - AC coefficient length greater than 10\n";
                return false;
            }
            if(coeffLength != 0){
                coeff = b.readBits(coeffLength);
                mcuSize += coeffLength;
                if(coeff == -1){
                    std::cout << "Error - Invalid AC value\n";
                    return false;
                }
                if(coeff < (1 << (coeffLength - 1))) coeff -= (1 << coeffLength) - 1;
                component[zigzagMap[i]] = coeff;
                i += 1;
            }
        }
    }

    return true;
};

MCU *decodeHuffmanData(Header *const header){
    const uint mcuHeight = (header->height + 7) / 8;
    const uint mcuWidth = (header->width + 7) / 8;
    int intervalSize = 0;
    MCU *mcus = new (std::nothrow) MCU[mcuHeight * mcuWidth];
    if(mcus == nullptr){
        std::cout << "Error - Memory error\n";
        return nullptr;
    }

    for(uint i=0; i<4; i++){
        if(header->huffmanDCTables[i].set) generateCodes(header->huffmanDCTables[i]);
        if(header->huffmanACTables[i].set) generateCodes(header->huffmanACTables[i]);
    }

    BitReader b(header->huffmanData);

    int previousDCs[3] = { 0 };
    int intervalCnt = 0;

    for(uint i=0; i<mcuHeight * mcuWidth; i++){
        if(header->restartInterval != 0 && i % header->restartInterval == 0){
            // if(i != 0) std::cout << (intervalCnt++) << "th interval size: " << intervalSize << " (bits)\n";
            intervalSize = 0;
            previousDCs[0] = 0;
            previousDCs[1] = 0;
            previousDCs[2] = 0;
            b.align();
        }
        for(uint j=0; j<header->numComponents; j++){
            if(!decodeMCUComponent(b, mcus[i][j], previousDCs[j], header->huffmanDCTables[header->colorComponents[j].huffmanDCTableID], header->huffmanACTables[header->colorComponents[j].huffmanACTableID], intervalSize)){
                delete[] mcus;
                return nullptr;
            }
        }
    }

    //std::cout << (intervalCnt++) << "th interval size: " << intervalSize << " (bits)\n";
    return mcus;
}

void writeBMP(const Header *const header, const MCU *const mcus, const std::string& filename){
    std::ofstream outFile = std::ofstream(filename, std::ios::out | std::ios::binary);
    if(!outFile.is_open()){
        std::cout << "Error - Error opening output file\n";
        return;
    }

    const uint mcuHeight = (header->height + 7) / 8;
    const uint mcuWidth = (header->width + 7) / 8;
    const uint paddingSize = header->width % 4;
    const uint size = 14 + 12 + header->height * header->width * 3 + paddingSize * header->height;

    outFile.put('B');
    outFile.put('M');
    putInt(outFile, size);
    putInt(outFile, 0);
    putInt(outFile, 0x1A);
    putInt(outFile, 12);
    putShort(outFile, header->width);
    putShort(outFile, header->height);
    putShort(outFile, 1);
    putShort(outFile, 24);

    for(uint y=header->height-1; y<header->height; y--){
        const uint mcuRow = y / 8;
        const uint pixelRow = y % 8;
        for(uint x=0; x<header->width; x++){
            const uint mcuColumn = x / 8;
            const uint pixelColumn = x % 8;
            const uint mcuIndex = mcuRow * mcuWidth + mcuColumn;
            const uint pixelIndex = pixelRow * 8 + pixelColumn;
            outFile.put(mcus[mcuIndex].b[pixelIndex]);
            outFile.put(mcus[mcuIndex].g[pixelIndex]);
            outFile.put(mcus[mcuIndex].r[pixelIndex]);
        }
        for(uint i=0; i<paddingSize; i++){
            outFile.put(0);
        }
    }

    outFile.close();
}

void dequantizeMCUComponent(const QuantizationTable& qTable, int *const component){
    for(uint i=0; i<64; i++){
        component[i] *= qTable.table[i];
    }
}

void dequantize(const Header *const header, MCU *const mcus){
    const uint mcuHeight = (header->height + 7) / 8;
    const uint mcuWidth = (header->width + 7) / 8;
    for(uint i=0; i<mcuHeight * mcuWidth; i++){
        for(uint j=0; j<header->numComponents; j++){
            dequantizeMCUComponent(header->quantizationTables[header->colorComponents[j].quantizationTableID], mcus[i][j]);
        }
    }
}

void inverseDCTComponent(const float *const idctMap, int *const component){
    float result[64] = { 0 };
    for(uint i=0; i<8; i++){
        for(uint y=0; y<8; y++){
            float sum = 0.0f;
            for(uint v=0; v<8; v++){
                sum += component[v * 8 + i] * idctMap[v * 8 + y];
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
            component[i * 8 + x] = (int)sum;
        }
    }
}

void inverseDCT(const Header *const header, MCU *const mcus){
    float idctMap[64];
    for(uint u=0; u<8; u++){
        const float c = (u == 0) ? (1.0 / std::sqrt(2.0) / 2.0) : (1.0 / 2.0);
        for(uint x=0; x<8; x++){
            idctMap[u * 8 + x] = c * std::cos((2.0 * x + 1.0) * u * M_PI / 16.0);
        }
    }

    const uint mcuHeight = (header->height + 7) / 8;
    const uint mcuWidth = (header->width + 7) / 8;
    for(uint i=0; i<mcuHeight * mcuWidth; i++){
        for(uint j=0; j<header->numComponents; j++){
            inverseDCTComponent(idctMap, mcus[i][j]);
        }
    }
}

void YCbCrToRGBMCU(MCU& mcu){
    for(int i=0; i<64; i++){
        int r = mcu.y[i] + 1.402f * mcu.cr[i] + 128;
        int g = mcu.y[i] - 0.344f * mcu.cb[i] - 0.714 * mcu.cr[i] + 128;
        int b = mcu.y[i] + 1.772f * mcu.cb[i] + 128;
        if(r < 0) r = 0;
        if(r > 255) r = 255;
        if(g < 0) g = 0;
        if(g > 255) g = 255;
        if(b < 0) b = 0;
        if(b > 255) b = 255;
        mcu.r[i] = r;
        mcu.g[i] = g;
        mcu.b[i] = b;
    }
}

void YCbCrToRGB(const Header *const header, MCU *const mcus){
    const uint mcuHeight = (header->height + 7) / 8;
    const uint mcuWidth = (header->width + 7) / 8;
    for(uint i=0; i<mcuHeight * mcuWidth; i++){
        YCbCrToRGBMCU(mcus[i]);
    }
}

int main(int argc, char *argv[]){
    if(argc < 2){
        std::cout << "Error - Invalid arguments\n";
        return 1;
    }

    for(int i=1; i<argc; i++){
        const std::string filename(argv[i]);
        Header *header = readJPG(filename);
        if(header == nullptr){
            continue;
        }
        if(header->valid == false){
            std::cout << "Error - Invalid JPG\n";
            delete header;
            continue;
        }

        // printHeader(header);
        
        MCU *mcus_host = decodeHuffmanData(header);
        if(mcus_host == nullptr){
            delete header;
            continue;
        }
        dequantize(header, mcus_host);
        inverseDCT(header, mcus_host);
        YCbCrToRGB(header, mcus_host);

        int mcuWidth = (header->width + 7) / 8;
        int mcuHeight = (header->height + 7) / 8;
        MCU *mcus = new (std::nothrow) MCU[mcuHeight * mcuWidth];
    
        // Offloading to DPUs
        try{
            int dpu_num =  1; // ceil((double)mcu_num / (double)header->restartInterval);
            std::cout << "Allocate " << dpu_num << " DPUs\n";
            auto system = DpuSet::allocate(dpu_num);

            // Prepare the buffer
            int cursor = 0;
            std::vector<uint32_t> buffer(BUFFER_SIZE, 0);
            for(int i=0; i<4; i++){
                if(!header->huffmanDCTables[i].set){
                    buffer[cursor++] = FALSE;
                    break;
                }
                buffer[cursor++] = TRUE;
                for(int j=0; j<17; j++) buffer[cursor++] = header->huffmanDCTables[i].offsets[j];
                for(int j=0; j<162; j++) buffer[cursor++] = header->huffmanDCTables[i].symbols[j];      
                for(int j=0; j<162; j++) buffer[cursor++] = header->huffmanDCTables[i].codes[j];
            }

            for(int i=0; i<4; i++){
                if(!header->huffmanACTables[i].set){
                    buffer[cursor++] = FALSE;
                    break;
                }
                buffer[cursor++] = TRUE;
                for(int j=0; j<17; j++) buffer[cursor++] = header->huffmanACTables[i].offsets[j];
                for(int j=0; j<162; j++) buffer[cursor++] = header->huffmanACTables[i].symbols[j];      
                for(int j=0; j<162; j++) buffer[cursor++] = header->huffmanACTables[i].codes[j];
            }
            for(int i=0; i<4; i++){
                if(!header->quantizationTables[i].set){
                    buffer[cursor++] = FALSE;
                    break;
                }
                buffer[cursor++] = TRUE;
                for(int j=0; j<64; j++) buffer[cursor++] = header->quantizationTables[i].table[j];
            }
            buffer[cursor++] = header->width;
            buffer[cursor++] = header->height;
            buffer[cursor++] = (uint)header->numComponents;
            for(int i=0; i<(uint)header->numComponents; i++){
                buffer[cursor++] = header->colorComponents[i].huffmanDCTableID;
                buffer[cursor++] = header->colorComponents[i].huffmanACTableID;
                buffer[cursor++] = header->colorComponents[i].quantizationTableID;
            }
            buffer[cursor++] = header->restartInterval;

            for(int i=0; cursor<BUFFER_SIZE&&i<header->huffmanData.size(); i++)
                buffer[cursor++] = (uint32_t)header->huffmanData[i];

            system.load("./bin/decoder_dpu");
            for(int i=0; i<dpu_num; i++){
                auto dpu = system.dpus()[i];
                dpu->copy("buffer", buffer, static_cast<unsigned>(BUFFER_SIZE));
            }
            system.exec();
            
            std::vector<std::vector<short>> y_r(1, std::vector<short>(64 * 5000, 0));
            std::vector<std::vector<short>> cb_g(1, std::vector<short>(64 * 5000, 0));
            std::vector<std::vector<short>> cr_b(1, std::vector<short>(64 * 5000, 0));
            auto dpu = system.dpus()[0];
            dpu->copy(y_r, "component01");
            dpu->copy(cb_g, "component02");
            dpu->copy(cr_b, "component03");
            for(int i=0; i<mcuHeight*mcuWidth; i++){
              for(int j=0; j<64; j++){
                mcus[i][0][j] = y_r[0][i * 64 + j];
                mcus[i][1][j] = cb_g[0][i * 64 + j];
                mcus[i][2][j] = cr_b[0][i * 64 + j];
              }
            }
            
            system.log(std::cout);
        }catch(const DpuError & e){
                std::cerr << e.what() << "\n";
        }

        // write BMP file
        const std::size_t pos = filename.find_last_of('.');
        const std::string outFilename_dpu = (pos == std::string::npos) ? (filename + "_dpu.bmp") : (filename.substr(0, pos) + "_dpu.bmp");
        const std::string outFilename_host = (pos == std::string::npos) ? (filename + "_host.bmp") : (filename.substr(0, pos) + "_host.bmp");
        writeBMP(header, mcus, outFilename_dpu);
        writeBMP(header, mcus_host, outFilename_host);

        delete[] mcus;
        delete[] mcus_host;
        delete header;
    }

    return 0;
}