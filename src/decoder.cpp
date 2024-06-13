#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <dpu>

#include "jpg.h"
#include "bit_reader.h"

#define DPU_BINARY "./bin/decoder_dpu"

using namespace dpu;

void putInt(std::ofstream& outFile, const uint v);
void putShort(std::ofstream& outFile, const uint v);

void readStartOfScan(std::ifstream& inFile, Header *const header);
void readHuffmanTable(std::ifstream& inFile, Header *const header);
void readStartOfFrame(std::ifstream& inFile, Header *const header);
void readQuantizationTable(std::ifstream& inFile, Header *const header);
void readRestartInterval(std::ifstream& inFile, Header *const header);
void readAPPN(std::ifstream& inFile, Header *const header);
void readComment(std::ifstream& inFile, Header *const header);

Header *readJPG(const std::string& filename);

void generateCodes(HuffmanTable& hTable);
byte getNextSymbol(BitReader& b, const HuffmanTable& hTable);
bool decodeMCUComponent(BitReader& b, int *const component, int& previousDC, const HuffmanTable& dcTable, const HuffmanTable& acTable);
MCU *decodeHuffmanData(Header *const header);
void dequantize(const Header *const header, MCU *const mcus);
void inverseDCTComponent(const float *const idctMap, int *const component);
void inverseDCT(const Header *const header, MCU *const mcus);
void YCbCrToRGB(const Header *const header, MCU *const mcus);

void printHeader(const Header *const header);
void writeBMP(const Header *const header, const MCU *const mcus, const std::string& filename);

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

        MCU *mcusFromHost = decodeHuffmanData(header);
        MCU *mcusFromDPU = decodeHuffmanData(header);
        if(mcusFromHost == nullptr){
            delete header;
            continue;
        }
        dequantize(header, mcusFromHost);
        inverseDCT(header, mcusFromHost);
        YCbCrToRGB(header, mcusFromHost);

        // Offloading to DPUs
        try{
            int mcuLinesPerDPU = header->mcuHeightReal;
            for(; mcuLinesPerDPU*header->mcuWidthReal>MAX_MCU_PER_DPU || header->mcuHeightReal%mcuLinesPerDPU!=0; mcuLinesPerDPU--);
            int dpuNums = header->mcuHeightReal / mcuLinesPerDPU;
            int mcuPerDPU = mcuLinesPerDPU * header->mcuWidthReal;
            auto system = DpuSet::allocate(dpuNums);
            std::cout << dpuNums << " DPUs are allocated.\n";
            std::cout << mcuPerDPU << " MCUs per DPU\n";

            std::vector<uint32_t> mcuNum(4);
            mcuNum[0] = mcuPerDPU;

            std::vector<uint32_t> metaDataBuffer(16);
            metaDataBuffer[0] = header->mcuHeight;
            metaDataBuffer[1] = header->mcuWidth;
            metaDataBuffer[2] = header->mcuHeightReal;
            metaDataBuffer[3] = header->mcuWidthReal;
            metaDataBuffer[4] = header->numComponents;
            metaDataBuffer[5] = header->verticalSamplingFactor;
            metaDataBuffer[6] = header->horizontalSamplingFactor;
            for(uint i=0; i<header->numComponents; i++)
                metaDataBuffer[i + 7] = header->colorComponents[i].quantizationTableID;
            for(uint i=0; i<header->numComponents; i++)
                metaDataBuffer[i + header->numComponents + 7] = header->colorComponents[i].horizontalSamplingFactor;
            for(uint i=0; i<header->numComponents; i++)
                metaDataBuffer[i + (header->numComponents * 2) + 7] = header->colorComponents[i].verticalSamplingFactor;
            std::cout << "Metadata buffer is prepared\n";

            std::vector<uint32_t> quantizationTables(4 * 64);
            for(uint i=0; i<4; i++){
                if(!header->quantizationTables[i].set) break;
                for(uint j=0; j<64; j++){
                    quantizationTables[i * 64 + j] = header->quantizationTables[i].table[j];
                }
            }
            std::cout << "Quantization tables are prepared\n";

            std::vector<short> component01(64 * mcuPerDPU);
            std::vector<short> component02(64 * mcuPerDPU);
            std::vector<short> component03(64 * mcuPerDPU);
            std::cout << "Total MCUs " << (header->mcuHeightReal * header->mcuWidthReal) << "\n";

            system.load(DPU_BINARY);
            for(uint i=0; i<dpuNums; i++){
                for(int j=mcuPerDPU*i; j<mcuPerDPU*i+mcuPerDPU; j++){
                    for(uint k=0; k<64; k++){
                        component01[(j-mcuPerDPU*i) * 64 + k] = mcusFromDPU[j][0][k];
                        component02[(j-mcuPerDPU*i) * 64 + k] = mcusFromDPU[j][1][k];
                        component03[(j-mcuPerDPU*i) * 64 + k] = mcusFromDPU[j][2][k];
                    }
                }
                auto dpu = system.dpus()[i];
                dpu->copy("quantization_tables", quantizationTables, sizeof(uint32_t) * 4 * 64);
                dpu->copy("component01", component01, sizeof(short) * 64 * mcuPerDPU);
                dpu->copy("component02", component02, sizeof(short) * 64 * mcuPerDPU);
                dpu->copy("component03", component03, sizeof(short) * 64 * mcuPerDPU);
                dpu->copy("mcu_num", mcuNum, sizeof(uint32_t) * 4);
                dpu->copy("metadata_buffer", metaDataBuffer, sizeof(uint32_t) * 16);
            }
            system.exec();
            
            std::vector<std::vector<uint32_t>> initialization(1);
            initialization.front().resize(1);
            std::vector<std::vector<uint32_t>> dequantization(1);
            dequantization.front().resize(1);
            std::vector<std::vector<uint32_t>> inverse_dct(1);
            inverse_dct.front().resize(1);
            std::vector<std::vector<uint32_t>> color_space_conversion(1);
            color_space_conversion.front().resize(1);
            std::vector<std::vector<uint32_t>> clocks_per_sec(1);
            clocks_per_sec.front().resize(1);

            auto dpu = system.dpus()[0];
            dpu->copy(clocks_per_sec, "CLOCKS_PER_SEC");
            dpu->copy(initialization, "initialization");
            dpu->copy(dequantization, "dequantization");
            dpu->copy(inverse_dct, "inverse_dct");
            dpu->copy(color_space_conversion, "color_space_conversion");
            std::cout << "Initialization: " << (float)initialization.front().front() / clocks_per_sec.front().front() << "s\n";
            std::cout << "Dequantization: " << (float)dequantization.front().front() / clocks_per_sec.front().front() << "s\n";
            std::cout << "Inverse DCT: " << (float)inverse_dct.front().front() / clocks_per_sec.front().front() << "s\n";
            std::cout << "Color Space Conversion: " << (float)color_space_conversion.front().front() / clocks_per_sec.front().front() << "s\n";
            
            // Load decoded data
            std::vector<std::vector<short>> y_r(1, std::vector<short>(64 * mcuPerDPU, 0));
            std::vector<std::vector<short>> cb_g(1, std::vector<short>(64 * mcuPerDPU, 0));
            std::vector<std::vector<short>> cr_b(1, std::vector<short>(64 * mcuPerDPU, 0));
            for(uint i=0; i<dpuNums; i++){
                dpu = system.dpus()[i];
                dpu->copy(y_r, "component01");
                dpu->copy(cb_g, "component02");
                dpu->copy(cr_b, "component03");
                for(int j=mcuPerDPU*i; j<mcuPerDPU*i+mcuPerDPU; j++){
                    for(int k=0; k<64; k++){
                        mcusFromDPU[j][0][k] = y_r[0][(j - mcuPerDPU*i) * 64 + k];
                        mcusFromDPU[j][1][k] = cb_g[0][(j - mcuPerDPU*i) * 64 + k];
                        mcusFromDPU[j][2][k] = cr_b[0][(j - mcuPerDPU*i) * 64 + k];
                    }
                }
            }
            // system.log(std::cout);
        }catch(const DpuError & e){
                std::cerr << e.what() << "\n";
        }

        // write BMP file
        const std::size_t pos = filename.find_last_of('.');
        writeBMP(header, mcusFromHost, (pos == std::string::npos) ? (filename + "_host.bmp") : (filename.substr(0, pos) + "_host.bmp"));
        writeBMP(header, mcusFromDPU, (pos == std::string::npos) ? (filename + "_dpu.bmp") : (filename.substr(0, pos) + "_dpu.bmp"));

        delete[] mcusFromHost;
        delete[] mcusFromDPU;
        delete header;
    }

    return 0;
}

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
        header->colorComponents[i].usedInScan = false;
    }

    header->componentsInScan = inFile.get();
    if(header->componentsInScan == 0){
        std::cout << "Error - Scan must include at least 1 component\n";
        header->valid = false;
        return;
    }
    for(uint i=0; i<header->componentsInScan; i++){
        byte componentID = inFile.get();
        if(header->zeroBased) componentID += 1;
        if(componentID == 0 || componentID > header->numComponents){
            std::cout << "Error - Invalid color component ID: " << (uint)componentID << "\n";
            header->valid = false;
            return;
        }
        ColorComponent *component = &header->colorComponents[componentID - 1];
        if(!component->usedInFrame){
            std::cout << "Error - Invalid color component ID: " << (uint)componentID << "\n";
            header->valid = false;
            return;
        }
        if(component->usedInScan){
            std::cout << "Duplicate color component ID\n";
            header->valid = false;
            return;            
        }
        component->usedInScan = true;

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

    if(header->frameType == SOF0){
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
    }else if(header->frameType == SOF2){
        if(header->startOfSelection > header->endOfSelection){
            std::cout << "Error - Invalid spectral selection (start greater than end)\n";
            header->valid = false;
            return;
        }
        if(header->endOfSelection > 63){
            std::cout << "Error - Invalid spectral selection (end greater than 63)\n";
            header->valid = false;
            return;
        }
        if(header->startOfSelection == 0 && header->endOfSelection != 0){
            std::cout << "Error - Invalid spectral selection (contains DC and AC)\n";
            header->valid = false;
            return;
        }
        if(header->startOfSelection != 0 && header->componentsInScan != 1){
            std::cout << "Error - Invalid spectral selection (AC scan contains multiple components)\n";
            header->valid = false;
            return;
        }
        if(header->successiveApproximationHigh != 0 &&
           header->successiveApproximationLow != header->successiveApproximationHigh -1){
            std::cout << "Error - Invalid succesive approximation\n";
            header->valid = false;
            return;
        }
    }

    for(uint i=0; i<header->numComponents; i++){
        const ColorComponent& component = header->colorComponents[i];
        if(header->colorComponents[i].usedInScan){
            if(header->quantizationTables[component.quantizationTableID].set == false){
                std::cout << "Error - Color component using uninitialized quantization table\n";
                header->valid = false;
                return;
            }
            if(header->startOfSelection == 0){
                if(header->huffmanDCTables[component.huffmanDCTableID].set == false){
                    std::cout << "Error - Color component using uninitialized Huffman DC table\n";
                    header->valid = false;
                    return;
                }
            }
            if(header->endOfSelection > 0){
                if(header->huffmanACTables[component.huffmanACTableID].set == false){
                    std::cout << "Error - Color component using uninitialized Huffman AC table\n";
                    header->valid = false;
                    return;
                }
            }
        }
    }

    if(length - 6 - (2 * header->componentsInScan) != 0){
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
    header->mcuHeight = (header->height + 7) / 8;
    header->mcuWidth = (header->width + 7) / 8;
    header->mcuHeightReal = header->mcuHeight;
    header->mcuWidthReal = header->mcuWidth;

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
        if(componentID == 0 && i == 0) header->zeroBased = true;
        if(header->zeroBased) componentID += 1;
        if(componentID == 4 || componentID == 5){
            std::cout << "Error - YIQ color mode not supported\n";
            header->valid = false;
            return;
        }
        if(componentID == 0 || componentID > header->numComponents){
            std::cout << "Error - Invalid component ID:" << (uint)componentID << "\n";
            header->valid = false;
            return;
        }
        ColorComponent *component = &header->colorComponents[componentID - 1];
        if(component->usedInFrame){
            std::cout << "Duplicate color component ID\n";
            header->valid = false;
            return;
        }
        component->usedInFrame = true;
        byte samplingFactor = inFile.get();
        component->horizontalSamplingFactor = samplingFactor >> 4;
        component->verticalSamplingFactor = samplingFactor & 0x0F;
        if(componentID == 1){
            if((component->horizontalSamplingFactor != 1 && component->horizontalSamplingFactor != 2) ||
               (component->verticalSamplingFactor != 1 && component->verticalSamplingFactor != 2)){
                std::cout << "Error - Sampling factors not supported\n";
                header->valid = false;
                return;
            }
            if(component->horizontalSamplingFactor == 2 && header->mcuWidth % 2 == 1){
                header->mcuWidthReal += 1;
            }
            if(component->verticalSamplingFactor == 2 && header->mcuHeight % 2 == 1){
                header->mcuHeightReal += 1;
            }
            header->horizontalSamplingFactor = component->horizontalSamplingFactor;
            header->verticalSamplingFactor = component->verticalSamplingFactor;
        }else{
            if(component->horizontalSamplingFactor != 1 || component->verticalSamplingFactor != 1){
                std::cout << "Error - Sampling factors not supported\n";
                header->valid = false;
                return;
            }
        }

        std::cout << "vertical sampling factor: " << (uint)header->verticalSamplingFactor << ", horizontal sampling factor: " << (uint)header->horizontalSamplingFactor << "\n";
        
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

        if(current == SOF0 || current == SOF2){
            header->frameType = current;
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

byte getNextSymbol(BitReader& b, const HuffmanTable& hTable){
    uint currentCode = 0;
    for(uint i=0; i<16; i++){
        int bit = b.readBit();
        if(bit == -1) {
            return -1;
        }
        currentCode = (currentCode << 1) | bit;
        for(uint j=hTable.offsets[i]; j<hTable.offsets[i+1]; j++){
            if(currentCode == hTable.codes[j]) {
                return hTable.symbols[j];
            }
        }
    }
    return -1;
}

bool decodeMCUComponent(BitReader& b, int *const component, int& previousDC, const HuffmanTable& dcTable, const HuffmanTable& acTable){
    byte length = getNextSymbol(b, dcTable);
    if(length == (byte)-1){
        std::cout << "Error - Invalid DC value\n";
        return false;
    }
    if(length > 11){
        std::cout << "Error - DC coefficient length greater than 11\n";
        return false;
    }

    int coeff = b.readBits(length);
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
        byte symbol = getNextSymbol(b, acTable);
        if(symbol == (byte)-1){
            std::cout << "Error - Invalid AC value\n";
            return false;
        }
        if(symbol == 0x00){
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
    MCU *mcus = new (std::nothrow) MCU[header->mcuHeightReal * header->mcuWidthReal];
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

    for(uint y=0; y<header->mcuHeight; y += header->verticalSamplingFactor){
      for(uint x=0; x<header->mcuWidth; x += header->horizontalSamplingFactor){
        if(header->restartInterval != 0 && (y * header->mcuWidthReal + x) % header->restartInterval == 0){
            previousDCs[0] = 0;
            previousDCs[1] = 0;
            previousDCs[2] = 0;
            b.align();
        }
        for(uint j=0; j<header->numComponents; j++){
          for(uint v=0; v<header->colorComponents[j].verticalSamplingFactor; v++){
            for(uint h=0; h<header->colorComponents[j].horizontalSamplingFactor; h++){
              if(!decodeMCUComponent(
                b, 
                mcus[(y + v) * header->mcuWidthReal + (x + h)][j], 
                previousDCs[j], 
                header->huffmanDCTables[header->colorComponents[j].huffmanDCTableID], 
                header->huffmanACTables[header->colorComponents[j].huffmanACTableID])){
                delete[] mcus;
                return nullptr;
              }
            }
          }
        }
      }
    }

    return mcus;
}

void dequantize(const Header *const header, MCU *const mcus){
  for(uint y=0; y<header->mcuHeight; y += header->verticalSamplingFactor){
    for(uint x=0; x<header->mcuWidth; x += header->horizontalSamplingFactor){
      for(uint i=0; i<header->numComponents; i++){
        for(uint v=0; v<header->colorComponents[i].verticalSamplingFactor; v++){
          for(uint h=0; h<header->colorComponents[i].horizontalSamplingFactor; h++){
            for(uint k=0; k<64; k++){
                mcus[(y + v) * header->mcuWidthReal + (x + h)][i][k] *= header->quantizationTables[header->colorComponents[i].quantizationTableID].table[k];
            }
          }
        }
      }
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

    for(uint y=0; y<header->mcuHeight; y+=header->verticalSamplingFactor){
      for(uint x=0; x<header->mcuWidth; x+=header->horizontalSamplingFactor){
        for(uint i=0; i<header->numComponents; i++){
          for(uint v=0; v<header->colorComponents[i].verticalSamplingFactor; v++){
            for(uint h=0; h<header->colorComponents[i].horizontalSamplingFactor; h++){
              inverseDCTComponent(idctMap, mcus[(y + v) * header->mcuWidthReal + (x + h)][i]);
            }
          }
        }
      }
    }
}

void YCbCrToRGBMCU(const Header *const header, MCU& mcu, const MCU& cbcr, const uint v, const uint h){
    for(uint y=7; y<8; y--){
        for(uint x=7; x<8; x--){
            const uint pixel = y * 8 + x;
            const uint cbcrPixelRow = y / header->verticalSamplingFactor + 4 * v;
            const uint cbcrPixelColumn = x / header->horizontalSamplingFactor + 4 * h;
            const uint cbcrPixel = cbcrPixelRow * 8 + cbcrPixelColumn;
            int r = mcu.y[pixel] + 1.402f * cbcr.cr[cbcrPixel] + 128;
            int g = mcu.y[pixel] - 0.344f * cbcr.cb[cbcrPixel] - 0.714 * cbcr.cr[cbcrPixel] + 128;
            int b = mcu.y[pixel] + 1.772f * cbcr.cb[cbcrPixel] + 128;
            if(r < 0) r = 0;
            if(r > 255) r = 255;
            if(g < 0) g = 0;
            if(g > 255) g = 255;
            if(b < 0) b = 0;
            if(b > 255) b = 255;
            mcu.r[pixel] = r;
            mcu.g[pixel] = g;
            mcu.b[pixel] = b;
        }
    }
}

void YCbCrToRGB(const Header *const header, MCU *const mcus){
    for(uint y=0; y<header->mcuHeight; y+=header->verticalSamplingFactor){
        for(uint x=0; x<header->mcuWidth; x+=header->horizontalSamplingFactor){
            const MCU& cbcr = mcus[y * header->mcuWidthReal + x];
            for(uint v=header->verticalSamplingFactor-1; v<header->verticalSamplingFactor; v--){
                for(uint h=header->horizontalSamplingFactor-1; h<header->horizontalSamplingFactor; h--){
                    YCbCrToRGBMCU(header, mcus[(y + v) * header->mcuWidthReal + (x + h)], cbcr, v, h);
                }
            }
        }
    }
}

void writeBMP(const Header *const header, const MCU *const mcus, const std::string& filename){
    std::ofstream outFile = std::ofstream(filename, std::ios::out | std::ios::binary);
    if(!outFile.is_open()){
        std::cout << "Error - Error opening output file\n";
        return;
    }

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
            const uint mcuIndex = mcuRow * header->mcuWidthReal + mcuColumn;
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