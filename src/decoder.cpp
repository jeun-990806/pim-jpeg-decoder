#include <iostream>
#include <iomanip>
#include <fstream>
#include <map>
#include <algorithm>

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <cmath>
#include <chrono>

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

void printHeader(const Header *const header);
void writeBMP(const Header *const header, const MCU *const mcus, const std::string& filename);

std::queue<JPEG> JPEG_queue;
std::mutex mtx;
std::condition_variable cv;
bool finished = false;

auto pim = DpuSet::allocate(DPU_ALLOCATE_ALL);
uint nr_dpus = pim.dpus().size();

int max_mcu_buffer_size = nr_dpus * MAX_MCU_PER_DPU;

std::vector<std::map<std::string, std::chrono::duration<double>>> execution_times;
std::chrono::duration<double> mcu_prepare_time;

int get_mcus_in_queue(){
    int total_mcus = 0;
    std::queue<JPEG> temp_queue = JPEG_queue;
    while(!temp_queue.empty()){
        total_mcus += temp_queue.front().header->mcuWidthReal * temp_queue.front().header->mcuHeightReal;
        temp_queue.pop();
    }
    return total_mcus;
}

void mcu_prepare(const std::vector<std::string>& input_files){

    auto mcu_prepare_start = std::chrono::high_resolution_clock::now();

    for(const auto& filename : input_files){
        JPEG file;

        Header *header = readJPG(filename);
        if(header == nullptr || header->valid == false){
            std::cout << filename << ": Error - Invalid JPEG\n";
        }else{
            std::cout << filename << ": Loaded\n";
        }

        if(get_mcus_in_queue() >= max_mcu_buffer_size || JPEG_queue.size() >= nr_dpus) cv.notify_one();

        MCU *mcus = decodeHuffmanData(header);
        if(mcus == nullptr){
            std::cout << filename << ": Error - Huffman Decoding\n";
            continue;
        }

        file.filename = filename;
        file.header = header;
        file.mcus = mcus;
        
        {
            std::lock_guard<std::mutex> lock(mtx);
            JPEG_queue.push(file);
        }
    }

    auto mcu_prepare_end = std::chrono::high_resolution_clock::now();
    mcu_prepare_time = mcu_prepare_end - mcu_prepare_start;

    {
        std::lock_guard<std::mutex> lock(mtx);
        finished = true;
    }
    cv.notify_one();
}

void offloading(){
    while(true){
        std::map<std::string, std::chrono::duration<double>> execution_time;
        std::unique_lock<std::mutex> lock(mtx);

        auto wait_start = std::chrono::high_resolution_clock::now();
        cv.wait(lock, [] { return get_mcus_in_queue() >= max_mcu_buffer_size || JPEG_queue.size() >= nr_dpus || finished; });
        auto wait_end = std::chrono::high_resolution_clock::now();
        execution_time["wait_time"] = wait_end - wait_start;

        if(!JPEG_queue.empty()){
            std::vector<JPEG> JPEG_batch;
            std::vector<int> allocated_dpus_info;
            std::vector<int> mcus_per_dpu_info;
            int mcus_in_batch = 0;

            std::cout << "Batch =============================\n";
            auto batch_start = std::chrono::high_resolution_clock::now();
            while(!JPEG_queue.empty()){
                JPEG file = JPEG_queue.front();
                JPEG_batch.push_back(file);
                mcus_in_batch += file.header->mcuWidthReal * file.header->mcuHeightReal;
                JPEG_queue.pop();
            }
            auto batch_end = std::chrono::high_resolution_clock::now();
            execution_time["batch_time"] = batch_end - batch_start;
        
            lock.unlock();
            pim.load(DPU_BINARY);
            uint free_dpus = nr_dpus;
            int mcus_at_once = mcus_in_batch / nr_dpus;
            int start_dpu_num = 0;

            auto cpu_to_dpus_start = std::chrono::high_resolution_clock::now();
            for(const auto& file : JPEG_batch){
                int allocated_dpus = std::min(file.header->mcuWidthReal * file.header->mcuHeightReal / mcus_at_once, free_dpus);
                if(allocated_dpus <= 1) allocated_dpus = 1;
                else allocated_dpus -= 1;
                int mcus_per_dpu = (file.header->mcuWidthReal * file.header->mcuHeightReal) / allocated_dpus;
                if((file.header->mcuWidthReal * file.header->mcuHeightReal) % allocated_dpus != 0) allocated_dpus += 1;

                std::cout << file.filename << ": Use " << allocated_dpus << " dpus (" << mcus_per_dpu << " MCUs per DPU)\n";
                allocated_dpus_info.push_back(allocated_dpus);
                free_dpus -= allocated_dpus;
                mcus_per_dpu_info.push_back(mcus_per_dpu);

                std::vector<uint32_t> mcu_num(4);
                mcu_num[0] = mcus_per_dpu;
                    
                std::vector<uint32_t> metadata_buffer(16);
                metadata_buffer[0] = file.header->mcuHeight;
                metadata_buffer[1] = file.header->mcuWidth;
                metadata_buffer[2] = file.header->mcuHeightReal;
                metadata_buffer[3] = file.header->mcuWidthReal;
                metadata_buffer[4] = file.header->numComponents;
                metadata_buffer[5] = file.header->verticalSamplingFactor;
                metadata_buffer[6] = file.header->horizontalSamplingFactor;
                for(uint j=0; j<file.header->numComponents; j++)
                    metadata_buffer[j + 7] = file.header->colorComponents[j].quantizationTableID;
                for(uint j=0; j<file.header->numComponents; j++)
                    metadata_buffer[j + file.header->numComponents + 7] = file.header->colorComponents[j].horizontalSamplingFactor;
                for(uint j=0; j<file.header->numComponents; j++)
                    metadata_buffer[j + (file.header->numComponents * 2) + 7] = file.header->colorComponents[j].verticalSamplingFactor;

                std::vector<uint32_t> quantization_tables(4 * 64);
                for(uint j=0; j<4; j++){
                    if(!file.header->quantizationTables[j].set) break;
                    for(uint k=0; k<64; k++){
                        quantization_tables[j * 64 + k] = file.header->quantizationTables[j].table[k];
                    }
                }

                std::vector<short> component01(64 * mcus_per_dpu);
                std::vector<short> component02(64 * mcus_per_dpu);
                std::vector<short> component03(64 * mcus_per_dpu);

                for(uint j=0; j<allocated_dpus; j++){
                    for(int k=mcus_per_dpu*j; k<mcus_per_dpu*(j+1) && k<file.header->mcuWidthReal*file.header->mcuHeightReal; k++){
                        for(uint l=0; l<64; l++){
                            component01[(k-mcus_per_dpu*j) * 64 + l] = file.mcus[k][0][l];
                            component02[(k-mcus_per_dpu*j) * 64 + l] = file.mcus[k][1][l];
                            component03[(k-mcus_per_dpu*j) * 64 + l] = file.mcus[k][2][l];
                        }
                    }
                    auto dpu = pim.dpus()[j + start_dpu_num];
                    dpu->copy("quantization_tables", quantization_tables, sizeof(uint32_t) * 4 * 64);
                    dpu->copy("component01", component01, sizeof(short) * 64 * mcus_per_dpu);
                    dpu->copy("component02", component02, sizeof(short) * 64 * mcus_per_dpu);
                    dpu->copy("component03", component03, sizeof(short) * 64 * mcus_per_dpu);
                    dpu->copy("mcu_num", mcu_num, sizeof(uint32_t) * 4);
                    dpu->copy("metadata_buffer", metadata_buffer, sizeof(uint32_t) * 16);
                }

                start_dpu_num += allocated_dpus;
            
            }
            auto cpu_to_dpus_end = std::chrono::high_resolution_clock::now();
            execution_time["cpu_to_dpus_time"] = cpu_to_dpus_end - cpu_to_dpus_start;

            auto dpu_execution_start = std::chrono::high_resolution_clock::now();
            pim.exec();
            auto dpu_execution_end = std::chrono::high_resolution_clock::now();
            execution_time["dpu_execution_time"] = dpu_execution_end - dpu_execution_start;

            std::cout << "Decoded ===========================\n";
            start_dpu_num = 0;
            auto dpus_to_cpu_start = std::chrono::high_resolution_clock::now();
            for(const auto& file : JPEG_batch){
                int allocated_dpus = allocated_dpus_info.front();
                int mcus_per_dpu = mcus_per_dpu_info.front();
                allocated_dpus_info.erase(allocated_dpus_info.begin());
                mcus_per_dpu_info.erase(mcus_per_dpu_info.begin());

                std::vector<std::vector<short>> y_r(1, std::vector<short>(64 * mcus_per_dpu, 0));
                std::vector<std::vector<short>> cb_g(1, std::vector<short>(64 * mcus_per_dpu, 0));
                std::vector<std::vector<short>> cr_b(1, std::vector<short>(64 * mcus_per_dpu, 0));
                for(uint j=0; j<allocated_dpus; j++){
                    auto dpu = pim.dpus()[j + start_dpu_num];
                    dpu->copy(y_r, "component01");
                    dpu->copy(cb_g, "component02");
                    dpu->copy(cr_b, "component03");
                    for(int k=mcus_per_dpu*j; k<mcus_per_dpu*(j+1) && k<file.header->mcuWidthReal * file.header->mcuHeightReal; k++){
                        for(int l=0; l<64; l++){
                            file.mcus[k][0][l] = y_r[0][(k-mcus_per_dpu*j) * 64 + l];
                            file.mcus[k][1][l] = cb_g[0][(k-mcus_per_dpu*j) * 64 + l];
                            file.mcus[k][2][l] = cr_b[0][(k-mcus_per_dpu*j) * 64 + l];
                        }
                    }
                }
                start_dpu_num += allocated_dpus;
            }
            auto dpus_to_cpu_end = std::chrono::high_resolution_clock::now();
            execution_time["dpus_to_cpu_time"] = dpus_to_cpu_end - dpus_to_cpu_start;

            auto bmp_write_start = std::chrono::high_resolution_clock::now();
            for(const auto& file : JPEG_batch){
                const std::size_t pos = file.filename.find_last_of('.');
                writeBMP(file.header, file.mcus, (pos == std::string::npos) ? (file.filename + ".bmp") : (file.filename.substr(0, pos) + ".bmp"));

                std::cout << file.filename << ": Saved (" << (file.filename.substr(0, pos) + ".bmp") << ")\n";
            }
            auto bmp_write_end = std::chrono::high_resolution_clock::now();
            execution_time["bmp_write_time"] = bmp_write_end - bmp_write_start;
            std::cout << "===================================\n";
            
            execution_times.push_back(execution_time);
        }else if(finished) break;
    }
}

std::chrono::duration<double> sum_of_map(const std::map<std::string, std::chrono::duration<double>>& m) {
    std::chrono::duration<double> sum = static_cast<std::chrono::duration<double>>(0);
    for (const auto& pair : m) {
        sum += pair.second;
    }
    return sum;
}

int main(int argc, char *argv[]){
    if(argc < 2){
        std::cout << "Error - Invalid arguments\n";
        return 1;
    }

    std::vector<std::string> input_files;
    for(int i=1; i<argc; i++) input_files.push_back(argv[i]);
    
    std::cout << nr_dpus << " dpus are allocated\n";
    int max_mcu_window = nr_dpus * MAX_MCU_PER_DPU;

    auto start = std::chrono::high_resolution_clock::now();

    std::thread mcu_preparer(mcu_prepare, std::ref(input_files));
    std::thread decoding_offloader(offloading);

    mcu_preparer.join();
    decoding_offloader.join();

    auto end = std::chrono::high_resolution_clock::now();

    auto compare = [](const std::map<std::string, std::chrono::duration<double>>& a, const std::map<std::string, std::chrono::duration<double>>& b) {
        return sum_of_map(a) < sum_of_map(b);
    };
    std::sort(execution_times.begin(), execution_times.end(), compare);

    std::cout << "\nProfiles:\n";

    std::chrono::duration<double> execution_time = end - start;
    std::cout << "End-to-end execution time: " << execution_time.count() << "s\n";

    std::cout << "MCU Preparer execution time (total): " << mcu_prepare_time.count() << "s\n";

    std::cout << "MCU Offloader execution time (median): " << sum_of_map(execution_times[execution_times.size()/2]).count() << "s\n";
    std::cout << " - Queue waiting time: " << execution_times[execution_times.size()/2]["wait_time"].count() << "s\n";
    std::cout << " - CPU-to-DPUs transfer time: " << execution_times[execution_times.size()/2]["cpu_to_dpus_time"].count() << "s\n";
    std::cout << " - DPU execution time: " << execution_times[execution_times.size()/2]["dpu_execution_time"].count() << "s\n";
    std::cout << " - DPUs-to-CPU transfer time: " << execution_times[execution_times.size()/2]["dpus_to_cpu_time"].count() << "s\n";
    std::cout << " - BMP write time: " << execution_times[execution_times.size()/2]["bmp_write_time"].count() << "s\n";

    if(execution_times.size() > 1){
        std::cout << "MCU Offloader execution time (maximum): " << sum_of_map(execution_times[execution_times.size()-1]).count() << "s\n";
        std::cout << " - Queue waiting time: " << execution_times[execution_times.size()-1]["wait_time"].count() << "s\n";
        std::cout << " - CPU-to-DPUs transfer time: " << execution_times[execution_times.size()-1]["cpu_to_dpus_time"].count() << "s\n";
        std::cout << " - DPU execution time: " << execution_times[execution_times.size()-1]["dpu_execution_time"].count() << "s\n";
        std::cout << " - DPUs-to-CPU transfer time: " << execution_times[execution_times.size()-1]["dpus_to_cpu_time"].count() << "s\n";
        std::cout << " - BMP write time: " << execution_times[execution_times.size()-1]["bmp_write_time"].count() << "s\n";

        std::cout << "MCU Offloader execution time (minimum): " << sum_of_map(execution_times[0]).count() << "s\n";
        std::cout << " - Queue waiting time: " << execution_times[0]["wait_time"].count() << "s\n";
        std::cout << " - CPU-to-DPUs transfer time: " << execution_times[0]["cpu_to_dpus_time"].count() << "s\n";
        std::cout << " - DPU execution time: " << execution_times[0]["dpu_execution_time"].count() << "s\n";
        std::cout << " - DPUs-to-CPU transfer time: " << execution_times[0]["dpus_to_cpu_time"].count() << "s\n";
        std::cout << " - BMP write time: " << execution_times[0]["bmp_write_time"].count() << "s\n";
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
    uint length = (inFile.get() << 8) + inFile.get();

    header->restartInterval = (inFile.get() << 8) + inFile.get();
    if(length - 4 != 0){
        std::cout << "Error - DRI invalid\n";
        header->valid = false;
    }
}

void readAPPN(std::ifstream& inFile, Header *const header){
    uint length = (inFile.get() << 8) + inFile.get();

    for(uint i=0; i<length-2; ++i) inFile.get();
}

void readComment(std::ifstream& inFile, Header *const header){
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