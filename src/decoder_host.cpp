#include <iostream>
#include <iomanip>
#include <fstream>
#include <map>
#include <algorithm>

#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>

#include <sys/stat.h>

#include <time.h>

#include <dpu>

#include "headers/jpeg.h"
#include "headers/bmp.h"

#define DPU_BINARY "./bin/decoder_dpu"

using namespace dpu;

struct Batch {
    std::vector<std::string> filenames;
    std::vector<std::vector<short>> mcus;
    std::vector<std::vector<uint32_t>> metadata;
    std::vector<int> nr_allocated_dpus;
};

auto pim = DpuSet::allocate(DPU_ALLOCATE_ALL);
uint nr_dpus = pim.dpus().size();

std::queue<Batch> batched_queue;
std::mutex mtx;
std::condition_variable cv;
bool finished = false;

uintmax_t get_filesize(const std::string& file_path) {
    struct stat stat_buf;
    stat(file_path.c_str(), &stat_buf);
    return stat_buf.st_size;
}

void sort_by_size(std::vector<std::string>& image_paths) {
    std::vector<std::pair<uintmax_t, std::string>> images;

    for (const auto& path : image_paths) {
        uintmax_t filesize = get_filesize(path);
        images.emplace_back(filesize, path);
    }

    std::sort(images.begin(), images.end(), [](std::pair<uintmax_t, std::string>& a, std::pair<uintmax_t, std::string>& b) {
        return a.first < b.first;
    });

    for (size_t i = 0; i < images.size(); ++i) {
        image_paths[i] = images[i].second;
    }
}

/* Use chrono for profiling
std::chrono::duration<double> total_execution_time;
std::chrono::system_clock::time_point execution_start, execution_end;
int total_offloading_times = 0;

#ifdef DEBUG
    std::vector<std::map<std::string, std::chrono::duration<double>>> execution_times;
    std::chrono::duration<double> mcu_prepare_time;

    std::chrono::duration<double> sum_of_map(const std::map<std::string, std::chrono::duration<double>>& m) {
        std::chrono::duration<double> sum = static_cast<std::chrono::duration<double>>(0);
        for (const auto& pair : m) {
            sum += pair.second;
        }
        return sum;
    }
#endif
*/

std::vector<std::vector<uint32_t>> init_cycles(1, std::vector<uint32_t>(1));
std::vector<std::vector<uint32_t>> dequantization_cycles(1, std::vector<uint32_t>(1));
std::vector<std::vector<uint32_t>> idct_cycles(1, std::vector<uint32_t>(1));
std::vector<std::vector<uint32_t>> color_space_conversion_cycles(1, std::vector<uint32_t>(1));

double total_execution_time = 0;
double mcu_prepare_time = 0;
double total_queue_waiting_time = 0;
double total_batch_time = 0;
double total_cpu_to_dpus_transfer_time = 0;
double total_dpu_execution_time = 0;
double total_dpus_to_cpu_transfer_time = 0;
double total_bmp_write_time = 0;
int total_offloading_times = 0;

double get_time(struct timespec *start, struct timespec *end){
    return (end->tv_sec - start->tv_sec) + ((end->tv_nsec - start->tv_nsec) / 1e9);
}

void mcu_prepare(const std::vector<std::string>& input_files){
    /* Use chrono for profiling
    #ifdef DEBUG
        auto mcu_prepare_start = std::chrono::high_resolution_clock::now();
    #endif
    */
    struct timespec execution_start, execution_end;

    clock_gettime(CLOCK_MONOTONIC, &execution_start);
    std::vector<std::vector<short>> MCU_buffer(nr_dpus, std::vector<short>(MAX_MCU_PER_DPU * 3 * 64));
    std::vector<std::vector<uint32_t>> metadata_buffer;
    std::vector<uint32_t> metadata(20 + 4 * 64);
    std::vector<std::string> filenames;
    std::vector<int> nr_allocated_dpus;

    int dpu_offset = 0, need_dpus = 0, free_dpus = nr_dpus;

    for(const auto& filename : input_files){
        Header *header = read_JPEG(filename);
        if(header == nullptr || header->valid == false){
            std::cout << filename << ": Error - Invalid JPEG\n";
            continue;
        }

        int padded_mcu_width = (header->mcu_width_real + 1) / 2 * 2;
        int padded_mcu_height = (header->mcu_height_real + 1) / 2 * 2;
        int total_padded_mcu_count = padded_mcu_width * padded_mcu_height;
        need_dpus = (total_padded_mcu_count + MAX_MCU_PER_DPU - 1) / MAX_MCU_PER_DPU;

        if(need_dpus > free_dpus) {
            metadata_buffer.resize(nr_dpus, std::vector<uint32_t>(20 + 4 * 64));
            {
                std::lock_guard<std::mutex> lock(mtx);
                batched_queue.push({filenames, MCU_buffer, metadata_buffer, nr_allocated_dpus});
            }
            cv.notify_one();
            
            MCU_buffer = std::vector<std::vector<short>>(nr_dpus, std::vector<short>(MAX_MCU_PER_DPU * 3 * 64));
            metadata_buffer.clear();
            filenames.clear();
            nr_allocated_dpus.clear();
            dpu_offset = 0;
            free_dpus = nr_dpus;
        }

        if(need_dpus > nr_dpus){
            std::cout << filename << ": Error - Too high resolution\n";
            continue;
        }

        free_dpus -= need_dpus;

        filenames.push_back(filename);
        nr_allocated_dpus.push_back(need_dpus);

        metadata[0] = header->mcu_height;
        metadata[1] = header->mcu_width;
        metadata[2] = header->mcu_height_real;
        metadata[3] = header->mcu_width_real;
        metadata[4] = header->num_components;
        metadata[5] = header->v_sampling_factor;
        metadata[6] = header->h_sampling_factor;

        for(uint j=0; j<header->num_components; j++)
            metadata[j + 7] = header->color_components[j].QT_ID;
        for(uint j=0; j<header->num_components; j++)
            metadata[j + header->num_components + 7] = header->color_components[j].h_sampling_factor;
        for(uint j=0; j<header->num_components; j++)
            metadata[j + (header->num_components * 2) + 7] = header->color_components[j].v_sampling_factor;
        metadata[17] = header->height;
        metadata[18] = header->width;
        metadata[19] = MAX_MCU_PER_DPU;
        for(uint j=0; j<4; j++){
            if(!header->quantization_tables[j].set) break;
                for(uint k=0; k<64; k++){
                    metadata[20 + j * 64 + k] = header->quantization_tables[j].table[k];
                }
        }
        metadata_buffer.resize(metadata_buffer.size() + need_dpus, metadata);

        decode_Huffman_data(header, MCU_buffer, dpu_offset);

        dpu_offset += need_dpus;
    }

    if(dpu_offset > 0) {
        nr_allocated_dpus.push_back(dpu_offset);
        metadata_buffer.resize(nr_dpus, std::vector<uint32_t>(20 + 4 * 64));
        {
            std::lock_guard<std::mutex> lock(mtx);
            batched_queue.push({filenames, MCU_buffer, metadata_buffer, nr_allocated_dpus});
        }
        cv.notify_one();
    }
    
    {
        std::lock_guard<std::mutex> lock(mtx);
        finished = true;
    }
    cv.notify_one();

    clock_gettime(CLOCK_MONOTONIC, &execution_end);
    mcu_prepare_time = get_time(&execution_start, &execution_end);

    /* Use chrono for profiling
    #ifdef DEBUG
        auto mcu_prepare_end = std::chrono::high_resolution_clock::now();
        mcu_prepare_time = mcu_prepare_end - mcu_prepare_start;
    #endif
    */
}

void offloading(){
    /* Use chrono for profiling
    #ifdef DEBUG
        std::map<std::string, std::chrono::duration<double>> execution_time_breakdown;
    #endif
    execution_start = std::chrono::high_resolution_clock::now();
    */

    struct timespec execution_start, execution_end;
    struct timespec tmp_start, tmp_end;

    clock_gettime(CLOCK_MONOTONIC, &execution_start);
    while(true){
        Batch batch;
        {
            std::unique_lock<std::mutex> lock(mtx);
            
            /* Use chrono for profiling
            #ifdef DEBUG
                auto queue_waiting_start = std::chrono::high_resolution_clock::now();
            #endif
            */
            clock_gettime(CLOCK_MONOTONIC, &tmp_start);
            cv.wait(lock, []{ return !batched_queue.empty() || finished; });
            clock_gettime(CLOCK_MONOTONIC, &tmp_end);
            total_queue_waiting_time += get_time(&tmp_start, &tmp_end);
            /* Use chrono for profiling
            #ifdef DEBUG
                auto queue_waiting_end = std::chrono::high_resolution_clock::now();
                execution_time_breakdown["queue_wait_time"] = queue_waiting_end - queue_waiting_start;
            #endif
            */

            if (batched_queue.empty() && finished) {
                break; 
            }

            /* Use chrono for profiling
            #ifdef DEBUG
                auto batch_start = std::chrono::high_resolution_clock::now();
            #endif
            */
            clock_gettime(CLOCK_MONOTONIC, &tmp_start);
            batch = std::move(batched_queue.front());
            batched_queue.pop();
            clock_gettime(CLOCK_MONOTONIC, &tmp_end);
            total_batch_time += get_time(&tmp_start, &tmp_end);
            /* Use chrono for profiling
            #ifdef DEBUG
                auto batch_end = std::chrono::high_resolution_clock::now();
                execution_time_breakdown["batch_time"] = batch_end - batch_start;
            #endif
            */
        }

        pim.load(DPU_BINARY);

        /* Use chrono for profiling
        #ifdef DEBUG
            auto cpu_to_dpus_start = std::chrono::high_resolution_clock::now();
        #endif
        */
        clock_gettime(CLOCK_MONOTONIC, &tmp_start);
        pim.copy("metadata_buffer", batch.metadata);
        pim.copy("mcus", batch.mcus);
        clock_gettime(CLOCK_MONOTONIC, &tmp_end);
        total_cpu_to_dpus_transfer_time += get_time(&tmp_start, &tmp_end);

        /* Use chrono for profiling
        #ifdef DEBUG
            auto cpu_to_dpus_end = std::chrono::high_resolution_clock::now();
            execution_time_breakdown["cpu_to_dpus_time"] = cpu_to_dpus_end - cpu_to_dpus_start;
        #endif

        #ifdef DEBUG
            auto dpu_execution_start = std::chrono::high_resolution_clock::now();
        #endif
        */
        clock_gettime(CLOCK_MONOTONIC, &tmp_start);
        pim.exec();
        clock_gettime(CLOCK_MONOTONIC, &tmp_end);
        total_dpu_execution_time += get_time(&tmp_start, &tmp_end);
        total_offloading_times += 1;
        /* Use chrono for profiling
        #ifdef DEBUG
            auto dpu_execution_end = std::chrono::high_resolution_clock::now();
            execution_time_breakdown["dpu_execution_time"] = dpu_execution_end - dpu_execution_start;
            total_offloading_times += 1;
        #endif

        #ifdef DEBUG
            auto dpus_to_cpu_start = std::chrono::high_resolution_clock::now();
        #endif
        */
        clock_gettime(CLOCK_MONOTONIC, &tmp_start);
        pim.copy(batch.mcus, "mcus");
        pim.dpus()[0]->copy(init_cycles, "initialization");
        pim.dpus()[0]->copy(dequantization_cycles, "dequantization");
        pim.dpus()[0]->copy(idct_cycles, "inverse_dct");
        pim.dpus()[0]->copy(color_space_conversion_cycles, "color_space_conversion");
        clock_gettime(CLOCK_MONOTONIC, &tmp_end);
        total_dpus_to_cpu_transfer_time += get_time(&tmp_start, &tmp_end);
        /* Use chrono for profiling
        #ifdef DEBUG
            auto dpus_to_cpu_end = std::chrono::high_resolution_clock::now();
            execution_time_breakdown["dpus_to_cpu_time"] = dpus_to_cpu_end - dpus_to_cpu_start;
        #endif
        
        #ifdef DEBUG
            auto bmp_write_start = std::chrono::high_resolution_clock::now();
        #endif
        */
        clock_gettime(CLOCK_MONOTONIC, &tmp_start);
        int dpu_offset = 0, nr_allocated_dpus = 0;
        for(int i=0; i<batch.filenames.size(); i++){
            const std::size_t pos = batch.filenames[i].find_last_of('.');
            nr_allocated_dpus = batch.nr_allocated_dpus[i];
            write_BMP(batch.metadata[dpu_offset], batch.mcus, dpu_offset, (pos == std::string::npos) ? (batch.filenames[i] + ".bmp") : (batch.filenames[i].substr(0, pos) + ".bmp"));
            dpu_offset += nr_allocated_dpus;
        }
        clock_gettime(CLOCK_MONOTONIC, &tmp_end);
        total_bmp_write_time += get_time(&tmp_start, &tmp_end);
        /* Use chrono for profiling
        #ifdef DEBUG
            auto bmp_write_end = std::chrono::high_resolution_clock::now();
            execution_time_breakdown["bmp_write_time"] = bmp_write_end - bmp_write_start;
            execution_times.push_back(execution_time_breakdown);
        #endif
        */
    }

    /* Use chrono for profiling
    execution_end = std::chrono::high_resolution_clock::now();
    total_execution_time = execution_end - execution_start;
    */
    clock_gettime(CLOCK_MONOTONIC, &execution_end);
    total_execution_time = get_time(&execution_start, &execution_end);
}

int main(int argc, char *argv[]){
    if(argc < 2){
        std::cout << "Error - Invalid arguments\n";
        return 1;
    }

    std::vector<std::string> input_files;
    for(int i=1; i<argc; i++) input_files.push_back(argv[i]);
    sort_by_size(input_files);
    
    std::cout << nr_dpus << " dpus are allocated\n";

    std::thread mcu_preparer(mcu_prepare, std::ref(input_files));
    std::thread decoding_offloader(offloading);

    mcu_preparer.join();
    decoding_offloader.join();

    /* Use chrono for profiling
    #ifdef DEBUG
        auto compare = [](const std::map<std::string, std::chrono::duration<double>>& a, const std::map<std::string, std::chrono::duration<double>>& b) {
            return sum_of_map(a) < sum_of_map(b);
        };
        std::sort(execution_times.begin(), execution_times.end(), compare);
    #endif
    */

    std::cout << "\nProfiles:\n";
    // std::cout << "End-to-end execution time: " << total_execution_time.count() << "s\n";
    std::cout << "End-to-end execution time: " << total_execution_time << "s\n";

    std::cout << "MCU Offloader execution time (total): \n";
    std::cout << " - Queue waiting time: " << total_queue_waiting_time << "s\n";
    std::cout << " - Batch time: " << total_batch_time << "s\n";
    std::cout << " - CPU-to-DPUs transfer time: " << total_cpu_to_dpus_transfer_time << "s\n";
    std::cout << " - DPU execution time: " << total_dpu_execution_time << "s\n";
    std::cout << " - DPU execution - Init cycles: " << init_cycles[0][0] << " cycles\n";
    std::cout << " - DPU execution - Dequantization cycles: " << dequantization_cycles[0][0] << " cycles\n";
    std::cout << " - DPU execution - IDCT cycles: " << idct_cycles[0][0] << " cycles\n";
    std::cout << " - DPU execution - Color space conversion cycles: " << color_space_conversion_cycles[0][0] << " cycles\n";
    std::cout << " - DPUs-to-CPU transfer time: " << total_dpus_to_cpu_transfer_time << "s\n";
    std::cout << " - BMP write time: " << total_bmp_write_time << "s\n";
    std::cout << " - Total " << total_offloading_times << " calls\n";    

    /* Use chrono for profiling
    #ifdef DEBUG
        std::chrono::duration<double> total_queue_waiting_time(0), total_batch_time(0), total_cpu_to_dpus_transfer_time(0), total_dpu_execution_time(0), total_dpus_to_cpu_transfer_time(0), total_bmp_write_time(0);
        for(const auto& et : execution_times){
            total_queue_waiting_time += et.at("queue_wait_time");
            total_batch_time += et.at("batch_time");
            total_cpu_to_dpus_transfer_time += et.at("cpu_to_dpus_time");
            total_dpu_execution_time += et.at("dpu_execution_time");
            total_dpus_to_cpu_transfer_time += et.at("dpus_to_cpu_time");
            total_bmp_write_time += et.at("bmp_write_time");
        }
        std::cout << "MCU Preparer execution time (total): " << mcu_prepare_time.count() << "s\n";

        std::cout << "MCU Offloader execution time (total): \n";
        std::cout << " - Queue waiting time: " << total_queue_waiting_time.count() << "s\n";
        std::cout << " - Batch time: " << total_batch_time.count() << "s\n";
        std::cout << " - CPU-to-DPUs transfer time: " << total_cpu_to_dpus_transfer_time.count() << "s\n";
        std::cout << " - DPU execution time: " << total_dpu_execution_time.count() << "s\n";
        std::cout << " - DPU execution - Init cycles: " << init_cycles[0][0] << " cycles\n";
        std::cout << " - DPU execution - Dequantization cycles: " << dequantization_cycles[0][0] << " cycles\n";
        std::cout << " - DPU execution - IDCT cycles: " << idct_cycles[0][0] << " cycles\n";
        std::cout << " - DPU execution - Color space conversion cycles: " << color_space_conversion_cycles[0][0] << " cycles\n";
        std::cout << " - DPUs-to-CPU transfer time: " << total_dpus_to_cpu_transfer_time.count() << "s\n";
        std::cout << " - BMP write time: " << total_bmp_write_time.count() << "s\n";
        std::cout << " - Total " << total_offloading_times << " calls\n";

        std::cout << "MCU Offloader execution time (median): " << sum_of_map(execution_times[execution_times.size()/2]).count() << "s\n";
        std::cout << " - Queue waiting time: " << execution_times[execution_times.size()/2]["queue_wait_time"].count() << "s\n";
        std::cout << " - Batch time: " << execution_times[execution_times.size()/2]["batch_time"].count() << "s\n";
        std::cout << " - CPU-to-DPUs transfer time: " << execution_times[execution_times.size()/2]["cpu_to_dpus_time"].count() << "s\n";
        std::cout << " - DPU execution time: " << execution_times[execution_times.size()/2]["dpu_execution_time"].count() << "s\n";
        std::cout << " - DPUs-to-CPU transfer time: " << execution_times[execution_times.size()/2]["dpus_to_cpu_time"].count() << "s\n";
        std::cout << " - BMP write time: " << execution_times[execution_times.size()/2]["bmp_write_time"].count() << "s\n";

        if(execution_times.size() > 1){
            std::cout << "MCU Offloader execution time (maximum): " << sum_of_map(execution_times[execution_times.size()-1]).count() << "s\n";
            std::cout << " - Queue waiting time: " << execution_times[execution_times.size()-1]["queue_wait_time"].count() << "s\n";
            std::cout << " - Batch time: " << execution_times[execution_times.size()-1]["batch_time"].count() << "s\n";
            std::cout << " - CPU-to-DPUs transfer time: " << execution_times[execution_times.size()-1]["cpu_to_dpus_time"].count() << "s\n";
            std::cout << " - DPU execution time: " << execution_times[execution_times.size()-1]["dpu_execution_time"].count() << "s\n";
            std::cout << " - DPUs-to-CPU transfer time: " << execution_times[execution_times.size()-1]["dpus_to_cpu_time"].count() << "s\n";
            std::cout << " - BMP write time: " << execution_times[execution_times.size()-1]["bmp_write_time"].count() << "s\n";

            std::cout << "MCU Offloader execution time (minimum): " << sum_of_map(execution_times[0]).count() << "s\n";
            std::cout << " - Queue waiting time: " << execution_times[0]["queue_wait_time"].count() << "s\n";
            std::cout << " - Batch time: " << execution_times[0]["batch_time"].count() << "s\n";
            std::cout << " - CPU-to-DPUs transfer time: " << execution_times[0]["cpu_to_dpus_time"].count() << "s\n";
            std::cout << " - DPU execution time: " << execution_times[0]["dpu_execution_time"].count() << "s\n";
            std::cout << " - DPUs-to-CPU transfer time: " << execution_times[0]["dpus_to_cpu_time"].count() << "s\n";
            std::cout << " - BMP write time: " << execution_times[0]["bmp_write_time"].count() << "s\n";
        }
    #endif
    */

    return 0;
}