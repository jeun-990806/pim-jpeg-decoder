#include <vector>
#include <fstream>

void put_integer(std::ofstream& output, const uint v);
void put_short(std::ofstream& output, const uint v);
void write_BMP(std::vector<uint32_t>& metadata, std::vector<std::vector<short>>& mcus, int start_dpu_index, const std::string& filename);