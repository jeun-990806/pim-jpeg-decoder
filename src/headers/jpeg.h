#include <vector>
#include <fstream>

#include "common.h"

const byte SOF0 = 0xC0;     // Baseline DCT
const byte SOF1 = 0xC1;     // Extended sequential DCT
const byte SOF2 = 0xC2;     // Progressive DCT
const byte SOF3 = 0xC3;     // Lossless (sequential)

const byte SOF5 = 0xC5;     // Differential sequential DCT
const byte SOF6 = 0xC6;     // Differential progressive DCT
const byte SOF7 = 0xC7;     // Differential lossless (sequential)

const byte SOF9 = 0xC9;     // Extended sequential DCT
const byte SOF10 = 0xCA;    // Progressive DCT
const byte SOF11 = 0xCB;    // Lossless (sequential)

const byte SOF13 = 0xCD;    // Differential sequential DCT
const byte SOF14 = 0xCE;    // Differential progressive DCT
const byte SOF15 = 0xCF;    // Differential lossless (sequential)

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

class BitReader {
    private:
        const std::vector<byte>& data;
        uint next_byte = 0;
        uint next_bit = 0;
    public:
        BitReader(const std::vector<byte>& d) :
        data(d)
        {}

        int read_bit(){
            if(next_byte >= data.size()) return -1;
            int bit = (data[next_byte] >> (7-next_bit)) & 1;
            next_bit += 1;
            if(next_bit == 8){
                next_bit = 0;
                next_byte += 1;
            }
            return bit;
        }
        
        int read_bits(const uint length){
            int bits = 0;
            for(uint i=0; i<length; i++){
                int bit = read_bit();
                if(bit == -1){
                    bits = -1;
                    break;
                }
                bits = (bits << 1) | bit;
            }
            return bits;
        }

        void align(){
            if(next_byte >= data.size()) return;
            if(next_bit != 0){
                next_bit = 0;
                next_byte += 1;
            }
        }
};

struct QuantizationTable {
    uint table[64] = { 0 };
    bool set = false;
};

struct HuffmanTable {
    byte offsets[17] = { 0 };
    byte symbols[162] = { 0 };
    uint codes[162] = { 0 };
    bool set = false;
};

struct ColorComponent {
    byte h_sampling_factor = 1;
    byte v_sampling_factor = 1;
    byte QT_ID = 0;
    byte DHT_ID = 0;
    byte AHT_ID = 0;
    bool used_in_frame = false;
    bool used_in_scan = false;
};

struct Header {
    std::string filename;

    QuantizationTable quantization_tables[4];
    HuffmanTable huffman_DC_tables[4];
    HuffmanTable huffman_AC_tables[4];

    byte frame_type = 0;
    uint height = 0;
    uint width = 0;
    byte num_components = 0;
    ColorComponent color_components[3];
    bool zero_based = false;

    byte components_in_scan = 0;
    byte start_of_selection = 0;
    byte end_of_selection = 63;
    byte successive_approximation_high = 0;
    byte successive_approximation_low = 0;

    uint restart_interval = 0;

    std::vector<byte> huffman_data;

    uint mcu_height = 0;
    uint mcu_height_real = 0;
    uint mcu_width = 0;
    uint mcu_width_real = 0;

    byte h_sampling_factor = 1;
    byte v_sampling_factor = 1;

    bool valid = true;
};

void read_StartOfScan(std::ifstream& input, Header *const header);
void read_HuffmanTable(std::ifstream& input, Header *const header);
void read_StartOfFrame(std::ifstream& input, Header *const header);
void read_QuantizationTable(std::ifstream& input, Header *const header);
void read_RestartInterval(std::ifstream& input, Header *const header);
void read_APPN(std::ifstream& input, Header *const header);
void read_Comment(std::ifstream& input, Header *const header);

Header *read_JPEG(const std::string& filename);

void generate_codes(HuffmanTable& HT);
byte get_next_symbol(BitReader& b, const HuffmanTable& HT);
bool decode_MCU_component(BitReader& b, short *component, int& previous_DC, const HuffmanTable& DT, const HuffmanTable& AT);
bool decode_Huffman_data(Header *const header, std::vector<std::vector<short>>& MCU_buffer, int dpu_offset);