#include <iostream>
#include <fstream>

#include "headers/jpeg.h"

void read_StartOfScan(std::ifstream& input, Header *const header){
    if(header->num_components == 0){
        std::cout << header->filename << ": Error - SOS detected before SOF\n";
        header->valid = false;
        return;
    }

    uint length = (input.get() << 8) + input.get();

    for(uint i=0; i<header->num_components; i++){
        header->color_components[i].used_in_scan = false;
    }

    header->components_in_scan = input.get();
    if(header->components_in_scan == 0){
        std::cout << header->filename << ": Error - Scan must include at least 1 component\n";
        header->valid = false;
        return;
    }

    for(uint i=0; i<header->components_in_scan; i++){
        byte component_ID = input.get();
        if(header->zero_based) component_ID += 1;
        if(component_ID == 0 || component_ID > header->num_components){
            std::cout << header->filename << ": Error - Invalid color component ID: " << (uint)component_ID << "\n";
            header->valid = false;
            return;
        }
        ColorComponent *component = &header->color_components[component_ID - 1];
        if(!component->used_in_frame){
            std::cout << header->filename << ": Error - Invalid color component ID: " << (uint)component_ID << "\n";
            header->valid = false;
            return;
        }
        if(component->used_in_scan){
            std::cout << header->filename << ": Error - Duplicate color component ID\n";
            header->valid = false;
            return;            
        }
        component->used_in_scan = true;

        byte huffman_table_IDs = input.get();
        component->DHT_ID = huffman_table_IDs >> 4;
        component->AHT_ID = huffman_table_IDs & 0x0F;
        if(component->DHT_ID > 3){
            std::cout << header->filename << ": Error - Invalid Huffman DC table ID: " << (uint)component->DHT_ID << "\n";
            header->valid = false;
            return;
        }
        if(component->AHT_ID > 3){
            std::cout << header->filename << ": Error - Invalid Huffman AC table ID: " << (uint)component->AHT_ID << "\n";
            header->valid = false;
            return;
        }
    }

    header->start_of_selection = input.get();
    header->end_of_selection = input.get();
    byte successive_approximation = input.get();
    header->successive_approximation_high = successive_approximation >> 4;
    header->successive_approximation_low = successive_approximation & 0x0F;

    if(header->frame_type == SOF0){
        if(header->start_of_selection !=0 || header->end_of_selection != 63){
            std::cout << header->filename << ": Error - Invalid spectral selection\n";
            header->valid = false;
            return;
        }
        if(header->successive_approximation_high != 0 || header->successive_approximation_low != 0){
            std::cout << header->filename << ": Error - Invalid successive approximation\n";
            header->valid = false;
            return;
        }
    }else if(header->frame_type == SOF2){
        if(header->start_of_selection > header->end_of_selection){
            std::cout << header->filename << ": Error - Invalid spectral selection (start greater than end)\n";
            header->valid = false;
            return;
        }
        if(header->end_of_selection > 63){
            std::cout << header->filename << ": Error - Invalid spectral selection (end greater than 63)\n";
            header->valid = false;
            return;
        }
        if(header->start_of_selection == 0 && header->end_of_selection != 0){
            std::cout << header->filename << ": Error - Invalid spectral selection (contains DC and AC)\n";
            header->valid = false;
            return;
        }
        if(header->start_of_selection != 0 && header->components_in_scan != 1){
            std::cout << header->filename << ": Error - Invalid spectral selection (AC scan contains multiple components)\n";
            header->valid = false;
            return;
        }
        if(header->successive_approximation_high != 0 &&
           header->successive_approximation_low != header->successive_approximation_high - 1){
            std::cout << header->filename << ": Error - Invalid succesive approximation\n";
            header->valid = false;
            return;
        }
    }

    for(uint i=0; i<header->num_components; i++){
        const ColorComponent& component = header->color_components[i];
        if(header->color_components[i].used_in_scan){
            if(header->quantization_tables[component.QT_ID].set == false){
                std::cout << header->filename << ": Error - Color component using uninitialized quantization table\n";
                header->valid = false;
                return;
            }
            if(header->start_of_selection == 0){
                if(header->huffman_DC_tables[component.DHT_ID].set == false){
                    std::cout << header->filename << ": Error - Color component using uninitialized Huffman DC table\n";
                    header->valid = false;
                    return;
                }
            }
            if(header->end_of_selection > 0){
                if(header->huffman_AC_tables[component.AHT_ID].set == false){
                    std::cout << header->filename << ": Error - Color component using uninitialized Huffman AC table\n";
                    header->valid = false;
                    return;
                }
            }
        }
    }

    if(length - 6 - (2 * header->components_in_scan) != 0){
        std::cout << header->filename << ": Error - SOS invalid\n";
        header->valid = false;
        return;
    }
}

void read_HuffmanTable(std::ifstream& input, Header *const header){
    HuffmanTable *huffman_table;
    byte table_info, table_ID;
    int length = ((input.get() << 8) + input.get()) - 2;

    while(length > 0){
        table_info = input.get();
        table_ID = table_info & 0x0F;

        if(table_ID > 3){
            std::cout << header->filename << ": Error - Invalid Huffman table ID: " << (uint)table_ID << "\n";
            header->valid = false;
            return;
        }

        if(table_info >> 4){
            huffman_table = &header->huffman_AC_tables[table_ID];
        }else{
            huffman_table = &header->huffman_DC_tables[table_ID];
        }
        huffman_table->set = true;

        huffman_table->offsets[0] = 0;
        uint all_symbols = 0;
        for(uint i=1; i<=16; i++){
            all_symbols += input.get();
            huffman_table->offsets[i] = all_symbols;
        }
        if(all_symbols > 162){
            std::cout << header->filename << ": : Error - Too many symbols in Huffman table\n";
            header->valid = false;
            return;
        }

        for(uint i=0; i<all_symbols; i++){
            huffman_table->symbols[i] = input.get();
        }

        length -= 17 + all_symbols;
    }

    if(length != 0){
        std::cout << header->filename << ": Error - DHT invalid\n";
        header->valid = false;
    }
}

void read_StartOfFrame(std::ifstream& input, Header *const header){
    if(header->num_components != 0){
        std::cout << header->filename << ": Error - Multiple SOFs detected\n";
        header->valid = false;
        return;
    }

    uint length = (input.get() << 8) + input.get();

    byte precision = input.get();
    if(precision != 8){
        std::cout << header->filename << ": Error - Invalid precision: " << (uint)precision << "\n";
        header->valid = false;
        return;
    }

    header->height = (input.get() << 8) + input.get();
    header->width = (input.get() << 8) + input.get();
    if(header->height == 0 || header->width == 0){
        std::cout << header->filename << ": Error - Invalid dimensions\n";
        header->valid = false;
        return;
    }
    header->mcu_height = (header->height + 7) / 8;
    header->mcu_width = (header->width + 7) / 8;
    header->mcu_height_real = header->mcu_height;
    header->mcu_width_real = header->mcu_width;

    header->num_components = input.get();
    if(header->num_components == 4){
        std::cout << header->filename << ": Error - CMYK color mode not supported\n";
        header->valid = false;
        return;
    }
    if(header->num_components == 0){
        std::cout << header->filename << ": Error - Number of color components must not be zero\n";
        header->valid = false;
        return;
    }
    for(uint i=0; i<header->num_components; i++){
        byte component_ID = input.get();
        if(component_ID == 0 && i == 0) header->zero_based = true;
        if(header->zero_based) component_ID += 1;
        if(component_ID == 4 || component_ID == 5){
            std::cout << header->filename << ": Error - YIQ color mode not supported\n";
            header->valid = false;
            return;
        }
        if(component_ID == 0 || component_ID > header->num_components){
            std::cout << header->filename << ": Error - Invalid component ID:" << (uint)component_ID << "\n";
            header->valid = false;
            return;
        }
        ColorComponent *component = &header->color_components[component_ID - 1];
        if(component->used_in_frame){
            std::cout << header->filename << ": Duplicate color component ID\n";
            header->valid = false;
            return;
        }
        component->used_in_frame = true;
        byte sampling_factors = input.get();
        component->h_sampling_factor = sampling_factors >> 4;
        component->v_sampling_factor = sampling_factors & 0x0F;
        if(component_ID == 1){
            if((component->h_sampling_factor != 1 && component->h_sampling_factor != 2) ||
               (component->v_sampling_factor != 1 && component->v_sampling_factor != 2)){
                std::cout << header->filename << ": Error - Sampling factors not supported\n";
                header->valid = false;
                return;
            }
            if(component->h_sampling_factor == 2 && header->mcu_width % 2 == 1){
                header->mcu_width_real += 1;
            }
            if(component->v_sampling_factor == 2 && header->mcu_height % 2 == 1){
                header->mcu_height_real += 1;
            }
            header->h_sampling_factor = component->h_sampling_factor;
            header->v_sampling_factor = component->v_sampling_factor;
        }else{
            if(component->h_sampling_factor != 1 || component->v_sampling_factor != 1){
                std::cout << header->filename << ": Error - Sampling factors not supported\n";
                header->valid = false;
                return;
            }
        }
        
        component->QT_ID = input.get();
        if(component->QT_ID > 3){
            std::cout << header->filename << ": Error - Invalid quantization table ID in frame components\n";
            header->valid = false;
            return;
        }
    }

    if(length - 8 - (3 * header->num_components) != 0){
        std::cout << header->filename << ": Error - SOF invalid\n";
        header->valid = false;
    }
}

void read_QuantizationTable(std::ifstream& input, Header *const header){
    byte table_info, table_ID;
    int length = ((input.get() << 8) + input.get()) - 2;

    while(length > 0){
        table_info = input.get();
        length -= 1;
        table_ID = table_info & 0x0F;

        if(table_ID > 3){
            std::cout << header->filename << ": Error Invalid quantization table ID: " << (uint)table_ID << "\n";
            header->valid = false;
            return;
        }
        
        header->quantization_tables[table_ID].set = true;

        if(table_info >> 4 != 0){
            for(uint i=0; i<64; i++){
                header->quantization_tables[table_ID].table[zigzag_map[i]] = (input.get() << 8) + input.get();
            }
            length -= 128;
        }else{
            for(uint i=0; i<64; i++){
                header->quantization_tables[table_ID].table[zigzag_map[i]] = input.get();
            }
            length -= 64;
        }
    }

    if(length != 0){
        std::cout << header->filename << ": Error - DQT invalid\n";
        header->valid = false;
    }
}

void read_RestartInterval(std::ifstream& input, Header *const header){
    uint length = (input.get() << 8) + input.get();

    header->restart_interval = (input.get() << 8) + input.get();
    if(length - 4 != 0){
        std::cout << header->filename << ": Error - DRI invalid\n";
        header->valid = false;
    }
}

void read_APPN(std::ifstream& input, Header *const header){
    uint length = (input.get() << 8) + input.get();

    for(uint i=0; i<length-2; i++) input.get();
}

void read_Comment(std::ifstream& input, Header *const header){
    uint length = (input.get() << 8) + input.get();

    for(uint i=0; i<length-2; i++) input.get();
}

Header *read_JPEG(const std::string& filename){
    std::ifstream input = std::ifstream(filename, std::ios::in | std::ios::binary);
    if(!input.is_open()){
        std::cout << filename << ": Error - Error opening input file\n";
        return nullptr;
    }

    Header *header = new (std::nothrow) Header;
    if(header == nullptr){
        std::cout << filename << ": Error - Memory error\n";
        input.close();
        return nullptr;
    }

    header->filename = filename;

    byte last, current;

    last = input.get(); current = input.get();
    if(last != 0xFF || current != SOI){
        header->valid = false;
        input.close();
        return header;
    }

    last = input.get(); current = input.get();
    while(header->valid){
        if(!input || last != 0xFF){
            if(!input) std::cout << filename << ": Error - File ended prematurely\n";
            if(last != 0xFF) std::cout << filename << ": Error - Expected a marker\n";
            header->valid = false;
            input.close();
            return header;
        }

        if(current == SOF0 || current == SOF2){
            header->frame_type = current;
            read_StartOfFrame(input, header);
        }
        else if(current == DQT) read_QuantizationTable(input, header);
        else if(current == DHT) read_HuffmanTable(input, header);
        else if (current == SOS){
            read_StartOfScan(input, header);
            break;
        }
        else if(current == DRI) read_RestartInterval(input, header);
        else if(current >= APP0 && current <= APP15) read_APPN(input, header);
        else if(current == COM) read_Comment(input, header);
        else if((current >= JPG0 && current <= JPG13) || current == DNL || current == DHP || current == EXP) read_Comment(input, header);
        else if(current == TEM) {}
        else if(current == 0xFF){
            current = input.get();
            continue;
        }
        else std::cout << filename << ": Error - Unknown marker: 0x" << std::hex << (uint)current << std::dec << "\n";

        last = input.get();
        current = input.get();
    }
    
    if(header->valid){
        current = input.get();
        while(true){
            if(!input){
                std::cout << filename << ": Error - File ended prematurely\n";
                header->valid = false;
                input.close();
                return header;
            }

            last = current;
            current = input.get();
            if(last == 0xFF){
                if(current == EOI) break;
                else if(current == 0x00){
                    header->huffman_data.push_back(last);
                    current = input.get();
                }
                else if(current >= RST0 && current <= RST7) current = input.get();
                else if(current == 0xFF) continue;
                else{
                    std::cout << filename << ": Error - Invalid marker during compressed data scan: 0x" << std::hex << (uint)current << std::dec << "\n";
                    header->valid = false;
                    input.close();
                    return header;       
                }
            } else header->huffman_data.push_back(last);
        }
    }
    input.close();
    return header;
}

void generate_codes(HuffmanTable& HT){
    uint code = 0;

    for(uint i=0; i<16; i++){
        for(uint j=HT.offsets[i]; j<HT.offsets[i+1]; j++){
            HT.codes[j] = code;
            code += 1;
        }
        code <<= 1;
    }
}

byte get_next_symbol(BitReader& b, const HuffmanTable& HT){
    uint codeword = 0;
    int bit;

    for(uint i=0; i<16; i++){
        bit = b.read_bit();
        if(bit == -1) return -1;
        codeword = (codeword << 1) | bit;
        for(uint j=HT.offsets[i]; j<HT.offsets[i+1]; j++){
            if(codeword == HT.codes[j]) {
                return HT.symbols[j];
            }
        }
    }
    return -1;
}

bool decode_MCU_component(Header *const header, BitReader& b, short *component, int& previous_DC, uint& skips, const HuffmanTable& DT, const HuffmanTable& AT){
    if(header->frame_type == SOF0){
        byte length = get_next_symbol(b, DT);
        if(length == (byte) - 1){
            std::cout << header->filename << ": Error - Invalid DC value (" << (uint)length << ")\n";
            return false;
        }
        if(length > 11){
            std::cout << header->filename << ": Error - DC coefficient length greater than 11\n";
            return false;
        }

        int coeff = b.read_bits(length);
        if(coeff == -1){
            std::cout << header->filename << ": Error - Invalid DC value\n";
            return false;
        }
        if(length != 0 && coeff < (1 << (length - 1))) coeff -= (1 << length) - 1;
        component[0] = coeff + previous_DC;
        previous_DC = component[0];

        for(uint i=1; i<64; i++){
            byte symbol = get_next_symbol(b, AT);
            if(symbol == (byte) - 1){
                std::cout << header->filename << ": Error - Invalid AC value\n";
                return false;
            }
            if(symbol == 0x00) return true;

            byte num_zeros = symbol >> 4;
            byte coeff_length = symbol & 0x0F;
            coeff = 0;

            if(i + num_zeros >= 64){
                std::cout << header->filename << ": Error - Zero run-length exceeded block component\n";
                return false;
            }
            i += num_zeros;

            if(coeff_length > 10){
                std::cout << header->filename << ": Error - AC coefficient length greater than 10\n";
                return false;
            }
            
            coeff = b.read_bits(coeff_length);
            if(coeff == -1){
                std::cout << header->filename << ": Error - Invalid AC value\n";
                return false;
            }
            if(coeff < (1 << (coeff_length - 1))) coeff -= (1 << coeff_length) - 1;
            component[zigzag_map[i]] = coeff;
        }

        return true;
    }else{
        if (header->start_of_selection == 0 && header->successive_approximation_high == 0) {
            byte length = get_next_symbol(b, DT);
            if(length == (byte) - 1){
                std::cout << header->filename << ": Error - Invalid DC value\n";
                return false;
            }
            if(length > 11){
                std::cout << header->filename << ": Error - DC coefficient length greater than 11\n";
                return false;
            }

            int coeff = b.read_bits(length);
            if(coeff == -1){
                std::cout << header->filename << ": Error - Invalid DC value\n";
                return false;
            }
            if(length != 0 && coeff < (1 << (length - 1))) coeff -= (1 << length) - 1;
            coeff += previous_DC;
            previous_DC = coeff;
            component[0] = coeff << header->successive_approximation_low;
            return true;
        } else if (header->start_of_selection == 0 && header->successive_approximation_high != 0) {
            int bit = b.read_bit();
            if (bit == -1) {
                std::cout << header->filename << ": Error - Invalid DC value\n";
                return false;
            }
            component[0] |= bit << header->successive_approximation_low;
            return true;
        }else if(header->start_of_selection != 0 && header->successive_approximation_high == 0){
            if(skips > 0){
                skips -= 1;
                return true;
            }
            for(uint i=header->start_of_selection; i<=header->end_of_selection; i++) {
                byte symbol = get_next_symbol(b, AT);
                if (symbol == (byte)-1) {
                    std::cout << header->filename << ": Error - Invalid AC value\n";
                    return false;
                }

                byte num_zeros = symbol >> 4;
                byte coeff_length = symbol & 0x0F;
                if(coeff_length != 0){
                    if(i + num_zeros > header->end_of_selection){
                        std::cout << header->filename << ": Error - Zero run-length exceeded spectral selection\n";
                        return false;
                    }
                    for(uint j=0; j<num_zeros; j++, i++){
                        component[zigzag_map[i]] = 0;
                    }
                    if(coeff_length > 10){
                        std::cout << header->filename << ": Error - AC coefficient length greater than 10\n";
                        return false;
                    }

                    int coeff = b.read_bits(coeff_length);
                    if(coeff == -1){
                        std::cout << header->filename << ": Error - Invalid AC value\n";
                        return false;
                    }
                    if(coeff < (1 << (coeff_length - 1))) coeff -= (1 << coeff_length) - 1;
                    component[zigzag_map[i]] = coeff << header->successive_approximation_low;
                }else{
                    if(num_zeros == 15){
                        if(i + num_zeros > header->end_of_selection){
                            std::cout << header->filename << ": Error - Zero run-length exceeded spectral selection\n";
                            return false;
                        }
                        for(uint j=0; j<num_zeros; j++, i++){
                            component[zigzag_map[i]] = 0;
                        }
                    }else{
                        skips = (1 << num_zeros) - 1;
                        uint extra_skips = b.read_bits(num_zeros);
                        if (extra_skips == (uint) - 1) {
                            std::cout << header->filename << ": Error - Invalid AC value\n";
                            return false;
                        }
                        skips += extra_skips;
                        break;
                    }
                }
            }
            return true;
        }else{
            int positive = 1 << header->successive_approximation_low;
            int negative = ((unsigned) - 1) << header->successive_approximation_low;
            int i = header->start_of_selection;
            if(skips == 0){
                for(; i<=header->end_of_selection; i++){
                    byte symbol = get_next_symbol(b, AT);
                    if(symbol == (byte) - 1){
                        std::cout << header->filename << ": Error - Invalid AC value\n";
                        return false;
                    }

                    byte num_zeros = symbol >> 4;
                    byte coeff_length = symbol & 0x0F;
                    int coeff = 0;

                    if(coeff_length != 0){
                        if(coeff_length != 1){
                            std::cout << header->filename << ": Error - Invalid AC value\n";
                            return false;
                        }
                        switch(b.read_bit()){
                        case 1:
                            coeff = positive;
                            break;
                        case 0:
                            coeff = negative;
                            break;
                        default:
                            std::cout << header->filename << ": Error - Invalid AC value\n";
                            return false;
                        }
                    }else{
                        if(num_zeros != 15){
                            skips = 1 << num_zeros;
                            uint extra_skips = b.read_bits(num_zeros);
                            if(extra_skips == (uint) - 1){
                                std::cout << header->filename << ": Error - Invalid AC value\n";
                                return false;
                            }
                            skips += extra_skips;
                            break;
                        }
                    }

                    do{
                        if(component[zigzag_map[i]] != 0){
                            switch(b.read_bit()){
                            case 1:
                                if((component[zigzag_map[i]] & positive) == 0){
                                    if (component[zigzag_map[i]] >= 0) component[zigzag_map[i]] += positive;
                                    else component[zigzag_map[i]] += negative;
                                }
                                break;
                            case 0:
                                break;
                            default: 
                                std::cout << header->filename << ": Error - Invalid AC value\n";
                                return false;
                            }
                        }else{
                            if(num_zeros == 0) break;
                            num_zeros -= 1;
                        }
                        i += 1;
                    }while(i <= header->end_of_selection);

                    if(coeff != 0 && i <= header->end_of_selection) component[zigzag_map[i]] = coeff;
                }
            }

            if(skips > 0){
                for(; i <= header->end_of_selection; i++){
                    if(component[zigzag_map[i]] != 0){
                        switch(b.read_bit()){
                        case 1:
                            if ((component[zigzag_map[i]] & positive) == 0) {
                                if (component[zigzag_map[i]] >= 0) {
                                    component[zigzag_map[i]] += positive;
                                }
                                else {
                                    component[zigzag_map[i]] += negative;
                                }
                            }
                            break;
                        case 0:
                            break;
                        default: 
                            std::cout << header->filename << ": Error - Invalid AC value\n";
                            return false;
                        }
                    }
                }
                skips -= 1;
            }
            return true;
        }
    }
}

bool decode_Huffman_data(Header *const header, std::vector<std::vector<short>>& MCU_buffer, int dpu_offset){
    for(uint i=0; i<4; i++){
        if(header->huffman_DC_tables[i].set) generate_codes(header->huffman_DC_tables[i]);
        if(header->huffman_AC_tables[i].set) generate_codes(header->huffman_AC_tables[i]);
    }

    BitReader b(header->huffman_data);
    int previous_DCs[3] = { 0 };
    uint skips = 0;

    int max_blk_per_dpu = MAX_MCU_PER_DPU / 4;
    const int blk_offset_full = 768;
    const int blk_offset_comp = 256;

    for(uint y=0; y<header->mcu_height; y+=header->v_sampling_factor){
      for(uint x=0; x<header->mcu_width; x+=header->h_sampling_factor){
        if(header->restart_interval != 0 && (y * header->mcu_width_real + x) % header->restart_interval == 0){
            previous_DCs[0] = 0;
            previous_DCs[1] = 0;
            previous_DCs[2] = 0;
            skips = 0;
            b.align();
        }
        for(uint j=0; j<header->num_components; j++){
          for(uint v=0; v<header->color_components[j].v_sampling_factor; v++){
            for(uint h=0; h<header->color_components[j].h_sampling_factor; h++){
                int mcu_index = (y + v) * header->mcu_width_real + (x + h);
                int block_index = (mcu_index / (header->mcu_width_real * 2)) * ((header->mcu_width_real + 1) / 2) + ((mcu_index % header->mcu_width_real) / 2);
                int block_position = ((mcu_index / header->mcu_width_real) % 2) * 2 + ((mcu_index % header->mcu_width_real) % 2);
                int dpu_index = dpu_offset + (block_index / max_blk_per_dpu);
                block_index %= max_blk_per_dpu;
                if(!decode_MCU_component(
                    header,
                    b, 
                    &MCU_buffer[dpu_index][(block_index * blk_offset_full) + (j * blk_offset_comp) + (block_position * 64)], 
                    previous_DCs[j],
                    skips,
                    header->huffman_DC_tables[header->color_components[j].DHT_ID], 
                    header->huffman_AC_tables[header->color_components[j].AHT_ID]))
                {
                    return false;
                }
            }
          }
        }
      }
    }

    return true;
}