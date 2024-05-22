typedef struct {
  short component[3][64];
} MCU;

typedef struct {
  byte offsets[17];
  byte symbols[162];
  unsigned int codes[162];
  byte set;
} huffman_table;

typedef struct {
    uint table[64];
    byte set;
} quantization_table;

typedef struct {
  byte horizontal_sampling_factor;
  byte vertical_sampling_factor;
  byte quantization_table_ID;
  byte huffman_DC_table_ID;
  byte huffman_AC_table_ID;
} color_component;

typedef struct {
  int width;
  int height;
  color_component color_components[3];
  int num_components;
  int restart_interval;
} jpeg_info;