#define BUFFER_SIZE 16777216/4

typedef unsigned char byte;
typedef unsigned int uint;

const byte zigzagMap[] = {
     0,   1,   8,  16,   9,   2,   3,  10,
    17,  24,  32,  25,  18,  11,   4,   5,
    12,  19,  26,  33,  40,  48,  41,  34,
    27,  20,  13,   6,   7,  14,  21,  28,
    35,  42,  49,  56,  57,  50,  43,  36,
    29,  22,  15,  23,  30,  37,  44,  51,
    38,  59,  52,  45,  38,  31,  39,  46,
    53,  60,  61,  54,  47,  55,  62,  63
};

const float idctMap[] = {
    0.353553, 0.353553, 0.353553, 0.353553, 0.353553, 0.353553, 0.353553, 0.353553, 
    0.490393, 0.415735, 0.277785, 0.0975452, -0.0975452, -0.277785, -0.415735, -0.490393, 
    0.46194, 0.191342, -0.191342, -0.46194, -0.46194, -0.191342, 0.191342, 0.46194, 
    0.415735, -0.0975452, -0.490393, -0.277785, 0.277785, 0.490393, 0.0975452, -0.415735, 
    0.353553, -0.353553, -0.353553, 0.353553, 0.353553, -0.353553, -0.353553, 0.353553, 
    0.277785, -0.490393, 0.0975452, 0.415735, -0.415735, -0.0975452, 0.490393, -0.277785, 
    0.191342, -0.46194, 0.46194, -0.191342, -0.191342, 0.46194, -0.46194, 0.191342, 
    0.0975452, -0.277785, 0.415735, -0.490393, 0.490393, -0.415735, 0.277785, -0.0975452
};