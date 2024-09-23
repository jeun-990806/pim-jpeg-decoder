NUM_TASKLETS := 11
MAX_MCU_PER_DPU := 100

CXX = g++
CXXFLAGS = --std=c++11 -O3 `dpu-pkg-config --cflags --libs dpu` -g -DDEBUG -DMAX_MCU_PER_DPU=$(MAX_MCU_PER_DPU)

SOURCE_DIR = src
BINARY_DIR = bin

SOURCES = $(SOURCE_DIR)/decoder_host.cpp $(SOURCE_DIR)/jpeg_scanner.cpp $(SOURCE_DIR)/bmp_writer.cpp

TARGET_HOST = $(BINARY_DIR)/decoder
TARGET_DPU = $(BINARY_DIR)/decoder_dpu

all: clean $(BINARY_DIR) $(TARGET_HOST) $(TARGET_DPU)

$(TARGET_HOST):
	$(CXX) $(CXXFLAGS) -o $@ $(SOURCES) -lpthread

$(TARGET_DPU):
	dpu-upmem-dpurte-clang -DNR_TASKLETS=$(NUM_TASKLETS) -DMAX_MCU_PER_DPU=$(MAX_MCU_PER_DPU) $(SOURCE_DIR)/decoder_dpu.c -o $@

$(BINARY_DIR):
	mkdir -p $(BINARY_DIR)

clean:
	rm -f $(TARGET_HOST) $(TARGET_DPU)

.PHONY: all clean
