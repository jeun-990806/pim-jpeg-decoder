all:
	@mkdir bin -p
	g++ --std=c++11 -o bin/decoder src/decoder.cpp `dpu-pkg-config --cflags --libs dpu` -g
	dpu-upmem-dpurte-clang -DNR_TASKLETS=4 src/decoder_dpu.c -o bin/decoder_dpu

clean:
	rm bin/decoder