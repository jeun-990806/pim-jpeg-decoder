# pim-jpeg-decoder

## 1. Build
```
git clone https://github.com/jeun-990806/pim-jpeg-decoder.git
cd pim-jpeg-decoder
make
```
* Tasklet 개수는 Makefile의 `NUM_TASKLETS` 변수 값으로 지정한다.
* DPU당 처리 가능한 MCU 개수는 Makefile의 `MAX_MCU_PER_DPU` 변수 값으로 지정한다.

## 2. Decoding
```
./bin/decoder <jpeg_image_1> ...
```
* 주어진 이미지와 동일한 위치에 동일한 이름의 bmp 파일이 생성된다.