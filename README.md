# pim-jpeg-decoder

## 1. Build
```
git clone https://github.com/jeun-990806/pim-jpeg-decoder.git
cd pim-jpeg-decoder
make
```

## 2. Decoding
```
./bin/decoder ./test_images/test.jpg
```
* CPU를 이용한 디코딩 결과와 대조를 위해, CPU로 디코딩/DPU로 디코딩한 두 개의 bmp 파일이 생성된다.