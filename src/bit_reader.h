class BitReader {
    private:
        const std::vector<byte>& data;
        uint nextByte = 0;
        uint nextBit = 0;
    public:
        BitReader(const std::vector<byte>& d) :
        data(d)
        {}

        int readBit(){
            if(nextByte >= data.size()) return -1;
            int bit = (data[nextByte] >> (7-nextBit)) & 1;
            nextBit += 1;
            if(nextBit == 8){
                nextBit = 0;
                nextByte += 1;
            }
            return bit;
        }
        
        int readBits(const uint length){
            int bits = 0;
            for(uint i=0; i<length; i++){
                int bit = readBit();
                if(bit == -1){
                    bits = -1;
                    break;
                }
                bits = (bits << 1) | bit;
            }
            return bits;
        }

        void align(){
            if(nextByte >= data.size()) return;
            if(nextBit != 0){
                nextBit = 0;
                nextByte += 1;
            }
        }
};