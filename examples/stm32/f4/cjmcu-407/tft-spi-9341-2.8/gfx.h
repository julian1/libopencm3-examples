
/*
    see overrides in

    Adafruit-GFX-Library/Adafruit_SPITFT.cpp

    
    void Adafruit_SPITFT::startWrite(void) {
    void Adafruit_SPITFT::endWrite(void) {
    void Adafruit_SPITFT::writePixel(int16_t x, int16_t y, uint16_t color) {

*/



void fillRect(Context *ctx, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) ;

void fillScreen(Context *ctx, uint16_t color) ;


void writeFillRect(Context *ctx, int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) ;

void writeFastHLine(Context *ctx, int16_t x, int16_t y, int16_t w, uint16_t color); 

// not sure if should be exposed.
// void drawPixel(Context *ctx, int16_t x, int16_t y, uint16_t color) ;


void writePixel(Context *ctx, int16_t x, int16_t y, uint16_t color) ;

void writeLine(Context *ctx, int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) ;


void drawCircle(Context *ctx, int16_t x0, int16_t y0, int16_t r, uint16_t color) ;




