/*!
   @file Adafruit_SSD1306.cpp

   @mainpage Arduino library for monochrome OLEDs based on SSD1306 drivers.

   @section intro_sec Introduction

   This is documentation for Adafruit's SSD1306 library for monochrome
   OLED displays: http://www.adafruit.com/category/63_98

   These displays use I2C or SPI to communicate. I2C requires 2 pins
   (SCL+SDA) and optionally a RESET pin. SPI requires 4 pins (MOSI, SCK,
   select, data/command) and optionally a reset pin. Hardware SPI or
   'bitbang' software SPI are both supported.

   Adafruit invests time and resources providing this open source code,
   please support Adafruit and open-source hardware by purchasing
   products from Adafruit!

   @section dependencies Dependencies

   This library depends on <a href="https://github.com/adafruit/Adafruit-GFX-Library">
   Adafruit_GFX</a> being present on your system. Please make sure you have
   installed the latest version before using this library.

   @section author Author

   Written by Limor Fried/Ladyada for Adafruit Industries, with
   contributions from the open source community.

   @section license License

   BSD license, all text above, and the splash screen included below,
   must be included in any redistribution.

*/

#ifdef __AVR__
#include <avr/pgmspace.h>
#else
#define pgm_read_byte(addr) \
  (*(const unsigned char *)(addr)) ///< PROGMEM workaround for non-AVR
#endif

#if !defined(__ARM_ARCH) && !defined(ENERGIA) && !defined(ESP8266) && !defined(ESP32) && !defined(__arc__)
#include <util/delay.h>
#endif

#include "Adafruit_GFX.h"
#include "SSD1306.h"

// SOME DEFINES AND STATIC VARIABLES USED INTERNALLY -----------------------

#if defined(BUFFER_LENGTH)
#define WIRE_MAX BUFFER_LENGTH          ///< AVR or similar Wire lib
#elif defined(SERIAL_BUFFER_SIZE)
#define WIRE_MAX (SERIAL_BUFFER_SIZE-1) ///< Newer Wire uses RingBuffer
#else
#define WIRE_MAX 32                     ///< Use common Arduino core default
#endif

#define ssd1306_swap(a, b) \
  (((a) ^= (b)), ((b) ^= (a)), ((a) ^= (b))) ///< No-temp-var swap operation


#define SSD1306_SELECT       digitalWrite(csPin, LOW);  ///< Device select
#define SSD1306_DESELECT     digitalWrite(csPin, HIGH); ///< Device deselect
#define SSD1306_MODE_COMMAND digitalWrite(dcPin, LOW);  ///< Command mode
#define SSD1306_MODE_DATA    digitalWrite(dcPin, HIGH); ///< Data mode

#if (ARDUINO >= 157) && !defined(ARDUINO_STM32_FEATHER)
#define SETWIRECLOCK wire->setClock(wireClk)    ///< Set before I2C transfer
#define RESWIRECLOCK wire->setClock(restoreClk) ///< Restore after I2C xfer
#else // setClock() is not present in older Arduino Wire lib (or WICED)
#define SETWIRECLOCK ///< Dummy stand-in define
#define RESWIRECLOCK ///< keeps compiler happy
#endif

#if defined(SPI_HAS_TRANSACTION)
#define SPI_TRANSACTION_START spi->beginTransaction(spiSettings) ///< Pre-SPI
#define SPI_TRANSACTION_END   spi->endTransaction()              ///< Post-SPI
#else // SPI transactions likewise not present in older Arduino SPI lib
#define SPI_TRANSACTION_START ///< Dummy stand-in define
#define SPI_TRANSACTION_END   ///< keeps compiler happy
#endif

// The definition of 'transaction' is broadened a bit in the context of
// this library -- referring not just to SPI transactions (if supported
// in the version of the SPI library being used), but also chip select
// (if SPI is being used, whether hardware or soft), and also to the
// beginning and end of I2C transfers (the Wire clock may be sped up before
// issuing data to the display, then restored to the default rate afterward
// so other I2C device types still work).  All of these are encapsulated
// in the TRANSACTION_* macros.

// Check first if Wire, then hardware SPI, then soft SPI:
#define TRANSACTION_START   \
  if(wire) {                 \
    SETWIRECLOCK;            \
  } else {                   \
    if(spi) {                \
      SPI_TRANSACTION_START; \
    }                        \
    SSD1306_SELECT;          \
  } ///< Wire, SPI or bitbang transfer setup
#define TRANSACTION_END     \
  if(wire) {                 \
    RESWIRECLOCK;            \
  } else {                   \
    SSD1306_DESELECT;        \
    if(spi) {                \
      SPI_TRANSACTION_END;   \
    }                        \
  }

Adafruit_SSD1306::Adafruit_SSD1306(uint8_t w, uint8_t h, SPIClass *spi,
                                   int8_t dc_pin, int8_t rst_pin, int8_t cs_pin, uint32_t bitrate) :
  Adafruit_GFX(w, h), spi(spi ? spi : & SPI), wire(NULL), buffer(NULL),
  mosiPin(-1), clkPin(-1), dcPin(dc_pin), csPin(cs_pin), rstPin(rst_pin) {
#ifdef SPI_HAS_TRANSACTION
  spiSettings = SPISettings(bitrate, MSBFIRST, SPI_MODE0);
#endif
}


Adafruit_SSD1306::~Adafruit_SSD1306(void) {
  if (buffer) {
    free(buffer);
    buffer = NULL;
  }
}

inline void Adafruit_SSD1306::SPIwrite(uint8_t d) {
  (void)spi->transfer(d);
}

void Adafruit_SSD1306::ssd1306_command1(uint8_t c) {
  SSD1306_MODE_COMMAND
  SPIwrite(c);
}

void Adafruit_SSD1306::ssd1306_commandList(const uint8_t *c, uint8_t n) {
  SSD1306_MODE_COMMAND
  while (n--) SPIwrite(pgm_read_byte(c++));
}

void Adafruit_SSD1306::ssd1306_command(uint8_t c) {
  TRANSACTION_START
  ssd1306_command1(c);
  TRANSACTION_END
}

boolean Adafruit_SSD1306::begin(uint8_t vcs, uint8_t addr, boolean reset,
                                boolean periphBegin) {

  if ((!buffer) && !(buffer = (uint8_t *)malloc(WIDTH * ((HEIGHT + 7) / 8))))
    return false;

  clearDisplay();

  vccstate = vcs;

  pinMode(dcPin, OUTPUT); // Set data/command pin as output
  pinMode(csPin, OUTPUT); // Same for chip select
#ifdef HAVE_PORTREG
  dcPort    = (PortReg *)portOutputRegister(digitalPinToPort(dcPin));
  dcPinMask = digitalPinToBitMask(dcPin);
  csPort    = (PortReg *)portOutputRegister(digitalPinToPort(csPin));
  csPinMask = digitalPinToBitMask(csPin);
#endif
  SSD1306_DESELECT
  if (spi) { // Hardware SPI
    // SPI peripheral begin same as wire check above.
    if (periphBegin) spi->begin();
  } else {  // Soft SPI
    pinMode(mosiPin, OUTPUT); // MOSI and SCLK outputs
    pinMode(clkPin , OUTPUT);
#ifdef HAVE_PORTREG
    mosiPort    = (PortReg *)portOutputRegister(digitalPinToPort(mosiPin));
    mosiPinMask = digitalPinToBitMask(mosiPin);
    clkPort     = (PortReg *)portOutputRegister(digitalPinToPort(clkPin));
    clkPinMask  = digitalPinToBitMask(clkPin);
    *clkPort   &= ~clkPinMask; // Clock low
#else
    digitalWrite(clkPin, LOW); // Clock low
#endif

  }

  // Reset SSD1306 if requested and reset pin specified in constructor
  if (reset && (rstPin >= 0)) {
    pinMode(     rstPin, OUTPUT);
    digitalWrite(rstPin, HIGH);
    delay(1);                   // VDD goes high at start, pause for 1 ms
    digitalWrite(rstPin, LOW);  // Bring reset low
    delay(10);                  // Wait 10 ms
    digitalWrite(rstPin, HIGH); // Bring out of reset
  }

  TRANSACTION_START


  // Init sequence
  static const uint8_t PROGMEM init1[] = {
    SSD1306_DISPLAYOFF,   0,                // 0xAE
    SSD1306_SETDISPLAYCLOCKDIV,           // 0xD5
    0x80,                                 // the suggested ratio 0x80
    SSD1306_SETMULTIPLEX
  };               // 0xA8
  ssd1306_commandList(init1, sizeof(init1));
  ssd1306_command1(HEIGHT - 1);

  static const uint8_t PROGMEM init2[] = {
    SSD1306_SETDISPLAYOFFSET,             // 0xD3
    0x0,                                  // no offset
    SSD1306_SETSTARTLINE | 0x0,           // line #0
    SSD1306_CHARGEPUMP
  };                 // 0x8D
  ssd1306_commandList(init2, sizeof(init2));

  ssd1306_command1((vccstate == SSD1306_EXTERNALVCC) ? 0x10 : 0x14);

  static const uint8_t PROGMEM init3[] = {
    SSD1306_MEMORYMODE,                   // 0x20
    0x00,                                 // 0x0 act like ks0108
    SSD1306_SEGREMAP | 0x1,
    SSD1306_COMSCANDEC
  };
  ssd1306_commandList(init3, sizeof(init3));

  if ((WIDTH == 128) && (HEIGHT == 32)) {
    static const uint8_t PROGMEM init4a[] = {
      SSD1306_SETCOMPINS,                 // 0xDA
      0x12,//normalerweise hier 0x02
      SSD1306_SETCONTRAST,                // 0x81
      0x8F
    };
    ssd1306_commandList(init4a, sizeof(init4a));
  } else if ((WIDTH == 128) && (HEIGHT == 64)) {
    static const uint8_t PROGMEM init4b[] = {
      SSD1306_SETCOMPINS,                 // 0xDA
      0x12,
      SSD1306_SETCONTRAST
    };              // 0x81
    ssd1306_commandList(init4b, sizeof(init4b));
    ssd1306_command1((vccstate == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF);
  } else if ((WIDTH == 96) && (HEIGHT == 16)) {
    static const uint8_t PROGMEM init4c[] = {
      SSD1306_SETCOMPINS,                 // 0xDA
      0x2,    // ada x12
      SSD1306_SETCONTRAST
    };              // 0x81
    ssd1306_commandList(init4c, sizeof(init4c));
    ssd1306_command1((vccstate == SSD1306_EXTERNALVCC) ? 0x10 : 0xAF);
  } else {
    // Other screen varieties -- TBD
  }

  ssd1306_command1(SSD1306_SETPRECHARGE); // 0xd9
  ssd1306_command1((vccstate == SSD1306_EXTERNALVCC) ? 0x22 : 0xF1);
  static const uint8_t PROGMEM init5[] = {
    SSD1306_SETVCOMDETECT,               // 0xDB
    0x40,
    SSD1306_DISPLAYALLON_RESUME,         // 0xA4
    SSD1306_NORMALDISPLAY,               // 0xA6
    SSD1306_DEACTIVATE_SCROLL,
    SSD1306_DISPLAYON
  };                 // Main screen turn on
  ssd1306_commandList(init5, sizeof(init5));
  TRANSACTION_END

  return true; // Success
}

void Adafruit_SSD1306::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
      case 1:
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        break;
      case 2:
        x = WIDTH  - x - 1;
        y = HEIGHT - y - 1;
        break;
      case 3:
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }
    switch (color) {
      case WHITE:   buffer[x + (y / 8)*WIDTH] |=  (1 << (y & 7)); break;
      case BLACK:   buffer[x + (y / 8)*WIDTH] &= ~(1 << (y & 7)); break;
      case INVERSE: buffer[x + (y / 8)*WIDTH] ^=  (1 << (y & 7)); break;
    }
  }
}

void Adafruit_SSD1306::clearDisplay(void) {
  memset(buffer, 0, WIDTH * ((HEIGHT + 7) / 8));
}

void Adafruit_SSD1306::drawFastHLine(
  int16_t x, int16_t y, int16_t w, uint16_t color) {
  boolean bSwap = false;
  switch (rotation) {
    case 1:
      // 90 degree rotation, swap x & y for rotation, then invert x
      bSwap = true;
      ssd1306_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      // 180 degree rotation, invert x and y, then shift y around for height.
      x  = WIDTH  - x - 1;
      y  = HEIGHT - y - 1;
      x -= (w - 1);
      break;
    case 3:
      // 270 degree rotation, swap x & y for rotation,
      // then invert y and adjust y for w (not to become h)
      bSwap = true;
      ssd1306_swap(x, y);
      y  = HEIGHT - y - 1;
      y -= (w - 1);
      break;
  }

  if (bSwap) drawFastVLineInternal(x, y, w, color);
  else      drawFastHLineInternal(x, y, w, color);
}

void Adafruit_SSD1306::drawFastHLineInternal(
  int16_t x, int16_t y, int16_t w, uint16_t color) {

  if ((y >= 0) && (y < HEIGHT)) { // Y coord in bounds?
    if (x < 0) { // Clip left
      w += x;
      x  = 0;
    }
    if ((x + w) > WIDTH) { // Clip right
      w = (WIDTH - x);
    }
    if (w > 0) { // Proceed only if width is positive
      uint8_t *pBuf = &buffer[(y / 8) * WIDTH + x],
               mask = 1 << (y & 7);
      switch (color) {
        case WHITE:               while (w--) {
            *pBuf++ |= mask;
          }; break;
        case BLACK: mask = ~mask; while (w--) {
            *pBuf++ &= mask;
          }; break;
        case INVERSE:             while (w--) {
            *pBuf++ ^= mask;
          }; break;
      }
    }
  }
}

void Adafruit_SSD1306::drawFastVLine(
  int16_t x, int16_t y, int16_t h, uint16_t color) {
  boolean bSwap = false;
  switch (rotation) {
    case 1:
      // 90 degree rotation, swap x & y for rotation,
      // then invert x and adjust x for h (now to become w)
      bSwap = true;
      ssd1306_swap(x, y);
      x  = WIDTH - x - 1;
      x -= (h - 1);
      break;
    case 2:
      // 180 degree rotation, invert x and y, then shift y around for height.
      x = WIDTH  - x - 1;
      y = HEIGHT - y - 1;
      y -= (h - 1);
      break;
    case 3:
      // 270 degree rotation, swap x & y for rotation, then invert y
      bSwap = true;
      ssd1306_swap(x, y);
      y = HEIGHT - y - 1;
      break;
  }

  if (bSwap) drawFastHLineInternal(x, y, h, color);
  else      drawFastVLineInternal(x, y, h, color);
}

void Adafruit_SSD1306::drawFastVLineInternal(
  int16_t x, int16_t __y, int16_t __h, uint16_t color) {

  if ((x >= 0) && (x < WIDTH)) { // X coord in bounds?
    if (__y < 0) { // Clip top
      __h += __y;
      __y = 0;
    }
    if ((__y + __h) > HEIGHT) { // Clip bottom
      __h = (HEIGHT - __y);
    }
    if (__h > 0) { // Proceed only if height is now positive
      // this display doesn't need ints for coordinates,
      // use local byte registers for faster juggling
      uint8_t  y = __y, h = __h;
      uint8_t *pBuf = &buffer[(y / 8) * WIDTH + x];

      // do the first partial byte, if necessary - this requires some masking
      uint8_t mod = (y & 7);
      if (mod) {
        // mask off the high n bits we want to set
        mod = 8 - mod;
        // note - lookup table results in a nearly 10% performance
        // improvement in fill* functions
        // uint8_t mask = ~(0xFF >> mod);
        static const uint8_t PROGMEM premask[8] =
        { 0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE };
        uint8_t mask = pgm_read_byte(&premask[mod]);
        // adjust the mask if we're not going to reach the end of this byte
        if (h < mod) mask &= (0XFF >> (mod - h));

        switch (color) {
          case WHITE:   *pBuf |=  mask; break;
          case BLACK:   *pBuf &= ~mask; break;
          case INVERSE: *pBuf ^=  mask; break;
        }
        pBuf += WIDTH;
      }

      if (h >= mod) { // More to go?
        h -= mod;
        // Write solid bytes while we can - effectively 8 rows at a time
        if (h >= 8) {
          if (color == INVERSE) {
            // separate copy of the code so we don't impact performance of
            // black/white write version with an extra comparison per loop
            do {
              *pBuf ^= 0xFF;  // Invert byte
              pBuf  += WIDTH; // Advance pointer 8 rows
              h     -= 8;     // Subtract 8 rows from height
            } while (h >= 8);
          } else {
            // store a local value to work with
            uint8_t val = (color != BLACK) ? 255 : 0;
            do {
              *pBuf = val;    // Set byte
              pBuf += WIDTH;  // Advance pointer 8 rows
              h    -= 8;      // Subtract 8 rows from height
            } while (h >= 8);
          }
        }

        if (h) { // Do the final partial byte, if necessary
          mod = h & 7;
          // this time we want to mask the low bits of the byte,
          // vs the high bits we did above
          // uint8_t mask = (1 << mod) - 1;
          // note - lookup table results in a nearly 10% performance
          // improvement in fill* functions
          static const uint8_t PROGMEM postmask[8] =
          { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
          uint8_t mask = pgm_read_byte(&postmask[mod]);
          switch (color) {
            case WHITE:   *pBuf |=  mask; break;
            case BLACK:   *pBuf &= ~mask; break;
            case INVERSE: *pBuf ^=  mask; break;
          }
        }
      }
    } // endif positive height
  } // endif x in bounds
}

boolean Adafruit_SSD1306::getPixel(int16_t x, int16_t y) {
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
      case 1:
        ssd1306_swap(x, y);
        x = WIDTH - x - 1;
        break;
      case 2:
        x = WIDTH  - x - 1;
        y = HEIGHT - y - 1;
        break;
      case 3:
        ssd1306_swap(x, y);
        y = HEIGHT - y - 1;
        break;
    }
    return (buffer[x + (y / 8) * WIDTH] & (1 << (y & 7)));
  }
  return false; // Pixel out of bounds
}

uint8_t *Adafruit_SSD1306::getBuffer(void) {
  return buffer;
}

void Adafruit_SSD1306::display(void) {
  TRANSACTION_START
  static const uint8_t PROGMEM dlist1[] = {
    SSD1306_PAGEADDR,//22
    0,                         // Page start address
    0xFF,                      // Page end (not really, but works here)
    SSD1306_COLUMNADDR,//22
    0
  };                       // Column start address
  ssd1306_commandList(dlist1, sizeof(dlist1));
  ssd1306_command1(WIDTH - 1); // Column end address

  uint16_t count = WIDTH * ((HEIGHT + 7) / 8);
  uint8_t *ptr   = buffer;
  SSD1306_MODE_DATA
  while (count--) SPIwrite(*ptr++);
  TRANSACTION_END
}

void Adafruit_SSD1306::startscrollright(uint8_t start, uint8_t stop) {
  TRANSACTION_START
  static const uint8_t PROGMEM scrollList1a[] = {
    SSD1306_RIGHT_HORIZONTAL_SCROLL,
    0X00
  };
  ssd1306_commandList(scrollList1a, sizeof(scrollList1a));
  ssd1306_command1(start);
  ssd1306_command1(0X00);
  ssd1306_command1(stop);
  static const uint8_t PROGMEM scrollList1b[] = {
    0X00,
    0XFF,
    SSD1306_ACTIVATE_SCROLL
  };
  ssd1306_commandList(scrollList1b, sizeof(scrollList1b));
  TRANSACTION_END
}

void Adafruit_SSD1306::startscrollleft(uint8_t start, uint8_t stop) {
  TRANSACTION_START
  static const uint8_t PROGMEM scrollList2a[] = {
    SSD1306_LEFT_HORIZONTAL_SCROLL,
    0X00
  };
  ssd1306_commandList(scrollList2a, sizeof(scrollList2a));
  ssd1306_command1(start);
  ssd1306_command1(0X00);
  ssd1306_command1(stop);
  static const uint8_t PROGMEM scrollList2b[] = {
    0X00,
    0XFF,
    SSD1306_ACTIVATE_SCROLL
  };
  ssd1306_commandList(scrollList2b, sizeof(scrollList2b));
  TRANSACTION_END
}

void Adafruit_SSD1306::startscrolldiagright(uint8_t start, uint8_t stop) {
  TRANSACTION_START
  static const uint8_t PROGMEM scrollList3a[] = {
    SSD1306_SET_VERTICAL_SCROLL_AREA,
    0X00
  };
  ssd1306_commandList(scrollList3a, sizeof(scrollList3a));
  ssd1306_command1(HEIGHT);
  static const uint8_t PROGMEM scrollList3b[] = {
    SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL,
    0X00
  };
  ssd1306_commandList(scrollList3b, sizeof(scrollList3b));
  ssd1306_command1(start);
  ssd1306_command1(0X00);
  ssd1306_command1(stop);
  static const uint8_t PROGMEM scrollList3c[] = {
    0X01,
    SSD1306_ACTIVATE_SCROLL
  };
  ssd1306_commandList(scrollList3c, sizeof(scrollList3c));
  TRANSACTION_END
}

void Adafruit_SSD1306::startscrolldiagleft(uint8_t start, uint8_t stop) {
  TRANSACTION_START
  static const uint8_t PROGMEM scrollList4a[] = {
    SSD1306_SET_VERTICAL_SCROLL_AREA,
    0X00
  };
  ssd1306_commandList(scrollList4a, sizeof(scrollList4a));
  ssd1306_command1(HEIGHT);
  static const uint8_t PROGMEM scrollList4b[] = {
    SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL,
    0X00
  };
  ssd1306_commandList(scrollList4b, sizeof(scrollList4b));
  ssd1306_command1(start);
  ssd1306_command1(0X00);
  ssd1306_command1(stop);
  static const uint8_t PROGMEM scrollList4c[] = {
    0X01,
    SSD1306_ACTIVATE_SCROLL
  };
  ssd1306_commandList(scrollList4c, sizeof(scrollList4c));
  TRANSACTION_END
}

void Adafruit_SSD1306::stopscroll(void) {
  TRANSACTION_START
  ssd1306_command1(SSD1306_DEACTIVATE_SCROLL);
  TRANSACTION_END
}

void Adafruit_SSD1306::invertDisplay(boolean i) {
  TRANSACTION_START
  ssd1306_command1(i ? SSD1306_INVERTDISPLAY : SSD1306_NORMALDISPLAY);
  TRANSACTION_END
}

void Adafruit_SSD1306::dim(boolean dim) {
  uint8_t contrast;

  if (dim) {
    contrast = 0; // Dimmed display
  } else {
    contrast = (vccstate == SSD1306_EXTERNALVCC) ? 0x9F : 0xCF;
  }
  // the range of contrast to too small to be really useful
  // it is useful to dim the display
  TRANSACTION_START
  ssd1306_command1(SSD1306_SETCONTRAST);
  ssd1306_command1(contrast);
  TRANSACTION_END
}
