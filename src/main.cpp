#include <Arduino.h>

#include "sdkconfig.h"          // used for log printing
#include "esp_system.h"
#include "freertos/FreeRTOS.h"  //freeRTOS items to be used
#include "freertos/task.h"

// SPIFFS File System on ESP32-S3
//#include <LittleFS.h>
//struct LittleFile {
//  File file;
//};

// SD-CARD File System [FAT32]
#include "SD.h"
#include "FS.h"
#include <ffat.h>
struct FatFile {
  File file;
};
//#define FILESYSTEM LittleFS //FFat
#define FILESYSTEM FFat
#define FORMAT_FILESYSTEM false

#include <LovyanGFX.hpp>
#define LGFX_USE_V1
#include <lvgl.h>
//#include <demos/lv_demos.h>
#include "lv_ui.h"

//extern lv_obj_t * win_obj1;
extern   lv_obj_t * img_logo;
extern   lv_obj_t * label_obj0;

#include <SPI.h>
#include <Wire.h>
#include "board_config.h"
//uninitalised pointers to SPI objects
SPIClass * vspi = NULL; //LoRa

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_ST7796  _panel_instance;
  lgfx::Bus_SPI       _bus_instance;     // SPI bus instance
  lgfx::Light_PWM     _light_instance;
  lgfx::Touch_FT5x06  _touch_instance;   // FT5206, FT5306, FT5406, FT6206, FT6236, FT6336, FT6436

public:
  LGFX(void)
  {
    { // Configure bus control settings.
      auto cfg = _bus_instance.config(); // Get the structure for bus settings.
      // Configuring the SPI bus
      cfg.spi_host = HSPI_HOST;
      cfg.spi_mode = 0;                  // Set SPI communication mode (0 ~ 3)
      cfg.freq_write = 78000000;         // SPI clock when transmitting (maximum 80MHz, rounded to 80MHz divided by an integer)
      cfg.freq_read  = 16000000;         // SPI clock when receiving
      cfg.spi_3wire  = false;            // Set true if receiving is done using the MOSI pin.
      cfg.use_lock   = true;             // Set true to use transaction locking
      cfg.dma_channel = SPI_DMA_CH_AUTO; // Set the DMA channel to use (0=DMA not used / 1=1ch / 2=ch / SPI_DMA_CH_AUTO=automatic setting)
      cfg.pin_sclk = HSPI_SCLK_PIN;       // Set SPI SCLK pin number
      cfg.pin_mosi = HSPI_MOSI_PIN;       // Set SPI MOSI pin number
      cfg.pin_miso = HSPI_MISO_PIN;       // Set SPI MISO pin number (-1 = disable)
      cfg.pin_dc   = TFT_DC_PIN;         // Set SPI D/C pin number (-1 = disable)

      _bus_instance.config(cfg);         // Reflects the setting value on the bus.
      _panel_instance.setBus(&_bus_instance);      // Place the bus on the panel.
    }

    { // Configure display panel control settings.
      auto cfg = _panel_instance.config();    // Gets the structure for display panel settings.

      cfg.pin_cs           =    TFT_CS_PIN;   // Pin number to which CS is connected (-1 = disable)
      cfg.pin_rst          =    TFT_RST_PIN;  // Pin number to which RST is connected (-1 = disable)
      cfg.pin_busy         =    -1;  // Pin number to which BUSY is connected (-1 = disable)

      cfg.panel_width      =   320;  // Actual displayable width
      cfg.panel_height     =   480;  // Actual display height
      cfg.offset_x         =     0;  // Panel X direction offset amount
      cfg.offset_y         =     0;  // Panel Y direction offset amount
      cfg.offset_rotation  =     0;  // Offset of value in rotation direction 0~7 (4~7 are upside down)
      cfg.dummy_read_pixel =     8;  // Number of dummy read bits before pixel readout
      cfg.dummy_read_bits  =     1;  // Number of bits for dummy read before reading data other than pixels
      cfg.readable         =  true;  // Set to true if data reading is possible
      cfg.invert           =  true;  // Set to true if the brightness and darkness of the panel is reversed.
      cfg.rgb_order        = false;  // Set to true if the red and blue colors of the panel are swapped.
      cfg.dlen_16bit       = false;  // Set to true for panels that transmit data length in 16-bit units using 16-bit parallel or SPI.
      cfg.bus_shared       =  true;  // Set to true when sharing the bus with the SD card (control the bus using drawJpgFile, etc.)

      _panel_instance.config(cfg);
    }

    { // Configure backlight control settings. (Delete if unnecessary)
      auto cfg = _light_instance.config();    // Gets the structure for backlight settings.

      cfg.pin_bl = TFT_BL_PIN;      // Pin number to which the backlight is connected
      cfg.invert = false;           // true to invert the backlight brightness
      cfg.freq   = 75000; //44100;           // Backlight PWM frequency
      cfg.pwm_channel = 5;          // PWM channel number to use

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);  // Set the backlight on the panel.
    }

    { // Configure touch screen control settings. (Delete if unnecessary)
      auto cfg = _touch_instance.config();

      cfg.x_min      = 0;          // Minimum X value obtained from touch screen (raw value)
      cfg.x_max      = 319;        // Maximum X value obtained from touch screen (raw value)
      cfg.y_min      = 0;          // Minimum Y value obtained from touch screen (raw value)
      cfg.y_max      = 479;        // Maximum Y value obtained from touch screen (raw value)
      cfg.pin_int    = TP_INT_PIN; // Pin number to which INT is connected
      cfg.bus_shared = true;       // Set true if using the same bus as the screen
      cfg.offset_rotation = 0;     // Adjustment when the display and touch direction do not match Set as a value from 0 to 7
      // For I2C connection
      cfg.i2c_port = 0;            // Select I2C to use (0 or 1)
      cfg.i2c_addr = TP_I2C_ADDR;  // I2C device address number
      cfg.pin_sda  = I2C_SDA_PIN;  // Pin number to which SDA is connected
      cfg.pin_scl  = I2C_SCL_PIN;  // Pin number to which SCL is connected
      cfg.freq = 400000;           // Set I2C clock

      _touch_instance.config(cfg);
      _panel_instance.setTouch(&_touch_instance);  // Set the touch screen on the panel.
    }
    setPanel(&_panel_instance);    // Set the panel to be used.
  }
};
// Create an instance of the prepared class.
LGFX gfx;

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t        *disp_draw_buf;
static lv_disp_drv_t      disp_drv;
static lv_indev_drv_t     indev_drv;
static lv_fs_drv_t        fs_drv;                /*Needs to be static or global*/

void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p );
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data );
static void* fs_open(lv_fs_drv_t* drv, const char* path, lv_fs_mode_t mode);
static lv_fs_res_t fs_close(lv_fs_drv_t* drv, void* file_p);
static lv_fs_res_t fs_read(lv_fs_drv_t* drv, void* file_p, void* buf, uint32_t btr, uint32_t* br);
static lv_fs_res_t fs_write(lv_fs_drv_t* drv, void* file_p, const void* buf, uint32_t btw, uint32_t* bw);
static lv_fs_res_t fs_seek(lv_fs_drv_t* drv, void* file_p, uint32_t pos, lv_fs_whence_t whence);

void setup()
{
  Serial.begin(115200); // USB Serial
  //while(!Serial) { delay (10); }    // for waiting USB-Serial debug
  //Serial0.begin(115200, SERIAL_8N1, RX0_PIN, TX0_PIN);
  //Serial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN); //EC25
  //Serial2.begin(9600,   SERIAL_8N1, RX2_PIN, TX2_PIN); //BUS485

  gfx.begin();
  // Set the rotation direction from 4 directions from 0 to 3.
  // (If you use rotations 4 to 7, the image will be mirrored.)
  gfx.setRotation( 3 ); /* Landscape orientation, flipped */
  // Set the backlight brightness in the range 0-255
  gfx.setBrightness(255);
  // Set the color mode as needed. (Initial value is 16)
  // 16 - Faster, but the red and blue tones are 5 bits.
  // 24 - Slower, but the gradation expression is cleaner.
  gfx.setColorDepth(16);
  //gfx.setTextSize((std::max(gfx.width(), gfx.height()) + 255) >> 8);
/*
//  if(!LittleFS.begin()){
//    Serial.println("LittleFS Mount Failed");
//    return;
//  }
  //SPI.begin(SPI2_SCLK_PIN, SPI2_MISO_PIN, SPI2_MOSI_PIN, SD_CS_PIN);
  //if(!SD.begin(cs)){ //Change to this function to manually change CS pin
  if(!SD.begin(SD_CS_PIN)){
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
  Serial.println("No SD card attached");
    //return;
  } 

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
*/
  if(!FFat.begin()){
    //Serial.println("FFat Mount Failed");
    FFat.format(); //for first mount failed
    if(!FFat.begin()){
      Serial.println("FFat Mount Failed");
      return;
    }
  }
  Serial.println("SPIFFS FFat Mount");
  Serial.printf("Total space: %10u\n", FFat.totalBytes());
  Serial.printf("Free space: %10u\n", FFat.freeBytes());

  lv_init();
  //disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 40, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * screenWidth * 40, MALLOC_CAP_SPIRAM);
  if (!disp_draw_buf) {
    Serial.println("LVGL disp_draw_buf1 allocate failed!");
  } else {
    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, screenWidth * 40);

    /*Initialize the display*/
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the input device driver*/
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    /*Initialize the (LittleFS) File System device driver*/
    lv_fs_drv_init(&fs_drv);                  /*Basic initialization*/

    fs_drv.letter = 'S';                      /*An uppercase letter to identify the drive */
    //fs_drv.cache_size = LV_FS_FATFS_CACHE_SIZE;
    fs_drv.open_cb = fs_open;                 /*Callback to open a file */
    fs_drv.close_cb = fs_close;               /*Callback to close a file */
    fs_drv.read_cb = fs_read;                 /*Callback to close a file */
    fs_drv.write_cb = fs_write;               /*Callback to write a file */
    fs_drv.seek_cb = fs_seek;                 /*Callback to seek in a file (Move cursor) */
    fs_drv.tell_cb = NULL;                    /*Callback to tell the cursor position  */
    fs_drv.dir_open_cb = NULL;                /*Callback to open directory to read its content */
    fs_drv.dir_read_cb = NULL;                /*Callback to read a directory's content */
    fs_drv.dir_close_cb = NULL;               /*Callback to close a directory */

    lv_fs_drv_register(&fs_drv);              /*Finally register the drive*/

    lv_things_widgets();

  }

// LoRa Port init
//  vspi = new SPIClass(VSPI_HOST);
//  vspi->begin(VSPI_SCLK_PIN, VSPI_MISO_PIN, VSPI_MOSI_PIN, VSPI_SS_PIN); //SCLK, MISO, MOSI, SS // LoRa
//  pinMode(vspi->pinSS(), OUTPUT); //VSPI SS
}

////////////////////////////////////////////////////////////////////////
void loop()
{
  lv_timer_handler(); /* let the GUI do its work */  
  delay(10);

}

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    if (gfx.getStartCount() == 0)
    {   // Processing if not yet started
        gfx.startWrite();
    }
    gfx.pushImageDMA( area->x1
                    , area->y1
                    , area->x2 - area->x1 + 1
                    , area->y2 - area->y1 + 1
                    , ( lgfx::rgb565_t* )&color_p->full);
    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX, touchY;

    data->state = LV_INDEV_STATE_REL;

    if( gfx.getTouch( &touchX, &touchY ) )
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;
    }
}

/* ************************************************************************* *
 * Open a file
 * @param drv       pointer to a driver where this function belongs
 * @param path      path to the file beginning with the driver letter (e.g. S:/folder/file.txt)
 * @param mode      read: FS_MODE_RD, write: FS_MODE_WR, both: FS_MODE_RD | FS_MODE_WR
 * @return          a file descriptor or NULL on error
 * ************************************************************************* */
static void* fs_open(lv_fs_drv_t* drv, const char* path, lv_fs_mode_t mode) {
  LV_UNUSED(drv);

  const char* flags = "";

  if (mode == LV_FS_MODE_WR)
    flags = FILE_WRITE;
  else if (mode == LV_FS_MODE_RD)
    flags = FILE_READ;
  else if (mode == (LV_FS_MODE_WR | LV_FS_MODE_RD))
    flags = FILE_WRITE;

  //File f = LittleFS.open(path, flags);
  File f = FFat.open(path, flags);
  if (!f) {
    return NULL;
  }

  //LittleFile* lf = new LittleFile{f};
  FatFile* lf = new FatFile{f};

  return (void*)lf;
}

/* ************************************************************************* *
 * Close an opened file
 * @param drv       pointer to a driver where this function belongs
 * @param file_p    pointer to a file_t variable. (opened with fs_open)
 * @return          LV_FS_RES_OK: no error or  any error from @lv_fs_res_t enum
 * ************************************************************************* */
static lv_fs_res_t fs_close(lv_fs_drv_t* drv, void* file_p) {
  LV_UNUSED(drv);
  //LittleFile* lf = (LittleFile*)file_p;
  FatFile* lf = (FatFile*)file_p;

  lf->file.close();

  delete lf;
  return LV_FS_RES_OK;
}

/* ************************************************************************* *
 * Read data from an opened file
 * @param drv       pointer to a driver where this function belongs
 * @param file_p    pointer to a file_t variable.
 * @param buf       pointer to a memory block where to store the read data
 * @param btr       number of Bytes To Read
 * @param br        the real number of read bytes (Byte Read)
 * @return          LV_FS_RES_OK: no error or  any error from @lv_fs_res_t enum
 * ************************************************************************* */
static lv_fs_res_t fs_read(lv_fs_drv_t* drv, void* file_p, void* buf, uint32_t btr, uint32_t* br) {
  LV_UNUSED(drv);
  //LittleFile* lf = (LittleFile*)file_p;
  FatFile* lf = (FatFile*)file_p;

  *br = lf->file.read((uint8_t*)buf, btr);

  return (int32_t)(*br) < 0 ? LV_FS_RES_UNKNOWN : LV_FS_RES_OK;
}

/* ************************************************************************* *
 * Write into a file
 * @param drv       pointer to a driver where this function belongs
 * @param file_p    pointer to a file_t variable
 * @param buf       pointer to a buffer with the bytes to write
 * @param btw       Bytes To Write
 * @param bw        the number of real written bytes (Bytes Written). NULL if unused.
 * @return          LV_FS_RES_OK: no error or  any error from @lv_fs_res_t enum
 * ************************************************************************* */
static lv_fs_res_t fs_write(lv_fs_drv_t* drv, void* file_p, const void* buf, uint32_t btw, uint32_t* bw) {
  LV_UNUSED(drv);
  //LittleFile* lf = (LittleFile*)file_p;
  FatFile* lf = (FatFile*)file_p;
  *bw = lf->file.write((uint8_t*)buf, btw);
  return (int32_t)(*bw) < 0 ? LV_FS_RES_UNKNOWN : LV_FS_RES_OK;
}

/* ************************************************************************* *
 * Set the read write pointer. Also expand the file size if necessary.
 * @param drv       pointer to a driver where this function belongs
 * @param file_p    pointer to a file_t variable. (opened with fs_open )
 * @param pos       the new position of read write pointer
 * @param whence    tells from where to interpret the `pos`. See @lv_fs_whence_t
 * @return          LV_FS_RES_OK: no error or  any error from @lv_fs_res_t enum
 * ************************************************************************* */
static lv_fs_res_t fs_seek(lv_fs_drv_t* drv, void* file_p, uint32_t pos, lv_fs_whence_t whence) {
  LV_UNUSED(drv);
  SeekMode mode;
  if (whence == LV_FS_SEEK_SET)
    mode = SeekSet;
  else if (whence == LV_FS_SEEK_CUR)
    mode = SeekCur;
  else if (whence == LV_FS_SEEK_END)
    mode = SeekEnd;

  //LittleFile* lf = (LittleFile*)file_p;
  FatFile* lf = (FatFile*)file_p;
  lf->file.seek(pos, mode);
  return LV_FS_RES_OK;
}
