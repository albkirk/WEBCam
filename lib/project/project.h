// WRITE HERE all the Project's logic / Flow.
// **** Libraries to include here ...
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "soc/soc.h"            //disable brownout problems
#include "soc/rtc_cntl_reg.h"   //disable brownout problems
#include "esp_http_server.h"
#include "microphone.h"

// **** Project code definition here ...
// Camera PINs
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Camera Capture configuration 
#define RESOLUTION           8          // Duty cycle bits (8 -> 255) equalto FF
#define PWM_FREQ          1000          // PWM Frequency
#define CHANNELLED           1          // LED Channel to control the Light (Big White LED)
#define Big_LED_Pin          4
bool Light = false;                     // [OFF / ON] Light switch
bool Light_Last = false;                // [OFF / ON] Light switch  (Last state)

static bool OnAir = true;               // OnAir switch
static bool OnAir_Last = false;         // OnAir switch  (Last state)

// HTTP pre-defined strings
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;

// **** Project code functions here ...

// to Handle the VIDEO STREAM
static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  while(OnAir){
    fb = esp_camera_fb_get();
    if (!fb) {
      telnet_println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      if(fb->width > 400){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            telnet_println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;
    }
    //Serial.printf("MJPG: %uB\n",(uint32_t)(_jpg_buf_len));
  }
  if(!OnAir) {
    telnet_println("Stop capturing video");
    res = ESP_ERR_TIMEOUT;
  }
  return res;
}

void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 2180;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
  
  //Serial.printf("Starting web server on port: '%d'\n", config.server_port);
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
      httpd_register_uri_handler(stream_httpd, &index_uri);
      telnet_println("Camera Stream Ready! Go to: http://" + WiFi.localIP().toString() + ":" + String(config.server_port));
  }
}

/*
// To handle the AUDIO STREAM
void handleAudioStream() {

  mic_i2s_init();

  WAVHeader wavHeader;
  initializeWAVHeader(wavHeader, sampleRate, bitsPerSample, numChannels);

  // Get access to the client object
  WiFiClient Audioclient = Audioserver.client();

  // Send the 200 OK response with the headers
  Audioclient.print("HTTP/1.1 200 OK\r\n");
  Audioclient.print("Content-Type: audio/wav\r\n");
  Audioclient.print("Access-Control-Allow-Origin: *\r\n");
  Audioclient.print("\r\n");

  // Send the initial part of the WAV header
  Audioclient.write(reinterpret_cast<const uint8_t*>(&wavHeader), sizeof(wavHeader));

  uint8_t buffer[bufferSize];
  size_t bytesRead = 0;
  //uint32_t totalDataSize = 0; // Total size of audio data sent

  while (true) {
    if (!Audioclient.connected()) {
      //i2s_driver_uninstall(I2S_PORT);
      Serial.println("Audioclient disconnected");
      break;
    }
    // Read audio data from I2S DMA
    i2s_read(I2S_PORT, buffer, bufferSize, &bytesRead, portMAX_DELAY);

    // Send audio data
    if (bytesRead > 0) {
      Audioclient.write(buffer, bytesRead);
      //totalDataSize += bytesRead;
      //Serial.println(totalDataSize);
    }
  }
}
*/



void project_setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  //Serial.begin(115200);
  Serial.setDebugOutput(false);

  // BIG White LED
  ledcSetup(CHANNELLED, PWM_FREQ, RESOLUTION);  // Duty cycle Range of values [o-PWMRANGE] equal to FF
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Big_LED_Pin, CHANNELLED);
  ledcWrite(CHANNELLED, 0);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //config.frame_size = FRAMESIZE_XGA;

  
  if(psramFound()){
    telnet_println("--> psramFound ! <--");
    config.frame_size = FRAMESIZE_XGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  // Camera initialization
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }


  
  // Start streaming web server
  //startCameraServer();
}

void project_loop() {
  if(Light != Light_Last) {
      if(Light) ledcWrite(CHANNELLED, 255);
      else ledcWrite(CHANNELLED, 0);
      mqtt_publish(mqtt_pathtele, "Light", String(Light));
      Light_Last = Light;
  }

  if(OnAir != OnAir_Last) {
      if(OnAir) startCameraServer();
      else telnet_println("Stopped HTTP stream with error: " + String(httpd_stop(stream_httpd)) );
      mqtt_publish(mqtt_pathtele, "OnAir", String(OnAir));
      Light = false;
      OnAir_Last = OnAir;
  }


}
