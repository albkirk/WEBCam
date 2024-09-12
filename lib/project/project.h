// WRITE HERE all the Project's logic / Flow.
// **** Libraries to include here ...
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "soc/soc.h"            //disable brownout problems
#include "soc/rtc_cntl_reg.h"   //disable brownout problems
#include "esp_http_server.h"
#include "sys/socket.h"
#include "microphone.h"
#include "EasyDDNS.h"

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

// CAM Sensor configuration
bool VFlip =             false;         // Vertical Flip image?
bool HMirror =           false;         // Hiorizontal mirror image?

// LED, PWM and others configuration 
#define RESOLUTION           8          // Duty cycle bits (8 -> 255) equalto FF
#define PWM_FREQ          1000          // PWM Frequency
#define CHANNELLED           1          // LED Channel to control the Light (Big White LED)
#define Big_LED_Pin          4

int Framerate =              8;         // Frame Rate Capture
bool Light =             false;         // [OFF / ON] Light switch
bool Light_Last =        false;         // [OFF / ON] Light switch  (Last state)

static bool OnAir =      false;         // OnAir switch
static bool OnAir_Last = false;         // OnAir switch  (Last state)
static bool Stream_VIDEO=false;         // Streaming Video
static bool Stream_AUDIO=false;         // Streaming Audio
static bool IMG_Capture =false;         // Capturing Image (JPEG or BMP)


// HTTP pre-defined strings
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_PART = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _CONTENT_TYPE_JPEG = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
static const char* _CONTENT_TYPE_AUDIO = "Content-Type: audio/wav\r\n";
static const char* _ACCESS_CONTROL = "Access-Control-Allow-Origin: *\r\n";

httpd_handle_t camera_httpd = NULL;
httpd_handle_t microphone_httpd = NULL;

typedef struct
{
    httpd_req_t *req;
    size_t len;
} jpg_chunking_t;

static camera_config_t cam_config;
static sensor_t * cam_sensor;


// **** Project code functions here ...
// Function to get the Client's IP address
void print_client_ip(httpd_req_t *req)
{
    int sockfd = httpd_req_to_sockfd(req);
    char ipstr[INET6_ADDRSTRLEN];
    struct sockaddr_in6 addr;   // esp_http_server uses IPv6 addressing
    socklen_t addr_size = sizeof(addr);
    
    if (getpeername(sockfd, (struct sockaddr *)&addr, &addr_size) < 0) {
        //ESP_LOGE(TAG, "Error getting client IP");
        telnet_println("Error getting client IP");
        return;
    }
    
    // Convert to IPv6 string
    //inet_ntop(AF_INET, &addr.sin6_addr, ipstr, sizeof(ipstr));
    //ESP_LOGI(TAG, "Client IP => %s", ipstr);

    // Convert to IPv4 string
    inet_ntop(AF_INET, &addr.sin6_addr.un.u32_addr[3], ipstr, sizeof(ipstr));
    //ESP_LOGI(TAG, "Client IP => %s", ipstr);
    telnet_println("Client IP: " + String(ipstr));

}


// Function that Handles the VIDEO STREAM
static esp_err_t video_stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];
  unsigned long Last_fb_capture = micros();
  Stream_VIDEO = true;
  CPU_Boost(true);
  int current_FR = Framerate;
  if (getRSSI()<-85) current_FR = int(Framerate / 4);
  else if (getRSSI()<-75) current_FR = int(Framerate / 2);
  else current_FR = Framerate;


  telnet_print("NEW HTTP GET request for VIDEO Stream from ");
  print_client_ip(req);  


  // Set the response type header
  res = httpd_resp_set_type(req, _STREAM_PART);
  if(res != ESP_OK){
    return res;
  }

  // Set "no-cache" on Cache-Control header
  res = httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  if (res != ESP_OK) {
    return res;
  }

  // Set the Access-Control-Allow-Origin header
  res = httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  if (res != ESP_OK) {
    return res;
  }
  /*
  char FRate[String(current_FR).length()];
  strcpy(FRate, String(current_FR).c_str());
  httpd_resp_set_hdr(req, "X-Framerate", FRate);
  */
  //httpd_resp_set_hdr(req, "X-Framerate", "");

  while(OnAir){
    if (micros() - Last_fb_capture < 1000000/current_FR) delayMicroseconds(1000000/current_FR - (micros() - Last_fb_capture) - 10);
    else telnet_println("FrameRate: " + String(current_FR) + " - Late Camera capture: " + String(micros() - Last_fb_capture));
    if (!IMG_Capture) {
        fb = esp_camera_fb_get();
        Last_fb_capture = micros();
        if (!fb) {
            telnet_println("Camera capture failed!");
            res = ESP_ERR_INVALID_RESPONSE;
            // shutting down the camera and reseting variables
            esp_camera_return_all();
            esp_camera_deinit();
            cam_sensor->reset(cam_sensor);
            fb = NULL;
            if(_jpg_buf) free(_jpg_buf);
            _jpg_buf = NULL;
            // restarting the camera
            esp_camera_init(&cam_config);
            cam_sensor = esp_camera_sensor_get();
            cam_sensor->set_vflip(cam_sensor, VFlip ? 1 : 0);
            cam_sensor->set_hmirror(cam_sensor, HMirror ? 1 : 0);
        }
        else {
            res = ESP_OK;
            if(fb->width > 400){
                if(fb->format != PIXFORMAT_JPEG){
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if(!jpeg_converted){
                        telnet_println("JPEG compression failed");
                        res = ESP_ERR_INVALID_RESPONSE;
                    }
                } else {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            }
        }
    }
    else res = ESP_ERR_INVALID_STATE;
    if(WIFI_state != WL_CONNECTED) res = ESP_ERR_WIFI_BASE; 
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _CONTENT_TYPE_JPEG, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if (httpd_req_to_sockfd(req) == -1) {
      telnet_println("VIDEO client disconnected");
      res = ESP_ERR_TIMEOUT;
    }

    if(res != ESP_OK){
      telnet_println("VIDEO RES msg: " + String(esp_err_to_name(res)));
      if (res != ESP_ERR_INVALID_RESPONSE) break;
    }
  }
  if(!OnAir) {
    telnet_println("Someone requested to stop streaming video");
    res = ESP_ERR_NOT_FINISHED;
  }
  esp_camera_return_all();
  fb = NULL;
  telnet_println("Finish VIDEO HTTP Stream request");
  Stream_VIDEO = false;
  if(!Stream_AUDIO) CPU_Boost(false);  // set to false to release CPU
  return res;
}


// Functions To handle CAPTURE of one JPEG Image
// the Aux function first...
static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len)
{
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if (!index)
    {
        j->len = 0;
    }
    if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK)
    {
        return 0;
    }
    j->len += len;
    return len;
}

 // ... and main function
static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    IMG_Capture = true;

    telnet_print("NEW HTTP GET request for Camera CAPTURE from ");
    print_client_ip(req);

   if (!Stream_VIDEO) {
        esp_camera_return_all();
        for (size_t i = 0; i < 2; i++) {  // Dummy loop to clean the buffer...                  
            fb = esp_camera_fb_get();
            esp_camera_fb_return(fb);
        }
        fb = esp_camera_fb_get();         // so this captures the image at the moment you click
    }
    if (!fb)
    {
        telnet_println("Camera capture failed - returning Error 500");
        httpd_resp_send_500(req);
        esp_camera_return_all();
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char ts[32];
    snprintf(ts, 32, "%ld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
    httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);

    if (fb->format == PIXFORMAT_JPEG) {
       res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    }
    else {
        jpg_chunking_t jchunk = {req, 0};
        res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
        httpd_resp_send_chunk(req, NULL, 0);
    }

    esp_camera_fb_return(fb);
    esp_camera_return_all();
    IMG_Capture = false;
    return res;
}

// Function that Handles the DOWNLOAD of a BMP Captured picture.
static esp_err_t bmp_handler(httpd_req_t *req) {
    camera_fb_t *fb = NULL;
    esp_err_t res = ESP_OK;
    IMG_Capture = true;

    if (!OnAir) {
        esp_camera_return_all();
        for (size_t i = 0; i < 2; i++) {  // Dummy loop to clean the buffer...                  
            fb = esp_camera_fb_get();
            esp_camera_fb_return(fb);
        }
        fb = esp_camera_fb_get();         // so this captures the image at the moment you click
    }
    if (!fb)
    {
        //log_e("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/x-windows-bmp");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.bmp");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    char ts[32];
    snprintf(ts, 32, "%ld.%06ld", fb->timestamp.tv_sec, fb->timestamp.tv_usec);
    httpd_resp_set_hdr(req, "X-Timestamp", (const char *)ts);


    uint8_t * buf = NULL;
    size_t buf_len = 0;
    bool converted = frame2bmp(fb, &buf, &buf_len);
    esp_camera_fb_return(fb);
    if(!converted){
        //log_e("BMP Conversion failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    res = httpd_resp_send(req, (const char *)buf, buf_len);
    free(buf);
    esp_camera_return_all();
    IMG_Capture = false;
    return res;
}


// Function to handle the AUDIO STREAM
static esp_err_t audio_stream_handler(httpd_req_t *req){
  esp_err_t res = ESP_OK;
  CPU_Boost(true);
 
  telnet_println("NEW HTTP GET request for AUDIO Stream");

  if (Stream_AUDIO) {
      telnet_println("Another AUDIO stream instance already in place.");
      httpd_resp_send_500(req);
      return ESP_ERR_NOT_SUPPORTED;
  }

  Stream_AUDIO = true;

  // Initialize the I2S microphone
  mic_i2s_init();

  // Set up the WAV File header
  WAVHeader wavHeader;
  initializeWAVHeader(wavHeader, sampleRate, bitsPerSample, numChannels);

  // Set the response type to audio/wav
  res = httpd_resp_set_type(req, "audio/wav");
  if(res != ESP_OK){
    return res;
  }

  // Set "no-cache" on Cache-Control header
  res = httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  if (res != ESP_OK) {
    return res;
  }

  // Set the Access-Control-Allow-Origin header
  res = httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  if (res != ESP_OK) {
    return res;
  }


  // Send the initial part of the WAV header
  httpd_resp_send_chunk(req, reinterpret_cast<const char*>(&wavHeader), sizeof(wavHeader));


  int32_t buffer[bufferSize/4];   // Buffer to store the audio data (32bit format)
  int32_t temp_buf = 0;           // temporary Buffer cell to manipulate audio sample
  int64_t amplified = 0;          // audio sample processed (digitally amplified)
  size_t bytesRead = 0;           // Number of bytes read, to be returned by the i2s_read() function.
  uint32_t totalDataSize = 0;   // Total size of audio data sent through the HTTP stream.
  
  while (OnAir) {
    if (httpd_req_to_sockfd(req) == -1) {
      //i2s_driver_uninstall(I2S_PORT);
      telnet_println("AUDIO client disconnected");
      break;
    }

    // Read audio data from I2S DMA
    res = i2s_read(I2S_PORT, buffer, bufferSize, &bytesRead, portMAX_DELAY);

    // code to Amplify the audio from 1x to 8x with no distortion. Beyond (ex.: 12x) expect peak limit.
    for (int i=0; i<bytesRead/4; i++) {
        temp_buf = buffer[i];
        amplified = int64_t(temp_buf) * 32;
        if (amplified > pow(2, 31)-1) amplified = pow(2, 31)-1;
        if (amplified < -pow(2, 31)) amplified = -pow(2, 31);
        temp_buf = int32_t(amplified);
        memcpy(&buffer[i], &temp_buf, sizeof(temp_buf)); // Copy so sign bits aren't interfered with somehow.
    }

    // Send audio data
    if (bytesRead > 0) {
      res = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(buffer), bytesRead);
      if (res != ESP_OK) {
          break;
      }
      totalDataSize += bytesRead;
      //telnet_println("Audio bytes sent: " + String(totalDataSize));
    }
  }
  if(!OnAir) {
    telnet_println("Stop capturing audio");
    res = ESP_ERR_TIMEOUT;
  }
  // Clean up resources
  i2s_driver_uninstall(I2S_PORT);
  telnet_println("Finish HTTP GET request AUDIO Stream. Total bytes sent: " + String(totalDataSize));
  Stream_AUDIO = false;
  if(!Stream_VIDEO) CPU_Boost(false);  // set to false to release CPU
  return res;
}


esp_err_t startCameraServer(){
  esp_err_t start_res;
  httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
  http_config.server_port = 2180;
  //http_config.lru_purge_enable=true;
  http_config.max_open_sockets=1;
  http_config.backlog_conn=0;
  //http_config.recv_wait_timeout=1;
  //http_config.send_wait_timeout=1;
  

  // URI handler for video stream
  httpd_uri_t video_uri = {
    .uri       = "/video.jpg",
    .method    = HTTP_GET,
    .handler   = video_stream_handler,
    .user_ctx  = NULL
  };


  httpd_uri_t capture_uri = {
    .uri       = "/capture.jpg",
    .method    = HTTP_GET,
    .handler   = capture_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t bmp_uri = {
    .uri       = "/bmp",
    .method    = HTTP_GET,
    .handler   = bmp_handler,
    .user_ctx  = NULL
  };


  /*
  // URI handler for audio stream
  httpd_uri_t audio_uri = {
    .uri = "/audio",
    .method = HTTP_GET,
    .handler = audio_stream_handler, // You need to implement this handler
    .user_ctx = NULL
  };
*/

  //Serial.printf("Starting web server on port: '%d'\n", http_config.server_port);
  start_res = httpd_start(&camera_httpd, &http_config);
  if ( start_res == ESP_OK) {
      httpd_register_uri_handler(camera_httpd, &video_uri);
      httpd_register_uri_handler(camera_httpd, &capture_uri);
      httpd_register_uri_handler(camera_httpd, &bmp_uri);
      telnet_println("VIDEO Stream Ready! Go to: http://" + WiFi.localIP().toString() + ":" + String(http_config.server_port) + String(video_uri.uri));
      telnet_println("Camera CAPTURE Ready! Go to: http://" + WiFi.localIP().toString() + ":" + String(http_config.server_port) + String(capture_uri.uri));
      telnet_println("BMP capture Ready! Go to: http://" + WiFi.localIP().toString() + ":" + String(http_config.server_port) + String(bmp_uri.uri));
      //httpd_register_uri_handler(camera_httpd, &audio_uri);
      //telnet_println("Audio Stream Ready! Go to: http://" + WiFi.localIP().toString() + ":" + String(http_config.server_port) + String(audio_uri.uri));

  }
  return start_res;
}



esp_err_t startAudioServer(){
    esp_err_t start_res;
    httpd_config_t http_config = HTTPD_DEFAULT_CONFIG();
    http_config.server_port = 2181; // Set the desired port number
    http_config.ctrl_port += 10;
    //http_config.lru_purge_enable=true;
    http_config.max_open_sockets=1;
    http_config.backlog_conn=0;
    //http_config.recv_wait_timeout=1;
    //http_config.send_wait_timeout=1;

    // URI handler for audio stream
    httpd_uri_t audio_uri = {
        .uri = "/audio",
        .method = HTTP_GET,
        .handler = audio_stream_handler, // You need to implement this handler
        .user_ctx = NULL
    };



  //Serial.printf("Starting web server on port: '%d'\n", http_config.server_port);
  start_res = httpd_start(&microphone_httpd, &http_config);
  if ( start_res == ESP_OK) {
      httpd_register_uri_handler(microphone_httpd, &audio_uri);
      telnet_println("MICROPHONE Stream Ready! Go to: http://" + WiFi.localIP().toString() + ":" + String(http_config.server_port) + String(audio_uri.uri));
  }
  return start_res;
}


void project_setup() {
  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); 

  // Slow down the CPU to minimize power consumption
  CPU_Boost(false);  // set to false to release CPU

  // BT Radio shoud be OFF
  telnet_println("BT radio: " + String(esp_bt_check()));

  // BIG White LED
  ledcSetup(CHANNELLED, PWM_FREQ, RESOLUTION);  // Duty cycle Range of values [o-PWMRANGE] equal to FF
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(Big_LED_Pin, CHANNELLED);
  ledcWrite(CHANNELLED, 0);

  // Use config.SLEEPTime values in seconds
  DateTime = ConvertTimeStamp(curUnixTime());
  if(NTP_Sync && (DateTime.hour > 20 || DateTime.hour <  7) ) {
      telnet_println("It's " +  String(DateTime.hour) + " hours. The sun should be high! : )");
      SLEEPTime = 3600*1000;
  }
  else {
      if(NTP_Sync) telnet_println("It's " +  String(DateTime.hour) + " hours. The sun should be high! : )");
      SLEEPTime = config.SLEEPTime * 1UL;  
  }

  // Initialize the camera settings
  cam_config.ledc_channel = LEDC_CHANNEL_0;
  cam_config.ledc_timer = LEDC_TIMER_0;
  cam_config.pin_d0 = Y2_GPIO_NUM;
  cam_config.pin_d1 = Y3_GPIO_NUM;
  cam_config.pin_d2 = Y4_GPIO_NUM;
  cam_config.pin_d3 = Y5_GPIO_NUM;
  cam_config.pin_d4 = Y6_GPIO_NUM;
  cam_config.pin_d5 = Y7_GPIO_NUM;
  cam_config.pin_d6 = Y8_GPIO_NUM;
  cam_config.pin_d7 = Y9_GPIO_NUM;
  cam_config.pin_xclk = XCLK_GPIO_NUM;
  cam_config.pin_pclk = PCLK_GPIO_NUM;
  cam_config.pin_vsync = VSYNC_GPIO_NUM;
  cam_config.pin_href = HREF_GPIO_NUM;
  cam_config.pin_sccb_sda = SIOD_GPIO_NUM;
  cam_config.pin_sccb_scl = SIOC_GPIO_NUM;
  cam_config.pin_pwdn = PWDN_GPIO_NUM;
  cam_config.pin_reset = RESET_GPIO_NUM;
  cam_config.xclk_freq_hz = 20000000;
  cam_config.pixel_format = PIXFORMAT_JPEG;
  //cam_config.grab_mode = CAMERA_GRAB_LATEST;
  //cam_config.frame_size = FRAMESIZE_XGA;    // Dependent of PSRAM availability
  //cam_config.jpeg_quality = 10;             // Dependent of PSRAM availability
  //cam_config.fb_count = 2;                  // Dependent of PSRAM availability

  //Check if the Board has PSRAM
  if(psramFound()){
    telnet_println("--> psramFound ! <--");
    cam_config.frame_size = FRAMESIZE_VGA;  // original : FRAMESIZE_XGA
    cam_config.jpeg_quality = 10;
    cam_config.fb_count = 2;
  } else {
    cam_config.frame_size = FRAMESIZE_SVGA;
    cam_config.jpeg_quality = 12;
    cam_config.fb_count = 1;
  }
  
  
  // Activate DDNS service
    /*
    List of supported DDNS providers:
    - "duckdns"
    - "noip"
    - "dyndns"
    - "dynu"
    - "enom"
    - "all-inkl"
    - "selfhost.de"
    - "dyndns.it"
    - "strato"
    - "freemyip"
    - "afraid.org"
  */
  EasyDDNS.service("noip");
  //EasyDDNS.client("camabrantes,hopto.org", "nralbuquerque", "NPegasop"); 
  EasyDDNS.client("all.ddnskey.com", "5qwae6k", "AbJG3ejfnd5n"); 

  // Get Notified when your IP changes
  EasyDDNS.onUpdate([&](const char* oldIP, const char* newIP) {
        telnet_println("EasyDDNS - IP Change Detected: " +  String(newIP));
        //mqtt_publish(mqtt_pathtele, "Public_IP", public_ip());
    }
  );



  // Start streaming web server
  if (WIFI_state == WL_CONNECTED) {
      // Camera initialization
      esp_err_t err = esp_camera_init(&cam_config);
      if (err != ESP_OK) {
          telnet_println("Camera init failed with error: [" + String(err,HEX) + "] - " + String(esp_err_to_name(err)));
          mqtt_publish(mqtt_pathtele, "Status", "CAM ERROR");
          deepsleep_procedure();
      }
      else {
          //mqtt_publish(mqtt_pathtele, "Public_IP", public_ip());
          mqtt_publish(mqtt_pathtele, "Local_IP", WiFi.localIP().toString());
          // Get ESP32-Cam sensor configuration, such vflip and hmirror features
          cam_sensor = esp_camera_sensor_get();
          cam_sensor->set_vflip(cam_sensor, VFlip ? 1 : 0);
          cam_sensor->set_hmirror(cam_sensor, HMirror ? 1 : 0);
          telnet_println("Configuring VFlip: " + String(VFlip) + " HMirror: " + String(HMirror));
          // Start HTTP servers
          startCameraServer();
          startAudioServer();
      }
  }
}

 
void project_loop() {
  if(Light != Light_Last) {
      if(Light) ledcWrite(CHANNELLED, 255);
      else ledcWrite(CHANNELLED, 0);
      mqtt_publish(mqtt_pathtele, "Light", String(Light));
      Light_Last = Light;
  }

  if(OnAir != OnAir_Last) {
      if(OnAir) {
        config.DEEPSLEEP = false;
      }
      else if(!IMG_Capture) {
        config.DEEPSLEEP = true;
        Light = false;            // Turning light OFF
      }
      else return;
      mqtt_publish(mqtt_pathtele, "OnAir", String(OnAir));
      OnAir_Last = OnAir;
  }

  /* //Logic flow to save energy depending on "sun" / time of the day. 
  if (NTP_Sync && millis() % 600000 < 5) {
      DateTime = ConvertTimeStamp(curUnixTime());
      if(DateTime.hour > 20 || DateTime.hour <  7) { if(!config.DEBUG)deepsleep_procedure(60); }  
      else telnet_println("It's " +  String(DateTime.hour) + " hours. The sun should be high! : )");
  }
  */
  
  if (millis() > 1800000) {
      mqtt_publish(mqtt_pathcomd, "OnAir", "", true);
      OnAir = false;      // Automatically disable OnAir after 30 minutes (to save battery).
      OnAir_Last = true;  // To force going through the OnAir logic flow.
  }
  // Update DDNS every 900 seconds
  EasyDDNS.update(900000);
}
