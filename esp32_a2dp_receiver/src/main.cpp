#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <esp_hf_client_api.h>
#include "soc/rtc.h"
#include <Wire.h>
#include "BluetoothA2DPSink.h"
#include <BlockNot.h>

BluetoothA2DPSink a2dp_sink;

// -------------------CONFIGURATION-------------------
// Enable steering wheel controls via CAN
#define USE_CAN

// Enable auto-switching to Aux with the help of SID CAN-UART
// Auto-switching only happens when we are connected to Bluetooth
// Requires USE_CAN
#define USE_SID_UART

// Enable Bluetooth auto-reconnect
#define BT_AUTO_RECONNECT

// Start playing audio immediately after connecting/reconnecting to phone
// If using USE_SID_UART, audio plays only when we are in Aux mode
#define RESUME_AUDIO_ON_CONNECTION

// Swap left/right audio channels due to this bug: https://github.com/espressif/esp-idf/issues/3399
#define SWAP_LR
// ---------------------------------------------------

#ifdef USE_CAN
struct can_frame canMsg;
struct can_frame src_frame;
MCP2515 mcp2515(5);
#endif

#ifdef USE_SID_UART
// SID CAN UART pins
#define RXD2 33
#define TXD2 32

// Aux mode message
const char auxPlayMsg[] = {0xFF,0x0,0x1,0x8,0x11,0x0,0x1,0x0,0x2,0x94,0x2,0x0,0xB2,0x2,0xFF};

int auxPlayMatchIndex = 0;
int doOnce = 0;
BlockNot ibusDelay(1000);
#endif

// I2S Pins
// ws_io_num => GPIO 25,
// data_out_num => GPIO 22
// bck_io_num => GPIO 26,
int mutePin = 2;

int inCall;

// Metadata
// 0x1 - Song Title
// 0x2 - Artist
// 0x4 - Album
void avrc_metadata_callback(uint8_t data1, const uint8_t *data2) {
  Serial.printf("AVRC metadata rsp: attribute id 0x%x, %s\n", data1, data2);
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr){
  if(state == ESP_A2D_CONNECTION_STATE_CONNECTED){
    Serial.println("Bluetooth connected!");
#ifndef USE_SID_UART
#ifdef RESUME_AUDIO_ON_CONNECTION
    a2dp_sink.play();
    Serial.println("Playing audio");
#endif
#endif
  }
  else{
    Serial.println("Bluetooth not connected!");
  }
}

// HFP
static void bt_app_hf_client_audio_open(esp_hf_client_audio_state_t state)
{
    // When in call with mSBC codec - set DAC and Mic to 32bit 16kHz mono
    if (state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC)
    {
      i2s_set_clk(I2S_NUM_0, 16000, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
      i2s_set_clk(I2S_NUM_1, 16000, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
    }
    
    // When in call with CVSD codec - set DAC and Mic to 32bit 8kHz mono
    else if (state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED)
    {
      i2s_set_clk(I2S_NUM_0, 8000, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
      i2s_set_clk(I2S_NUM_1, 8000, I2S_BITS_PER_SAMPLE_32BIT, I2S_CHANNEL_MONO);
    }
}

static void bt_app_hf_client_audio_close(void)
{
    // When call is done - reset DAC to default of 16bit 44.1kHz stereo for A2DP audio
    i2s_set_clk(I2S_NUM_0, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_STEREO);
}

// 16 and 32 bit mic buffers
// Buffer size is determined by the SCO data size, which is 120
// DMA buffer is set to the same value, but DMA buffer size also works with different values.
uint32_t mic_buf[120];
uint16_t mic_buf_16bit[120];
uint32_t dac_buf[120];
uint16_t dac_buf_16bit[120];

static uint32_t bt_app_hf_client_outgoing_cb(uint8_t p_buf[120], uint32_t mic_sz)
{
    // Read data from I2S mic
    size_t i2s_bytes_read = 0;
    i2s_read(I2S_NUM_1, mic_buf, mic_sz * sizeof(uint16_t), &i2s_bytes_read, 0);

    // Convert 32 bit mic samples to 16 bit
    if (mic_sz > 0)
    {
        for (int i = 0; i < mic_sz; i++)
        {
            // in theory we should shift to the right by 16 bits, but MEMS microphones have a very
            // high dynamic range, so if we shift all the way we lose a lot of signal.
            // Therefore we shift by 11 bits.
            mic_buf_16bit[i] = mic_buf[i] >> 11;
        }
    }

    // Copy our local 16 bit buffer into the HFP outgoing callback buffer
    memcpy((uint8_t *)p_buf, (uint16_t *)mic_buf_16bit, (uint32_t)mic_sz);

    return (uint32_t)mic_sz;
}

static void bt_app_hf_client_incoming_cb(const uint8_t buf[120], uint32_t dac_sz)
{
    // Copy HFP incoming callback buffer buffer into our local 16 bit buffer
    memcpy((uint16_t *)dac_buf_16bit, (uint8_t *)buf, (uint32_t)dac_sz);
    
    // Convert 16 bit DAC samples to 32 bit
    if (dac_sz > 0)
    {
        for (int i = 0; i < dac_sz; i++)
        {
            // Shift by 16 bits
            dac_buf[i] = dac_buf_16bit[i] << 16;
        }
    }

    // Write to I2S
    size_t i2s_bytes_written = 0;
    i2s_write(I2S_NUM_0, dac_buf, dac_sz * sizeof(uint16_t), &i2s_bytes_written, 0);

    // Data is ready
    esp_hf_client_outgoing_data_ready();
}

static const char *BT_HF_TAG = "BT_HF";
const char *c_hf_evt_str[] = {
    "CONNECTION_STATE_EVT",              /*!< connection state changed event */
    "AUDIO_STATE_EVT",                   /*!< audio connection state change event */
    "VR_STATE_CHANGE_EVT",                /*!< voice recognition state changed */
    "CALL_IND_EVT",                      /*!< call indication event */
    "CALL_SETUP_IND_EVT",                /*!< call setup indication event */
    "CALL_HELD_IND_EVT",                 /*!< call held indicator event */
    "NETWORK_STATE_EVT",                 /*!< network state change event */
    "SIGNAL_STRENGTH_IND_EVT",           /*!< signal strength indication event */
    "ROAMING_STATUS_IND_EVT",            /*!< roaming status indication event */
    "BATTERY_LEVEL_IND_EVT",             /*!< battery level indication event */
    "CURRENT_OPERATOR_EVT",              /*!< current operator name event */
    "RESP_AND_HOLD_EVT",                 /*!< response and hold event */
    "CLIP_EVT",                          /*!< Calling Line Identification notification event */
    "CALL_WAITING_EVT",                  /*!< call waiting notification */
    "CLCC_EVT",                          /*!< listing current calls event */
    "VOLUME_CONTROL_EVT",                /*!< audio volume control event */
    "AT_RESPONSE",                       /*!< audio volume control event */
    "SUBSCRIBER_INFO_EVT",               /*!< subscriber information event */
    "INBAND_RING_TONE_EVT",              /*!< in-band ring tone settings */
    "LAST_VOICE_TAG_NUMBER_EVT",         /*!< requested number from AG event */
    "RING_IND_EVT",                      /*!< ring indication event */
};
// esp_hf_client_callsetup_t
const char *c_call_setup_str[] = {
    "NONE",
    "INCOMING",
    "OUTGOING_DIALING",
    "OUTGOING_ALERTING"
};
// esp_hf_client_audio_state_t
const char *c_audio_state_str[] = {
    "disconnected",
    "connecting",
    "connected",
    "connected_msbc",
};

void bt_hf_client_cb(esp_hf_client_cb_event_t event, esp_hf_client_cb_param_t *param)
{
    if (event <= ESP_HF_CLIENT_RING_IND_EVT) {
        ESP_LOGE(BT_HF_TAG, "APP HFP event: %s", c_hf_evt_str[event]);
    } else {
        ESP_LOGE(BT_HF_TAG, "APP HFP invalid event %d", event);
    }

    switch (event) {
    case ESP_HF_CLIENT_CIND_CALL_SETUP_EVT:
        ESP_LOGE(BT_HF_TAG, "--Call setup indicator %s",
                 c_call_setup_str[param->call_setup.status]);
        // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_hf_defs.html#_CPPv426esp_hf_call_setup_status_t
        //
        // Call status for answer/reject call logic
        // While in call, the status is ESP_HF_CALL_SETUP_STATUS_INCOMING (1)
        // While there is no call, the status is ESP_HF_CALL_SETUP_STATUS_IDLE (0)
        if(param->call_setup.status == 1) inCall = 1;
        if(param->call_setup.status == 0) inCall = 0;
        break;
    case ESP_HF_CLIENT_AUDIO_STATE_EVT:
        ESP_LOGI(BT_HF_TAG, "--audio state %s",
                 c_audio_state_str[param->audio_stat.state]);
			  // If in call, enable HFP audio
        if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED ||
            param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_CONNECTED_MSBC) {
            bt_app_hf_client_audio_open(param->audio_stat.state);
            esp_hf_client_register_data_callback(bt_app_hf_client_incoming_cb,
                                                 bt_app_hf_client_outgoing_cb);
        // If not in call, set default DAC A2DP config
        } else if (param->audio_stat.state == ESP_HF_CLIENT_AUDIO_STATE_DISCONNECTED) {
				    bt_app_hf_client_audio_close();
        }
        break;
    }
}

void setup() {
  // Enable serial
  Serial.begin(115200);
  Serial.println("BOOTED!");

  pinMode(mutePin, OUTPUT);

  #ifdef USE_CAN
  // Setup CAN
  SPI.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_33KBPS, CAN_CLOCK::MCP_8MHZ);
  mcp2515.setNormalMode();
#endif

#ifdef USE_SID_UART
  // Steering Wheel SRC button CAN commands
  src_frame.can_id = 0x290;
  src_frame.can_dlc = 5;
  src_frame.data[0] = 0x00;
  src_frame.data[1] = 0x00;
  src_frame.data[2] = 0x19;
  src_frame.data[3] = 0x03;
  src_frame.data[4] = 0x00;

  // Enable Serial2 for SID CAN-UART
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  //pinMode(TXD2, OUTPUT);
#endif

  // Setup UDA1334A DAC
  static const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 44100,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
      .communication_format = (i2s_comm_format_t) (I2S_COMM_FORMAT_STAND_I2S),
      .intr_alloc_flags = 0, // default interrupt priority
      .dma_buf_count = 8,
      .dma_buf_len = 120,
      .use_apll = true,
      .tx_desc_auto_clear = true
    };
  a2dp_sink.set_i2s_config(i2s_config);

  // Setup INMP441 Microphone
  const i2s_config_t i2s_mic_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 8000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ALL_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 120,
    .use_apll = true
  };
  i2s_driver_install(I2S_NUM_1, &i2s_mic_config, 0, NULL);

  const i2s_pin_config_t mic_pin_config = {
    .bck_io_num = 16,
    .ws_io_num = 17,
    .data_out_num = -1,
    .data_in_num = 21
  };
  i2s_set_pin(I2S_NUM_1, &mic_pin_config);

  // Enable auto-reconnect
  // See for details: https://github.com/pschatzmann/ESP32-A2DP/wiki/Auto-Reconnect
  #ifdef BT_AUTO_RECONNECT
  a2dp_sink.set_auto_reconnect(true);
  #endif

  // Swap audio channels due to this bug: https://github.com/espressif/esp-idf/issues/3399
  #ifdef SWAP_LR
  a2dp_sink.set_swap_lr_channels(true);
  #endif

  // Bluetooth power level
  // Setting to max of +9 dBm should improve auto reconnect, as the BT range should be longer with higher TX power
  esp_bredr_tx_power_set(ESP_PWR_LVL_P9, ESP_PWR_LVL_P9);

  // Start the A2DP sink
  a2dp_sink.start("Saab 9-3");  

  // Metadata
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);

  // Connection change callback
  #ifdef RESUME_AUDIO_ON_CONNECTION
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
  #endif

  // HFP
  esp_hf_client_register_callback(bt_hf_client_cb);
  esp_hf_client_init();
  Serial.println("BT STACK UP!");
}

bool muteState = false;
bool playingStateRequest = true;
bool playingState = false;

String lastContent = "";

long playTime = 0;

void loop() {
  // If not connected to Bluetooth
  if (a2dp_sink.get_connection_state() != ESP_A2D_CONNECTION_STATE_CONNECTED)
  {
    // Set digital mute ON and do not handle events.
    digitalWrite(mutePin, 1);
    
    muteState = true;
    playingState = false;

#ifdef USE_SID_UART
    // If not connected to Bluetooth, reset Aux auto-switch loop
    doOnce = 0;
#endif

    return;
  }
  // If connected to Bluetooth
  else
  {
#ifdef USE_SID_UART
    if(Serial2.available()){
      // read the incoming byte:
      char d = Serial2.read();

      if(doOnce == 0){
        // Check for match with the aux 'play' msg
        if(d == auxPlayMsg[auxPlayMatchIndex]){
          // If we are done matching
          if(auxPlayMatchIndex == 14){
            // Matched the last char
            Serial.println("Aux mode detected!");

            // Reset aux play match
            auxPlayMatchIndex = 0;

            // Play audio
#ifdef RESUME_AUDIO_ON_CONNECTION
            a2dp_sink.play();
            Serial.println("Playing audio");
#endif

            // Stop the loop
            doOnce = 1;
          }else{
            auxPlayMatchIndex++;
          }
        }else{
          auxPlayMatchIndex = 0;

          // Send SRC button press message via I-BUS every second until we are in Aux mode
          if(ibusDelay.TRIGGERED){
            Serial.println("Sending SRC button command via I-Bus!");
            mcp2515.sendMessage(&src_frame);
          }
        }
      }
    }
#endif

    // Wait until playing again
    if (playingState)
    {
      if(muteState){
        muteState = false;
        // Reinit to prevent glitchy noise
        // Set digital mute off.
        digitalWrite(mutePin, 0);
      }
    }else{
      muteState = true;
    }
  }



// Read messages
// 
// CAN database can be found here:
// https://www.trionictuning.com/forum/viewtopic.php?f=46&t=5763
#ifdef USE_CAN
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
  {
    if (canMsg.can_id == 0x290)
    {
      int btnData = canMsg.data[3];

      switch (btnData)
      {
      case 0x0:
        // Nothing!
        break;
      case 0x5:
        Serial.println("NEXT");
        a2dp_sink.next();
        //NEXT
        break;
      case 0x6:
        Serial.println("PREV");
        a2dp_sink.previous();
        //PREV
        break;
      case 0x11:
        Serial.println("PLAY/PAUSE");
        if (playingStateRequest)
        {
          a2dp_sink.pause();
        }
        else
        {
          a2dp_sink.play();
        }
        // Update the boolean
        playingStateRequest = !playingStateRequest;
        //PLAY/PAUSE
        break;

      // ESP HF Client
      // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/bluetooth/esp_hf_client.html
      //
      // Voice Recognition
      case 0x04:
        Serial.println("VOICE REQ");
        esp_hf_client_start_voice_recognition();
        break;
      // Answer/reject phone call
      case 0x12:
        if(inCall == 0)
        {
          Serial.println("ANSWER CALL");
          esp_hf_client_answer_call();
          break;
        }
        if(inCall == 1)
        {
          Serial.println("REJECT CALL");
          esp_hf_client_reject_call();
          break;
        }
      }
    }
  }

  // If playing content
  if (a2dp_sink.get_audio_state() == 2)
  {
    playingState = true;
    // Determine full screen refresh
    String currentContent = "";

    //currentContent = a2dp_sink.audio_trackname + " - " + a2dp_sink.audio_trackalbum + " by " + a2dp_sink.audio_trackartist + " : " + a2dp_sink.audio_tracklength;

    // Do full refresh
    if (currentContent != lastContent)
    {
      lastContent = currentContent;
      Serial.println(currentContent);
    }
  }
  else
  {
    playingState = false;
  }
  #endif
}