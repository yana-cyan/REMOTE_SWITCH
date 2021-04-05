#include <WiFi.h>
#include <WiFiUdp.h>

WiFiUDP udp;

#define LED_G 16                                    // IO16
#define LED_R 17                                    // IO17
#define SW_R  18                                    // IO18
#define SW_B  19                                    // IO19
#define SSR    5                                    // IO5

char* ssid = "REMOTE_SWITCH";                       // ｱｸｾｽﾎﾟｲﾝﾄ名(9文字以上)
char* password = "0x0123456789ABCDEF";              // ｱｸｾｽﾎﾟｲﾝﾄのﾊﾟｽﾜｰﾄﾞ(9文字以上)
char* STA_IP_address = "192.168.4.2";               // stationのIPｱﾄﾞﾚｽ
unsigned int AP_Port = 60011;                       // ｱｸｾｽﾎﾟｲﾝﾄのﾎﾟｰﾄ番号
unsigned int STA_port = 60012;                      // stationのﾎﾟｰﾄ番号
String Send_Str, Receive_Str;

// 割込で使う変数は、ｺﾝﾊﾟｲﾗｰの最適化によって消去されないように、volatile宣言をします。
volatile boolean RED_SW = false;
volatile boolean BLK_SW = false;
volatile boolean Send_Flg = false;
volatile byte timer0_cnt = 0;
//volatile unsigned long RESET_cnt = 0;
volatile byte retry_cnt = 0;
volatile byte RED_LED = 0;                          // 0:消灯 1:点灯 2:点滅 
volatile byte GREEN_LED = 0;                        // 0:消灯 1:点灯 2:点滅 
hw_timer_t *timer0 = NULL;                          // ﾀｲﾏｰ割込0

//***********************************
void Software_RESET() {
  Serial.println();
  Serial.println("Restarting...");
  ESP.restart();
}

//***********************************
void IRAM_ATTR LED_ON_OFF() {                       // ﾀｲﾏｰ割込で使う関数に(IRAM_ATTR)を
  switch (RED_LED) {
  case 0:
    digitalWrite(LED_R, LOW);
    break;
  case 1:
    digitalWrite(LED_R, HIGH);
    break;
  case 2:
    digitalWrite(LED_R, !digitalRead(LED_R));
  }
  switch (GREEN_LED) {
  case 0:
    digitalWrite(LED_G, LOW);
    break;
  case 1:
    digitalWrite(LED_G, HIGH);
    break;
  case 2:
    digitalWrite(LED_G, !digitalRead(LED_G));
  }
  timer0_cnt++;
//RESET_cnt++;
}

//***********************************
void Push_RED_SW() {
  if (RED_SW == false) {
    digitalWrite(SSR, HIGH);
    Serial.println("digitalWrite(SSR, HIGH)");
    RED_SW = true;
    BLK_SW = false;
    Send_Flg = false;
    timer0_cnt = 0;
    retry_cnt = 0;
    RED_LED = 2;                                    // 0:消灯 1:点灯 2:点滅 
  }
}

//***********************************
void Push_BLACK_SW() {
  if (BLK_SW == false) {
    digitalWrite(SSR, LOW);
    Serial.println("digitalWrite(SSR, LOW)");
    BLK_SW = true;
    RED_SW = false;
    Send_Flg = false;
    timer0_cnt = 0;
    retry_cnt = 0;
    RED_LED = 2;                                    // 0:消灯 1:点灯 2:点滅 
  }
}

//***********************************
void Button_Proc() {
  if (timer0_cnt > 10) {
    Send_Flg = false;
    retry_cnt++;
  }
  if (retry_cnt > 2) {
    if (RED_SW == true) {
      RED_LED = 1;                                 // 0:消灯 1:点灯 2:点滅 
    }
    if (BLK_SW == true) {
      RED_LED = 0;                                 // 0:消灯 1:点灯 2:点滅 
    }
    RED_SW = false;
    BLK_SW = false;
    Send_Flg = false;
    timer0_cnt = 0;
    retry_cnt = 0;
  }
  if ((RED_SW == true) && (Send_Flg == false)) {
    Send_Str = "ON(S)";
    sendUDP();
  }
  if ((BLK_SW == true) && (Send_Flg == false)) {
    Send_Str = "OFF(S)";
    sendUDP();
  }
}

//***********************************
void receiveUDP() {
  int packetSize = udp.parsePacket();
  if(packetSize > 0){
    Receive_Str = udp.readStringUntil('\r');       // ｺﾏﾝﾄﾞ文字列
    udp.flush();                                   // 残り文字列をｸﾘｱ
    Serial.println("Receive_Str: " + String(Receive_Str));// debug
    if ((RED_SW == false) && (BLK_SW == false)) {
      if (Receive_Str == "ON(C)") {
        digitalWrite(SSR, HIGH);
        Serial.println("digitalWrite(SSR, HIGH)");
        RED_LED = 1;                               // 0:消灯 1:点灯 2:点滅 
      }
      if (Receive_Str == "OFF(C)") {
        digitalWrite(SSR, LOW);
        Serial.println("digitalWrite(SSR, LOW)");
        RED_LED = 0;                               // 0:消灯 1:点灯 2:点滅 
      }
      Send_Str = Receive_Str;
      Receive_Str = "";                            // return key
      sendUDP();
    }
    if (Receive_Str == "ON(S)") {
      RED_LED = 1;                                 // 0:消灯 1:点灯 2:点滅 
      RED_SW = false;
    }
    if (Receive_Str == "OFF(S)") {
      RED_LED = 0;                                 // 0:消灯 1:点灯 2:点滅 
      BLK_SW = false;
    }
    Receive_Str = "";                              // return key
  }
}

//***********************************
void sendUDP() {
  udp.beginPacket(STA_IP_address, STA_port);       // UDP送信ﾊﾟｹｯﾄ初期化
  udp.print(Send_Str);                             // UDP送信ﾊﾟｹｯﾄに文字列書き込み
  udp.print('\r');                                 // UDP送信ﾊﾟｹｯﾄに文字列書き込み
  udp.endPacket();                                 // UDPのﾊﾟｹｯﾄ送信
  Send_Flg = true;
  timer0_cnt = 0;
  Serial.println("Send_Str   : " + String(Send_Str)); // debug
}

//************************************
void setupWiFiUDPserver(){
//WiFi.setOutputPower(10.0);                       // Wi-Fiの出力レベル設定(0.0～20.5dBm)
  Serial.println();
  Serial.println("Connecting to WiFi network: " + String(ssid));
  WiFi.disconnect(true, true);                     // WiFi config設定ﾘｾｯﾄ
  WiFi.softAP(ssid, password);                     // ｱｸｾｽﾎﾟｲﾝﾄのSSIDとpasswordの設定
  delay(500);
  udp.begin(AP_Port);                              // 自身のUDPﾎﾟｰﾄ(待ち受け)を開放
  IPAddress myIP = WiFi.softAPIP();                // 自身のIPｱﾄﾞﾚｽ
  Serial.println("*** WiFi AP Start ***");
  Serial.print("AP IP address: ");
  Serial.println(myIP);
  GREEN_LED = 1;                                    // 0:消灯 1:点灯 2:点滅 
}

//***********************************
void setup() {
  Serial.begin(115200);
  pinMode(SW_R, INPUT_PULLUP);                      // ﾌﾟｯｼｭﾎﾞﾀﾝ赤(入)
  pinMode(SW_B, INPUT_PULLUP);                      // ﾌﾟｯｼｭﾎﾞﾀﾝ黒(切)
  pinMode(LED_G, OUTPUT);                           // GREEN LED
  pinMode(LED_R, OUTPUT);                           // RED LED
  pinMode(SSR, OUTPUT);                             // SSR
  RED_LED = 0;                                      // 0:消灯 1:点灯 2:点滅
  GREEN_LED = 2;                                    // 0:消灯 1:点灯 2:点滅

  attachInterrupt(digitalPinToInterrupt(SW_R), Push_RED_SW, FALLING); 
  attachInterrupt(digitalPinToInterrupt(SW_B), Push_BLACK_SW, FALLING);

  // 使用ﾀｲﾏｰ（0～3）の指定と初期化
  // 周波数は、80,000,000 / 80 = 1,000,000 Hz
  timer0 = timerBegin(0, 80, true);             // ﾀｲﾏｰ番号0、分周比80(1ﾏｲｸﾛ秒)

  // 割込処理関数（ISR）の設定
  timerAttachInterrupt(timer0, &LED_ON_OFF, true);

  // タイマー動作の指定(trueで繰り返し)
  timerAlarmWrite(timer0, 500000, true);               // 500,000us(0.5s)ごとにLED点滅

  // ﾀｲﾏｰ開始
  timerAlarmEnable(timer0);

  setupWiFiUDPserver();
}

//***********************************
void loop() {
  Button_Proc();
  receiveUDP();
//if (RESET_cnt > 1209600) {                         // 7日1回再起動(120*60*24*7)
//  Software_RESET();
//}
//Serial.print("RESET_cnt: ");                       // debug
//Serial.println(RESET_cnt);                         // debug
  delay(1000);
}

//
