#include <Wire.h>
#include "Adafruit_HTU21DF.h"

// Task Share Variables Info
// xDHT_temp   : ข้อมูล อุณหภูมิ ล่าสุด สำหรับ task อื่นนำไปใช้ได้ 
// xDHT_humid  : ข้อมูล ความชื้น ล่าสุด สำหรับ task อื่นนำไปใช้ได้ 

Adafruit_HTU21DF htu = Adafruit_HTU21DF();

void DHT_func(void*) {
  //----พื้นที่สำหรับประกาศตัวแปรที่ใช้ภายใน task นี้เท่านั้น----
  //-----------------------------------------------
  VOID SETUP() {                       // VOID SETUP() ใน task ใช้พิมพ์ใหญ่
    Serial.println("[HTU] task begin");
    htu.begin();
  }

  VOID LOOP() {                       // VOID LOOP() ใน task ใช้พิมพ์ใหญ่
    float t = NAN; float h = NAN;  
    while( isnan(t) || isnan(h) ) {
      t = htu.readTemperature();
      h = htu.readHumidity();
    }
    xDHT_temp = t;  xDHT_humid = h;  // ค่าที่อ่านได้ถูกต้องแล้ว ค่อย copy ไปไว้ที่ ตัวแปรค่าล่าสุด
    Serial.printf("[DHT] temp : %.2f C\thumid : %.2f %%\n", xDHT_temp, xDHT_humid);
    
    DELAY(1000);              // วนรอบถัดไปที่จะอ่าน sensor อีกครั้ง
  }
}
