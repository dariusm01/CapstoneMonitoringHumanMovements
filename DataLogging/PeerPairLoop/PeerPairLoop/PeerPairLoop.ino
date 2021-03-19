//Wifi Libraries
#include <WiFi.h>
#include <esp_now.h>

#define NUMMAIN 20
#define CHANNEL 1
#define PRINTSCANRESULTS 0
#define PINGS 12
#define CLOCKDATA 8

esp_now_peer_info_t main[NUMMAIN] = {};
int mainCnt = 0;

bool registered = false;


typedef struct sensorInfo {
  unsigned long int mSentClk;
  unsigned long int pClk;
  uint8_t packet;
  bool synced;
} sensorInfo;

sensorInfo recvData;

typedef struct deltaClk {
  bool sign;
  unsigned long int clk;
} deltaClk;

deltaClk dClkResult;

//Paired
void setup() {
  Serial.begin(115200);
  Serial.println("Syncing Process");
  //Set device in AP mode to begin with
  WiFi.mode(WIFI_AP);
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Slave in AP Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info.
  esp_now_register_recv_cb(OnDataRecv);
  //esp_now_register_send_cb(OnDataSent);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (!registered) {
    scanForMain();
    if (mainCnt > 0) {
      registered = true;
      managePeer();
    }
  }
}




// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}


// config AP SSID
void configDeviceAP() {
  String Prefix = "Peer:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}


// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.print("Data recieved size: ");
  Serial.println(data_len);
  if (data_len == PINGS) {
    if (registered) {
      const uint8_t *main_addr = main[0].peer_addr;
      memcpy(&recvData, data, sizeof(recvData));
      if (!recvData.synced) {
        recvData.pClk = millis();
        esp_err_t result = esp_now_send(main_addr, (uint8_t*) &recvData, sizeof(recvData));
        if (result == ESP_OK) {
          Serial.println("Successful valid response");
        }
      }
      else {
        //DONT DELETE THIS PLEASE
        /*
          Serial.println("Recieved Synced Signal");
          Serial.println("Turning off WiFi device");
          if (esp_now_deinit() == ESP_OK) {
            Serial.println("WiFi driver stopped");
            //WiFi.mode(WIFI_OFF);
          }
          WiFi.mode(WIFI_OFF);
          WiFi.disconnect();
        */
      }
    }
  }
  else if (data_len == CLOCKDATA) {
    memcpy(&dClkResult, data, sizeof(dClkResult));
    Serial.print("Clock Difference: ");
    if (!dClkResult.sign) {
      Serial.print("-");
    }
    Serial.println(dClkResult.clk);
  }


}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Data sent successfully");
  }
  else {
    Serial.println("Failed to send!");
  }
}


// Scan for peers in AP mode
void scanForMain() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset peers
  memset(main, 0, sizeof(main));
  mainCnt = 0;
  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `Main`
      if (SSID.indexOf("Main") == 0) {
        // SSID of interest
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the Main
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            main[mainCnt].peer_addr[ii] = (uint8_t) mac[ii];

          }
        }
        main[mainCnt].channel = CHANNEL; // pick a channel
        main[mainCnt].encrypt = 0; // no encryption
        mainCnt++;
      }
    }
  }

  if (mainCnt > 0) {
    Serial.print(mainCnt); Serial.println(" Main(s) found, processing..");
  } else {
    Serial.println("No Main Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

void managePeer() {
  if (mainCnt > 0) {
    for (int i = 0; i < mainCnt; i++) {
      Serial.print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        Serial.print((uint8_t) main[i].peer_addr[ii], HEX);
        if (ii != 5) Serial.print(":");
      }
      Serial.print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(main[i].peer_addr);
      if (exists) {
        // Peer already paired.
        Serial.println("Already Paired");
      } else {
        // Peer not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(&main[i]);
        if (addStatus == ESP_OK) {
          // Pair success
          Serial.println("Pair success");
          //create sensorSync struct

        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          Serial.println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          Serial.println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          Serial.println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          Serial.println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          Serial.println("Peer Exists");
        } else {
          Serial.println("Not sure what happened");
        }
        delay(100);
      }
    }
  } else {
    // No peer found to process
    Serial.println("No Peers found to process");
  }
}
