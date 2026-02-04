#include <WiFi.h>
#include <WiFiMulti.h>
#include <WebServer.h>
#include "driver/adc.h"
#include <time.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <WiFiClient.h>
#include <MySQL_Connection.h>
#include <MySQL_Cursor.h>

#include <LittleFS.h>
#include <ESPmDNS.h>

#include <esp_chip_info.h>
#include <esp_system.h>
#include <esp_mac.h>

// ---------------- Compile-time sanity checks ----------------
#if defined(__has_include)
  #if !__has_include(<Adafruit_BME280.h>)
    #error "Missing library: Adafruit BME280. Install 'Adafruit BME280 Library' and 'Adafruit Unified Sensor'."
  #endif
  #if !__has_include(<MySQL_Connection.h>)
    #error "Missing library: MySQL_Connector_Arduino. Install 'MySQL_Connector_Arduino'."
  #endif
#endif


const char* SKETCH_FILE = "ESP32_Vdet_realvoltage_OK_4.feb26.ino";
const char* SKETCH_BUILD = __DATE__ " " __TIME__;

// Forward declarations (needed because some helpers are used before definition)
static void addAPs();
static bool hasInternet(uint32_t timeout_ms = 2500);

// ---------------- Auto SoftAP fallback (when no Wi-Fi SSIDs are configured) ----------------
static const char* AUTOAP_SSID = "EspStation";
static const char* AUTOAP_PASS = "123456";
static bool g_softAPActive = false;


// ---------------- Wi-Fi (fallback with Internet check) ----------------
WiFiMulti wifiMulti;

// -------- Wi-Fi AP list (persistent) --------
static const int MAX_WIFI_APS = 6;
struct WifiAPEntry {
  char ssid[33];
  char pass[65];
};
WifiAPEntry g_wifiAps[MAX_WIFI_APS];
int g_wifiApCount = 0;

// Default APs (used if nothing saved in NVS yet)
static const WifiAPEntry DEFAULT_APS[] = {
  {"Abc", "1234"},
  {"CDE",   "56789"},

};

// Wi-Fi reconnect state machine (apply new AP list without reboot)
static bool g_wifiReconnectPending = false;
static uint32_t g_wifiReconnectStartMs = 0;
static uint32_t g_wifiLastInternetCheckMs = 0;
static const uint32_t WIFI_RECONNECT_TIMEOUT_MS = 60000; // 60s
static const uint32_t WIFI_INTERNET_RECHECK_MS  = 2000;  // 2s

static void resetWifiMulti() {
  wifiMulti = WiFiMulti(); // reset stored APs
  addAPs();
}

static void startWifiReconnect() {
  g_wifiReconnectPending = true;
  g_wifiReconnectStartMs = millis();

  if (g_softAPActive) {
    WiFi.softAPdisconnect(true);
    g_softAPActive = false;
  }

  WiFi.disconnect(true, true);
  resetWifiMulti();
}


static void processWifiReconnect() {
  if (!g_wifiReconnectPending) return;

  // timeout
  if (millis() - g_wifiReconnectStartMs > WIFI_RECONNECT_TIMEOUT_MS) {
    Serial.println("[WiFi] Reconnect timeout");
    g_wifiReconnectPending = false;
    return;
  }

  // try connect
  if (wifiMulti.run() == WL_CONNECTED) {
    // Check Internet periodically; if no Internet, drop and try next AP
    if (millis() - g_wifiLastInternetCheckMs >= WIFI_INTERNET_RECHECK_MS) {
      g_wifiLastInternetCheckMs = millis();
      if (hasInternet()) {
        Serial.printf("[WiFi] Connected: %s IP=%s (Internet OK)\n",
                      WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
        g_wifiReconnectPending = false;
      } else {
        Serial.println("[WiFi] Connected but no Internet; trying next SSID");
        WiFi.disconnect(true, true);
        delay(200);
      }
    }
  }
}

static void loadWifiApsFromNVS() {
  Preferences p;
  p.begin("adcmon", true);
  int n = (int)p.getUInt("wifi_n", 0);
  if (n < 0) n = 0;
  if (n > MAX_WIFI_APS) n = MAX_WIFI_APS;

  g_wifiApCount = 0;

  for (int i = 0; i < n; ++i) {
    char kS[16], kP[16];
    snprintf(kS, sizeof(kS), "wifiS%d", i);
    snprintf(kP, sizeof(kP), "wifiP%d", i);
    String ss = p.getString(kS, "");
    String pw = p.getString(kP, "");
    ss.trim();
    // allow empty password (open network) but require SSID
    if (ss.length() == 0) continue;
    ss.toCharArray(g_wifiAps[g_wifiApCount].ssid, sizeof(g_wifiAps[g_wifiApCount].ssid));
    pw.toCharArray(g_wifiAps[g_wifiApCount].pass, sizeof(g_wifiAps[g_wifiApCount].pass));
    g_wifiApCount++;
    if (g_wifiApCount >= MAX_WIFI_APS) break;
  }
  p.end();

  // If nothing stored in Settings, keep list empty (auto SoftAP will be started).
}

static void saveWifiApsToNVS() {
  Preferences p;
  p.begin("adcmon", false);
  p.putUInt("wifi_n", (uint32_t)g_wifiApCount);
  for (int i = 0; i < MAX_WIFI_APS; ++i) {
    char kS[16], kP[16];
    snprintf(kS, sizeof(kS), "wifiS%d", i);
    snprintf(kP, sizeof(kP), "wifiP%d", i);
    if (i < g_wifiApCount) {
      p.putString(kS, g_wifiAps[i].ssid);
      p.putString(kP, g_wifiAps[i].pass);
    } else {
      // clear unused slots
      p.remove(kS);
      p.remove(kP);
    }
  }
  p.end();
}

static void addAPs() {
  // rebuild wifiMulti list each time
  wifiMulti = WiFiMulti();
  if (g_wifiApCount == 0) loadWifiApsFromNVS();
  for (int i = 0; i < g_wifiApCount; ++i) {
    wifiMulti.addAP(g_wifiAps[i].ssid, g_wifiAps[i].pass);
  }
}

static bool hasInternet(uint32_t timeout_ms) {
  if (WiFi.status() != WL_CONNECTED) return false;
  HTTPClient http; http.setTimeout(timeout_ms);
  if (!http.begin("http://clients3.google.com/generate_204")) return false;
  int code = http.GET(); http.end();
  return (code == 204 || code == 200);
}
static void setupWifi() {
  loadWifiApsFromNVS();
  addAPs();

  if (g_wifiApCount == 0) {
    Serial.println("No Wi-Fi APs configured -> starting SoftAP");
    WiFi.mode(WIFI_AP);
    g_softAPActive = WiFi.softAP(AUTOAP_SSID, AUTOAP_PASS);
    Serial.printf("SoftAP %s started=%d IP=%s\n", AUTOAP_SSID, (int)g_softAPActive, WiFi.softAPIP().toString().c_str());
    return;
  }

  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to Wi-Fi");
  uint32_t startMs = millis();
  for (;;) {
    while (wifiMulti.run() != WL_CONNECTED) {
      delay(250);
      Serial.print(".");
      if (millis() - startMs > 45000) break;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("\nWi-Fi OK, IP=%s\n", WiFi.localIP().toString().c_str());
      if (hasInternet()) { Serial.println("Internet OK"); break; }
      Serial.println("No internet, trying next SSID...");
      WiFi.disconnect(true, true);
      delay(500);
      startMs = millis();
      continue;
    }
    Serial.println("\nWi-Fi connect timeout -> starting SoftAP");
    WiFi.disconnect(true, true);
    WiFi.mode(WIFI_AP);
    g_softAPActive = WiFi.softAP(AUTOAP_SSID, AUTOAP_PASS);
    Serial.printf("SoftAP %s started=%d IP=%s\n", AUTOAP_SSID, (int)g_softAPActive, WiFi.softAPIP().toString().c_str());
    break;
  }
}


// ---------------- Channels / Calibration ----------------
struct Channel {
  int pin;
  char name[16];         // editable + persisted
  bool stateOn;           // latched with hysteresis
  uint16_t zero_mv;       // baseline offset (used live)
  const char* rowClass;   // row color
  float gain;             // persistent gain (NVS) - tied to channel index
  const char* nvsKeyGain; // gain key (per channel index)
  const char* nvsKeyZero; // zero key (per channel index)
};
Channel chans[] = {
  {32, "ACmains",    false, 0, "cyan-row",   1.061f, "g0", "z0"},
  {33, "ACto12v",    false, 0, "cyan-row",   1.061f, "g1", "z1"},
  {34, "Solar12v",   false, 0, "orange-row", 1.061f, "g2", "z2"},
  {35, "SolarINPUT", false, 0, "orange-row", 1.061f, "g3", "z3"},
};
const int NCHAN = sizeof(chans)/sizeof(chans[0]);



// --- Channel config helpers (ADC1 only) ---
static bool isADC1Pin(int gpio) {
  switch (gpio) {
    case 32: case 33: case 34: case 35:
    case 36: case 37: case 38: case 39:
      return true;
    default:
      return false;
  }
}

// Apply GPIO mode and ADC attenuation for current channel pins
static void applyAdcPinConfig() {
  analogReadResolution(12);
  for (int i = 0; i < NCHAN; ++i) {
    const int p = chans[i].pin;
    if (p == 32 || p == 33) pinMode(p, INPUT_PULLDOWN); // internal pulldown available
    else pinMode(p, INPUT);                              // 34-39 need external pulldown
    analogSetPinAttenuation(p, ADC_11db);
  }
}

// Persist per-channel pin + name
static void loadChannelsFromNVS() {
  Preferences p;
  p.begin("adcmon", true);
  for (int i = 0; i < NCHAN; ++i) {
    char kpin[12], kname[12];
    snprintf(kpin,  sizeof(kpin),  "ch%d_pin",  i);
    snprintf(kname, sizeof(kname), "ch%d_name", i);

    int pin = (int)p.getUInt(kpin, (uint32_t)chans[i].pin);
    String nm = p.getString(kname, String(chans[i].name));
    if (isADC1Pin(pin)) chans[i].pin = pin;

    nm.trim();
    if (nm.length() > 0) nm.toCharArray(chans[i].name, sizeof(chans[i].name));
  }
  p.end();
}

static void saveChannelsToNVS() {
  Preferences p;
  p.begin("adcmon", false);
  for (int i = 0; i < NCHAN; ++i) {
    char kpin[12], kname[12];
    snprintf(kpin,  sizeof(kpin),  "ch%d_pin",  i);
    snprintf(kname, sizeof(kname), "ch%d_name", i);
    p.putUInt(kpin, (uint32_t)chans[i].pin);
    p.putString(kname, chans[i].name);
  }
  p.end();
}

// Optional: migrate old pin-tied keys (gain32/z32...) to per-channel keys (g0/z0...)
// Runs once; harmless if already migrated.
static void migrateOldPinKeysToChannelKeysIfNeeded() {
  Preferences p;
  p.begin("adcmon", false);
  for (int i = 0; i < NCHAN; ++i) {
    const bool hasNewGain = p.isKey(chans[i].nvsKeyGain);
    const bool hasNewZero = p.isKey(chans[i].nvsKeyZero);
    if (hasNewGain && hasNewZero) continue;

    char oldG[12], oldZ[12];
    snprintf(oldG, sizeof(oldG), "gain%d", chans[i].pin);
    snprintf(oldZ, sizeof(oldZ), "z%d",    chans[i].pin);

    if (!hasNewGain && p.isKey(oldG)) {
      float g = p.getFloat(oldG, chans[i].gain);
      p.putFloat(chans[i].nvsKeyGain, g);
    }
    if (!hasNewZero && p.isKey(oldZ)) {
      uint32_t z = p.getUInt(oldZ, 0);
      p.putUInt(chans[i].nvsKeyZero, z);
    }
  }
  p.end();
}

// Find a channel by its configured name (case-insensitive). Returns index or -1.
static int findChannelIndexByName(const char* wanted) {
  for (int i = 0; i < NCHAN; ++i) {
    if (strcasecmp(chans[i].name, wanted) == 0) return i;
  }
  return -1;
}
// Thresholds (mV)
const int TH_ON_mV    = 2000;  // ON at/over 2.0 V
const int TH_OFF_mV   = 1300;  // OFF at/under 1.3 V
const int FLOOR_mV    = 80;    // clamp tiny values to 0 after baseline
const int DUMMY_READS = 8;
const int AVG_SAMPLES = 24;

// ---------------- Web / NTP / NVS ----------------
WebServer server(80);
Preferences prefs; // namespace "adcmon"

// Public IP cache
String g_publicIP = "";
uint32_t g_publicIP_ts = 0;

// Settings flags
bool g_useSavedZeros = true;  // persisted in NVS: "useZeros"


// ---------------- Cached live values (used by JSON + latest.csv/txt + alarms) ----------------
// Must be declared before sampleAllNow()
float g_ch_volt[4] = {0,0,0,0};
bool  g_ch_on[4]   = {false,false,false,false};
String g_alarmMsg = "All systems OK";
String g_alarmLevel = "ok";


// ---------------- Sampling + cache (dual-core friendly) ----------------
portMUX_TYPE g_cacheMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t g_lastSampleMs = 0;

// Tasks (ESP32 WROOM has 2 cores: keep sampling/logging off the web server loop)
static TaskHandle_t g_samplerTask = nullptr;
static TaskHandle_t g_loggerTask  = nullptr;

// Forward declarations
static void samplerTask(void* arg);
static void loggerTask(void* arg);
static void sampleAllNowInternal();
static int  idxByName(const char* wanted);
static void addAPs(); // used before definition

// Copy cached values for fast handlers (avoids partial reads)
// BME280 cached values (used in WebUI + latest.csv + MySQL)
Adafruit_BME280 bme;
bool bme_ok = false;
float bme_T = NAN, bme_RH = NAN, bme_P = NAN; // &deg;C, %, hPa
uint32_t bme_last_ms = 0;
const uint32_t BME_PERIOD_MS = 1000;

static inline void copyCache(float outVolt[4], bool outOn[4], float &t, float &rh, float &p, String &alarmMsg, String &alarmLvl) {
  portENTER_CRITICAL(&g_cacheMux);
  for (int i=0;i<4;i++){ outVolt[i]=g_ch_volt[i]; outOn[i]=g_ch_on[i]; }
  t=bme_T; rh=bme_RH; p=bme_P;
  alarmMsg = g_alarmMsg;
  alarmLvl = g_alarmLevel;
  portEXIT_CRITICAL(&g_cacheMux);
}


// ---------------- HTTP Basic Auth (Settings only) ----------------
static const char* AUTH_USER_DEFAULT = "admin";
static const char* AUTH_PASS_DEFAULT = "yo4tnv";

bool g_authEnabled = true;                     // persisted in NVS: auth_en (default true)
bool g_authBypassThisBoot = false;             // true if BOOT held at startup (temporary bypass)
String g_authUser;
String g_authPass;

static void loadAuthFromNVS() {
  Preferences p;
  p.begin("adcmon", true);
  g_authEnabled = p.getBool("auth_en", true);
  g_authUser = p.getString("auth_user", "");
  g_authPass = p.getString("auth_pass", "");
  p.end();

  if (g_authUser.length() == 0) g_authUser = AUTH_USER_DEFAULT;
  if (g_authPass.length() == 0) g_authPass = AUTH_PASS_DEFAULT;
}

static void saveAuthToNVS(bool enabled, const String& u, const String& pw) {
  Preferences p;
  p.begin("adcmon", false);
  p.putBool("auth_en", enabled);
  p.putString("auth_user", u);
  p.putString("auth_pass", pw);
  p.end();
  g_authEnabled = enabled;
  g_authUser = u;
  g_authPass = pw;
}

// Returns true if authorized; otherwise sends 401 and returns false.
static bool requireAuth() {
  if (!g_authEnabled) return true;            // auth disabled in settings
  if (g_authBypassThisBoot) return true;      // temporary bypass via BOOT button
  if (server.authenticate(g_authUser.c_str(), g_authPass.c_str())) return true;
  server.requestAuthentication();             // triggers browser Basic-Auth prompt
  return false;
}

// Like requireAuth(), but never triggers a prompt (used to optionally show the Settings link).
static bool isAuthedNoPrompt() {
  if (!g_authEnabled) return true;
  if (g_authBypassThisBoot) return true;
  return server.authenticate(g_authUser.c_str(), g_authPass.c_str());
}


// ---------------- BME280 ----------------
void bmeSampleIfDue() {
  uint32_t now = millis();
  if (!bme_ok) return;
  if (now - bme_last_ms < BME_PERIOD_MS) return;
  bme_last_ms = now;

  float t  = bme.readTemperature();  // &deg;C
  float rh = bme.readHumidity();     // %
  float pPa = bme.readPressure();    // Pa
  float ph = isnan(pPa) ? NAN : (pPa / 100.0f); // hPa

  portENTER_CRITICAL(&g_cacheMux);
  bme_T  = t;
  bme_RH = rh;
  bme_P  = ph;
  portEXIT_CRITICAL(&g_cacheMux);
}


// ---------------- Public IP ----------------
static String fetchPublicIP(uint32_t cache_ms = 60000) {
  uint32_t now = millis();
  if (!g_publicIP.length() || now - g_publicIP_ts > cache_ms) {
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http; http.setTimeout(2500);
      if (http.begin("http://api.ipify.org")) {
        int code = http.GET();
        if (code > 0) { g_publicIP = http.getString(); g_publicIP.trim(); g_publicIP_ts = now; }
        http.end();
      }
      if (!g_publicIP.length()) {
        HTTPClient http2; http2.setTimeout(2500);
        if (http2.begin("http://ifconfig.me/ip")) {
          int code = http2.GET();
          if (code > 0) { g_publicIP = http2.getString(); g_publicIP.trim(); g_publicIP_ts = now; }
          http2.end();
        }
      }
    }
  }
  if (!g_publicIP.length()) return String("n/a");
  return g_publicIP;
}

// ---------------- Helpers ----------------
static uint16_t readMilliVoltsAvg(int pin, int samples = AVG_SAMPLES, int us_delay = 200, int dummies = DUMMY_READS) {
  for (int i = 0; i < dummies; ++i) { (void)analogReadMilliVolts(pin); delayMicroseconds(100); }
  uint32_t acc = 0;
  for (int i = 0; i < samples; ++i) { acc += analogReadMilliVolts(pin); delayMicroseconds(us_delay); }
  return (uint16_t)(acc / samples);
}
static inline void updateStateFromVoltage(bool &stateOn, int mv_net) {
  if (!stateOn && mv_net >= TH_ON_mV)        stateOn = true;
  else if (stateOn && mv_net <= TH_OFF_mV)   stateOn = false;
}

// Sample BME + ADC channels and compute alarm; updates globals and channel state hysteresis
static void sampleAllNowInternal() {
  bmeSampleIfDue();
  portENTER_CRITICAL(&g_cacheMux);

  for (int i=0;i<NCHAN;i++){
    uint16_t mv = readMilliVoltsAvg(chans[i].pin);
    int net = (int)mv - (int)chans[i].zero_mv;
    if (net < 0) net = 0;
    if (net < FLOOR_mV) net = 0;
    float mv_cal = net * chans[i].gain;
    if (mv_cal < 0) mv_cal = 0;
  // NOTE: do not clamp scaled (real-world) voltage to ADC range.
updateStateFromVoltage(chans[i].stateOn, (int)(mv_cal + 0.5f));

    // cache by index order in chans[] (assumed fixed 4 channels)
    if (i < 4) {
      g_ch_volt[i] = mv_cal / 1000.0f;
      g_ch_on[i]   = chans[i].stateOn;
    }
  }

  // Alarm logic priority (by NAME, not row order):
  // 1) 220v down if ACmains OFF
  // 2) 12vPSU failure if ACmains ON and ACto12v OFF
  // 3) BATlow - System on 220v if Solar12v OFF and ACto12v ON
  // 4) SolarPanel failure if SolarINPUT OFF
  // else OK
  const int i_acm   = findChannelIndexByName("ACmains");
  const int i_act   = findChannelIndexByName("ACto12v");
  const int i_sol12 = findChannelIndexByName("Solar12v");
  const int i_solin = findChannelIndexByName("SolarINPUT");

  if (i_acm < 0)        { g_alarmMsg="Config: missing ACmains";    g_alarmLevel="warn"; }
  else if (i_act < 0)   { g_alarmMsg="Config: missing ACto12v";    g_alarmLevel="warn"; }
  else if (i_sol12 < 0) { g_alarmMsg="Config: missing Solar12v";   g_alarmLevel="warn"; }
  else if (i_solin < 0) { g_alarmMsg="Config: missing SolarINPUT"; g_alarmLevel="warn"; }
  else {
    const bool acm_on   = chans[i_acm].stateOn;
    const bool act_on   = chans[i_act].stateOn;
    const bool sol12_on = chans[i_sol12].stateOn;
    const bool solin_on = chans[i_solin].stateOn;

    if (!acm_on)                   { g_alarmMsg="220v down";              g_alarmLevel="err"; }
    else if (acm_on && !act_on)    { g_alarmMsg="12vPSU failure";         g_alarmLevel="err"; }
    else if (!sol12_on && act_on)  { g_alarmMsg="BATlow - System on 220v";g_alarmLevel="warn"; }
    else if (!solin_on)            { g_alarmMsg="SolarPanel failure";     g_alarmLevel="warn"; }
    else                           { g_alarmMsg="All systems OK";         g_alarmLevel="ok"; }
  }
  portEXIT_CRITICAL(&g_cacheMux);
}

static String utcNowString() {
  time_t now = time(nullptr);
  if (now < 1700000000) return String("syncing...");
  struct tm g; gmtime_r(&now, &g);
  char buf[32]; strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &g);
  return String(buf);
}
static String localUTCplus2() {
  time_t now = time(nullptr);
  if (now < 1700000000) return String("syncing...");
  now += 2 * 3600;
  struct tm l; gmtime_r(&now, &l);
  char buf[32]; strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &l);
  return String(buf);
}


// ---------------- Chip info helpers ----------------
static int xtalFreqMHz() {
  // Arduino-ESP32 does not reliably expose ESP-IDF clock headers on all installs.
  // Most ESP32 dev boards use a 40 MHz crystal.
  return 40;
}

static String chipFeaturesString() {
  esp_chip_info_t info;
  esp_chip_info(&info);

  String s = "";
  // Core features
  if (info.features & CHIP_FEATURE_WIFI_BGN) s += "WiFi ";
  if (info.features & CHIP_FEATURE_BT) s += "BT ";
  if (info.features & CHIP_FEATURE_BLE) s += "BLE ";
  if (info.features & CHIP_FEATURE_EMB_FLASH) s += "EmbFlash ";
  if (info.features & CHIP_FEATURE_EMB_PSRAM) s += "EmbPSRAM ";

  if (s.length() == 0) s = "n/a";
  s.trim();
  return s;
}

static String chipModelString() {
  esp_chip_info_t info;
  esp_chip_info(&info);

  // Arduino core usually targets ESP32; report generic + revision
  String s = "ESP32";
  s += " rev ";
  s += String(info.revision);
  s += ", cores ";
  s += String(info.cores);
  return s;
}

static String macStaString() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}


static const char* flashModeString() {
  // ESP.getFlashChipMode() returns FlashMode_t
  auto m = ESP.getFlashChipMode();
  switch (m) {
    case FM_QIO:  return "QIO";
    case FM_QOUT: return "QOUT";
    case FM_DIO:  return "DIO";
    case FM_DOUT: return "DOUT";
    case FM_FAST_READ: return "FAST_READ";
    case FM_SLOW_READ: return "SLOW_READ";
    default: return "n/a";
  }
}

static String psramString() {
  #if defined(ESP32)
    bool ok = psramFound();
    if (!ok) return "no";
    uint32_t sz = ESP.getPsramSize();
    return String("yes, ") + String(sz / 1024) + " KB";
  #else
    return "n/a";
  #endif
}

static String flashString() {
  uint32_t sz = ESP.getFlashChipSize();
  uint32_t sp = ESP.getFlashChipSpeed();
  String s = String(sz / (1024 * 1024)) + " MB";
  s += ", " + String(sp / 1000000) + " MHz";
  s += ", " + String(flashModeString());
  return s;
}

static String chipInfoHTML() {
  String html = "";
  html += "<table class='table info' style='margin-top:8px'>"
          "<thead><tr><th colspan='2' style='text-align:center'>ESP32</th></tr></thead><tbody>";

  html += "<tr><td>Chip</td><td>" + chipModelString() + "</td></tr>";
  html += "<tr><td>Features</td><td>" + chipFeaturesString() + "</td></tr>";
  int xf = xtalFreqMHz();
  html += "<tr><td>XTAL</td><td>" + String(xf ? xf : 0) + " MHz</td></tr>";
  html += "<tr><td>CPU</td><td>" + String(getCpuFrequencyMhz()) + " MHz</td></tr>";
  html += "<tr><td>Flash</td><td>" + flashString() + "</td></tr>";
  html += "<tr><td>PSRAM</td><td>" + psramString() + "</td></tr>";
  html += "<tr><td>MAC (STA)</td><td>" + macStaString() + "</td></tr>";
  html += "<tr><td>Auth</td><td>";
  if (!g_authEnabled) html += "disabled";
  else if (g_authBypassThisBoot) html += "BYPASS (BOOT held)";
  else html += "enabled";
  html += "</td></tr>";

  html += "</tbody></table>";
  return html;
}


// ---------------- Settings persistence ----------------
static void loadSettingsFromNVS() {
  prefs.begin("adcmon", true);
  // gains
  for (int i = 0; i < NCHAN; ++i) {
    float g = prefs.getFloat(chans[i].nvsKeyGain, chans[i].gain);
    if (g < 0.05f) g = 0.05f;
    if (g > 50.0f) g = 50.0f;
    chans[i].gain = g;
  }
  // zeros
  g_useSavedZeros = prefs.getBool("useZeros", true);
  for (int i = 0; i < NCHAN; ++i) {
    uint32_t z = prefs.getUInt(chans[i].nvsKeyZero, 0);
    if (g_useSavedZeros && z <= 1000) { chans[i].zero_mv = (uint16_t)z; }
  }
  prefs.end();
}
static void saveGainsToNVS(const float newGains[]) {
  prefs.begin("adcmon", false);
  for (int i = 0; i < NCHAN; ++i) { prefs.putFloat(chans[i].nvsKeyGain, newGains[i]); chans[i].gain = newGains[i]; }
  prefs.end();
}
static void saveZerosToNVS() {
  prefs.begin("adcmon", false);
  for (int i = 0; i < NCHAN; ++i) { prefs.putUInt(chans[i].nvsKeyZero, chans[i].zero_mv); }
  prefs.end();
}
static void clearZerosInNVS() {
  prefs.begin("adcmon", false);
  for (int i = 0; i < NCHAN; ++i) { prefs.remove(chans[i].nvsKeyZero); }
  prefs.end();
}
static void setUseZeros(bool en) {
  prefs.begin("adcmon", false);
  prefs.putBool("useZeros", en);
  prefs.end();
  g_useSavedZeros = en;
}



// ---------------- MySQL logging (BME280 -> remote MySQL) ----------------
// Requires Arduino library: "MySQL_Connector_Arduino"
struct MySQLCfg {
  bool enabled;
  uint16_t interval_min; // log period (minutes)
  char host[64];     // IP or hostname
  uint16_t port;     // usually 3306
  char user[32];
  char pass[32];
  char db[32];
  char table[32];
};

MySQLCfg mysqlCfg = {
  false,
  1,
  "192.168.1.100",
  3306,
  "logger",
  "loggerpass",
  "sensors",
  "bme280_log"
};

WiFiClient mysqlClient;
MySQL_Connection mysqlConn(&mysqlClient);

uint32_t g_lastMySQLLogMs = 0;
// Effective MySQL logging period (ms). Derived from settings interval_min.
uint32_t g_mysqlLogPeriodMs = 60UL * 1000UL;
static bool g_mysqlDbEnsured = false;    // reset on reconnect/config change
static bool g_mysqlTableEnsured = false; // reset on reconnect/config change

// ---------------- Latest snapshot + latest.csv ----------------
struct LatestCfg {
  bool enabled;
  uint16_t interval_min; // write period (minutes)
};
LatestCfg latestCfg = { true, 1 }; // default: enabled, every 1 minute
uint32_t g_lastLatestWriteMs = 0;
bool g_fs_ok = false; // LittleFS mounted OK

static void loadLatestFromNVS() {
  Preferences p;
  p.begin("adcmon", true);
  latestCfg.enabled = p.getBool("lat_en", latestCfg.enabled);
  latestCfg.interval_min = (uint16_t)p.getUInt("lat_iv", latestCfg.interval_min);
  p.end();
  if (latestCfg.interval_min < 1) latestCfg.interval_min = 1;
  if (latestCfg.interval_min > 1440) latestCfg.interval_min = 1440;
}
static void saveLatestToNVS() {
  Preferences p;
  p.begin("adcmon", false);
  p.putBool("lat_en", latestCfg.enabled);
  p.putUInt("lat_iv", latestCfg.interval_min);
  p.end();
}

// cached latest values are declared near the top (before sampleAllNow())

static void loadMySQLFromNVS() {
  Preferences p;
  p.begin("adcmon", true);
  mysqlCfg.enabled = p.getBool("mq_en", mysqlCfg.enabled);
  mysqlCfg.interval_min = (uint16_t)p.getUInt("mq_iv", mysqlCfg.interval_min);
  if (mysqlCfg.interval_min < 1) mysqlCfg.interval_min = 1;
  if (mysqlCfg.interval_min > 1440) mysqlCfg.interval_min = 1440;

  String s;
  s = p.getString("mq_host", mysqlCfg.host);  s.toCharArray(mysqlCfg.host, sizeof(mysqlCfg.host));
  mysqlCfg.port = (uint16_t)p.getUInt("mq_port", mysqlCfg.port);
  s = p.getString("mq_user", mysqlCfg.user);  s.toCharArray(mysqlCfg.user, sizeof(mysqlCfg.user));
  s = p.getString("mq_pass", mysqlCfg.pass);  s.toCharArray(mysqlCfg.pass, sizeof(mysqlCfg.pass));
  s = p.getString("mq_db",   mysqlCfg.db);    s.toCharArray(mysqlCfg.db,   sizeof(mysqlCfg.db));
  s = p.getString("mq_tab",  mysqlCfg.table); s.toCharArray(mysqlCfg.table,sizeof(mysqlCfg.table));
  p.end();

  if (mysqlCfg.port == 0) mysqlCfg.port = 3306;
  g_mysqlLogPeriodMs = (uint32_t)mysqlCfg.interval_min * 60UL * 1000UL;
}

static void saveMySQLToNVS() {
  Preferences p;
  p.begin("adcmon", false);
  p.putBool("mq_en", mysqlCfg.enabled);
  p.putUInt("mq_iv", mysqlCfg.interval_min);
  p.putString("mq_host", mysqlCfg.host);
  p.putUInt("mq_port", mysqlCfg.port);
  p.putString("mq_user", mysqlCfg.user);
  p.putString("mq_pass", mysqlCfg.pass);
  p.putString("mq_db", mysqlCfg.db);
  p.putString("mq_tab", mysqlCfg.table);
  p.end();
}

static void mysqlDisconnect() {
  if (mysqlConn.connected()) mysqlConn.close();
  g_mysqlDbEnsured = false;
  g_mysqlTableEnsured = false;
}

static bool mysqlEnsureConnected() {
  if (!mysqlCfg.enabled) return false;
  if (WiFi.status() != WL_CONNECTED) return false;
  if (mysqlConn.connected()) return true;

  Serial.printf("[MySQL] connecting %s:%u ...\n", mysqlCfg.host, mysqlCfg.port);

  IPAddress ip;
  bool isIP = ip.fromString(mysqlCfg.host);
  bool ok = false;

  if (isIP) ok = mysqlConn.connect(ip, mysqlCfg.port, mysqlCfg.user, mysqlCfg.pass);
  else      ok = mysqlConn.connect(mysqlCfg.host, mysqlCfg.port, mysqlCfg.user, mysqlCfg.pass);

  if (!ok) {
    Serial.println("[MySQL] connect failed");
    return false;
  }

  Serial.println("[MySQL] connected");
  g_mysqlDbEnsured = false;
  g_mysqlTableEnsured = false;
  return true;
}


// Create the database if it does not exist.
// Uses CREATE DATABASE IF NOT EXISTS, safe to call repeatedly.
// Requires MySQL user to have CREATE privilege.
static bool mysqlEnsureDatabase() {
  if (g_mysqlDbEnsured) return true;
  if (!mysqlEnsureConnected()) return false;

  char q[256];
  // utf8mb4 is a good default for general text.
  snprintf(q, sizeof(q),
           "CREATE DATABASE IF NOT EXISTS `%s` CHARACTER SET utf8mb4 COLLATE utf8mb4_unicode_ci",
           mysqlCfg.db);

  MySQL_Cursor cur(&mysqlConn);
  bool ok = cur.execute(q);
  if (!ok) {
    Serial.println("[MySQL] database ensure FAILED (no privilege?)");
    mysqlDisconnect();
    return false;
  }
  Serial.println("[MySQL] database ensure OK (exists/created)");
  g_mysqlDbEnsured = true;
  return true;
}

// Create the logging table if it does not exist.
// Uses CREATE TABLE IF NOT EXISTS, so it's safe to call repeatedly.
static bool mysqlEnsureTable() {
  if (g_mysqlTableEnsured) return true;
  if (!mysqlEnsureDatabase()) return false;

  // Create DB.table if missing.
  char q[512];
  snprintf(q, sizeof(q),
           "CREATE TABLE IF NOT EXISTS `%s`.`%s` ("
           "id INT AUTO_INCREMENT PRIMARY KEY,"
           "date VARCHAR(10) NOT NULL,"
           "time VARCHAR(8) NOT NULL,"
           "temp FLOAT NOT NULL,"
           "pressure FLOAT NOT NULL,"
           "humidity FLOAT NOT NULL"
           ")",
           mysqlCfg.db, mysqlCfg.table);

  MySQL_Cursor cur(&mysqlConn);
  bool ok = cur.execute(q);
  if (!ok) {
    Serial.println("[MySQL] table ensure FAILED");
    mysqlDisconnect();
    return false;
  }
  Serial.println("[MySQL] table ensure OK (exists/created)");
  g_mysqlTableEnsured = true;
  return true;
}

static bool bmeReadNow(float &tC, float &rh, float &p_hPa) {
  if (!bme_ok) return false;
  tC = bme.readTemperature();
  rh = bme.readHumidity();
  float pPa = bme.readPressure();
  if (isnan(tC) || isnan(rh) || isnan(pPa)) return false;
  p_hPa = pPa / 100.0f;
  return true;
}

static bool getUTCDateTime(char *outDate, size_t outDateN, char *outTime, size_t outTimeN) {
  time_t now = time(nullptr);
  if (now < 1700000000) return false; // NTP not synced yet
  struct tm g; gmtime_r(&now, &g);
  // dd-mm-yyyy
  snprintf(outDate, outDateN, "%02d-%02d-%04d", g.tm_mday, g.tm_mon + 1, g.tm_year + 1900);
  // hh:mm:ss
  snprintf(outTime, outTimeN, "%02d:%02d:%02d", g.tm_hour, g.tm_min, g.tm_sec);
  return true;
}

static void logBME280ToMySQLIfDue() {
  if (!mysqlCfg.enabled) return;
  if (millis() - g_lastMySQLLogMs < g_mysqlLogPeriodMs) return;

  char d[11], t[9];
  if (!getUTCDateTime(d, sizeof(d), t, sizeof(t))) return; // wait for NTP time

  float tc, rh, ph;
  if (!bmeReadNow(tc, rh, ph)) return;

  if (!mysqlEnsureTable()) return;

  // Build INSERT query
  char query[320];
  // Use explicit db.table so we don't need "USE db"
  snprintf(query, sizeof(query),
           "INSERT INTO `%s`.`%s` (date,time,temp,pressure,humidity) "
           "VALUES ('%s','%s',%.2f,%.2f,%.2f)",
           mysqlCfg.db, mysqlCfg.table,
           d, t, tc, ph, rh);

  MySQL_Cursor cur(&mysqlConn);
  bool ok = cur.execute(query);

  if (ok) {
    g_lastMySQLLogMs = millis();
    Serial.println("[MySQL] insert OK");
  } else {
    Serial.println("[MySQL] insert FAILED; will retry/reconnect");
    mysqlDisconnect();
  }
}

// ---------------- HTML helpers ----------------
static String htmlHeaderStart(bool autoreload=false) {
  String hdr = R"HTML(<!doctype html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>ESP32 ADC1 Monitor</title>
<style>
  :root{--fg:#0f0;--grid:#0f0}
  body{
    background:#000;color:var(--fg);
    font-family:system-ui,Segoe UI,Arial;margin:10px;
    font-size:15px;line-height:1.4;
  }
  .wrap{max-width:420px;margin:0 auto}
  .title{margin:8px 0 10px;text-align:center;font-size:22px;font-weight:800;color:#00ff66;text-shadow:0 0 6px #00ff66,0 0 14px #00ff66,0 0 28px #00ff66;letter-spacing:0.5px}
  .table{width:100%;border-collapse:collapse}
  .info{margin:0 auto 8px auto;max-width:420px;font-size:14px}
  .info th,.info td{border:1px solid var(--grid);padding:4px 7px;text-align:left}
  .info th{background:#001600;font-weight:600}
  h1{margin:6px 0 6px;font-size:18px;font-weight:600;text-align:left}
  .main{width:100%;border-collapse:collapse;margin-top:4px;font-size:14px}
  .main th,.main td{border:1px solid var(--grid);padding:4px 7px}
  .main th{background:#001600;font-weight:600}
  .on{font-weight:700}
  .off{font-weight:700;color:#f66}
  .cyan-row{color:#00e5ff}
  .orange-row{color:#ffa500}
  /* Temperature color classes */
  .tcyan{color:#00ffff;font-weight:700}
  .tblue{color:#00aaff;font-weight:700}
  .tgreen{color:#00ff00;font-weight:700}
  .torange{color:#ffa500;font-weight:700}
  .foot{opacity:.75;font-size:12px;margin-top:6px}
  .nav{display:flex;gap:8px;justify-content:flex-end;font-size:13px;margin-top:4px}
  .nav a{color:#0ff;text-decoration:none}
  .nav a:hover{text-decoration:underline}
  .lock{opacity:.85;color:#f66;margin-left:6px}
  .frm{border:2px solid var(--grid);padding:10px;border-radius:12px;margin:10px 0;background:#000}
  .frm h2{margin:0 0 6px;font-size:16px;font-weight:600;border-bottom:1px dashed var(--grid);padding-bottom:4px}
  .frm h3{margin:10px 0 6px;font-size:14px;font-weight:600}
  .helpul{margin:6px 0 0 18px;padding:0}
  .helpul li{margin:3px 0}
  .frm label{display:block;margin:6px 0 2px}
  .frm input[type=number], .frm input[type=text], .frm input[type=password]{width:100%;padding:7px;background:#000;border:1px solid var(--grid);color:var(--fg);border-radius:6px;font-size:14px}
  .row{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  .btns{display:flex;gap:8px;margin-top:10px;flex-wrap:wrap}
  button, .btn{background:#001a00;border:1px solid #0f0;color:#0f0;border-radius:8px;padding:8px 10px;cursor:pointer;font-size:14px}
  .chk{display:flex;align-items:center;gap:8px;margin-top:8px;font-size:14px}
  .warn{opacity:.8;font-size:12px;margin-top:6px}
  @media (max-width:480px){
    .wrap{max-width:360px}
    .info,.main{font-size:13px}
    .info th,.info td,.main th,.main td{padding:4px 5px}
    .row{grid-template-columns:1fr}
  }
</style>
)HTML";
  // No meta refresh; AJAX will update values.
  hdr += "</head><body><div class=\"wrap\">\n";
  return hdr;
}
static String htmlFooterEndWithJS() {
  // Client-side updater
  return R"HTML(
<script>
async function updateData(){
  try{
    const r = await fetch('/data.json', {cache:'no-store'});
    const d = await r.json(); if(!d) return;
    // times
    const utc = document.getElementById('utc');
    const local = document.getElementById('local');
    if(utc) utc.textContent = d.utc || 'syncing...';
    if(local) local.textContent = d.local || 'syncing...';
    // bme
    const tempCell = document.getElementById('temp');
    const humCell  = document.getElementById('hum');
    const prsCell  = document.getElementById('press');
    if(tempCell){
      if(d.temp===null || d.temp===undefined){
        tempCell.textContent='n/a';
      }else{
        const cls = d.temp<0 ? 'tcyan' : d.temp<10 ? 'tblue' : d.temp<25 ? 'tgreen' : 'torange';
        tempCell.innerHTML = `<span class="${cls}">${d.temp.toFixed(1)} &deg;C</span>`;
      }
    }
    if(humCell)  humCell.textContent  = (d.hum==null)  ? 'n/a' : d.hum.toFixed(1)+' %';
    if(prsCell)  prsCell.textContent  = (d.press==null) ? 'n/a' : d.press.toFixed(1)+' hPa';
    // channels
    (d.channels||[]).forEach(c=>{
      const row = document.getElementById('r'+c.pin);
      if(!row) return;
      const v = row.querySelector('.volt');
      const s = row.querySelector('.state');
      if(v) v.textContent = c.volt.toFixed(3)+' V';
      if(s){
        s.textContent = c.state;
        s.className = 'state ' + (c.state==='ON'?'on':'off');
      }
    });
  }catch(e){}
}
setInterval(updateData, 1000);
updateData();
</script>
</div></body></html>
)HTML";
}

// ---------------- HTTP: JSON data endpoint ----------------

void handleDataJSON() {
  float v[4]; bool on[4]; float t,rh,p; String amsg, alvl;
  copyCache(v,on,t,rh,p,amsg,alvl);

  String json = "{";
  json += "\"utc\":\"" + utcNowString() + "\",";
  json += "\"local\":\"" + localUTCplus2() + "\",";
  json += "\"temp\":";  json += isnan(t)  ? "null" : String(t,1); json += ",";
  json += "\"hum\":";   json += isnan(rh) ? "null" : String(rh,1); json += ",";
  json += "\"press\":"; json += isnan(p)  ? "null" : String(p,1); json += ",";

  json += "\"channels\":[";
  for (int i=0;i<NCHAN;i++){
    if (i) json += ",";
    json += "{\"pin\":" + String(chans[i].pin);
    json += ",\"name\":\"" + String(chans[i].name) + "\"";
    float vv = (i < 4) ? v[i] : 0.0f;
    json += ",\"volt\":" + String(vv,3);
    json += ",\"state\":\"" + String((i<4 && on[i]) ? "ON":"OFF") + "\"}";
  }
  json += "],";
  json += "\"alarm\":\"" + amsg + "\",";
  json += "\"level\":\"" + alvl + "\"";
  json += "}";

  server.send(200, "application/json", json);
}



// ---------------- HTTP: Main page ----------------
void handleRoot() {
  // Use cached snapshot (sampling runs in background task)
  float v[4]; bool on[4]; float t,rh,p; String amsg, alvl;
  copyCache(v,on,t,rh,p,amsg,alvl);

  // Prepare initial table values (mV) for first render; JS will refresh after load
  uint16_t mv_net[NCHAN];
  for (int i = 0; i < NCHAN; ++i) {
    float vv = (i < 4) ? v[i] : 0.0f;
    int mv = (int)(vv * 1000.0f + 0.5f);
    if (mv < 0) mv = 0;
    if (mv > 3300) mv = 3300;
    mv_net[i] = (uint16_t)mv;
  }

String html = htmlHeaderStart(false);
  html += R"HTML(
  <div class="nav">__SETTINGS_LINK__</div>
  <div class="title">YO4TNV Voltage watchdog</div>
  <table class="table info">
    <thead><tr><th colspan="2" style="text-align:center">System</th></tr></thead>
    <tbody>
      <tr><td>UTC</td><td id="utc">)HTML";
  // Show Settings link only if already authenticated (no prompt)
  if (isAuthedNoPrompt()) {
    html.replace("__SETTINGS_LINK__", "<a href=\"/help\">Help</a><a href=\"/settings\">Settings</a>");
  } else {
    html.replace("__SETTINGS_LINK__", "<a href=\"/help\">Help</a><a href=\"/settings\">Settings</a><span class=\"lock\">&#128274; locked</span>");
  }

  html += utcNowString();
  html += R"HTML(</td></tr>
      <tr><td>Local (UTC+2)</td><td id="local">)HTML";
  html += localUTCplus2();
  html += R"HTML(</td></tr>
      <tr><td>Local IP</td><td id="lip">)HTML";
  html += WiFi.localIP().toString();
  html += R"HTML(</td></tr>
      <tr><td>Public IP</td><td id="pip">)HTML";
  html += (g_publicIP.length() ? g_publicIP : String("n/a"));
  html += R"HTML(</td></tr>
      <tr><td>Temperature</td><td id="temp">)HTML";
  if (bme_ok && !isnan(t)) {
    const char* tclass = (t < 0) ? "tcyan" : (t < 10) ? "tblue" : (t < 25) ? "tgreen" : "torange";
    char tbuf[48]; snprintf(tbuf, sizeof(tbuf), "<span class='%s'>%.1f &deg;C</span>", tclass, t);
    html += tbuf;
  } else html += "n/a";
  html += R"HTML(</td></tr>
      <tr><td>Humidity</td><td id="hum">)HTML";
  if (bme_ok && !isnan(rh)) { char hbuf[16]; snprintf(hbuf, sizeof(hbuf), "%.1f %%", rh); html += hbuf; }
  else html += "n/a";
  html += R"HTML(</td></tr>
      <tr><td>Pressure</td><td id="press">)HTML";
  if (bme_ok && !isnan(p)) { char pbuf[20]; snprintf(pbuf, sizeof(pbuf), "%.1f hPa", p); html += pbuf; }
  else html += "n/a";
  html += R"HTML(</td></tr>
    </tbody>
  </table>

  <h1>ADC1 Pins</h1>
  <table class="table main">
    <thead><tr><th>Pin</th><th>Name</th><th>Volt</th><th>Status</th></tr></thead>
    <tbody>
)HTML";

  // rows with IDs for AJAX updates
  for (int i = 0; i < NCHAN; ++i) {
    char vbuf[16]; snprintf(vbuf, sizeof(vbuf), "%.3f V", mv_net[i] / 1000.0f);
    html += String("<tr id='r") + String(chans[i].pin) + "' class='" + chans[i].rowClass + "'>";
    html += "<td>GPIO " + String(chans[i].pin) + "</td>";
    html += "<td>" + String(chans[i].name) + "</td>";
    html += "<td class='volt'>" + String(vbuf) + "</td>";
    html += String("<td class='state ") + ((i<4 && on[i]) ? "on'>ON" : "off'>OFF") + "</td>";
    html += "</tr>\n";
  }

    html += R"HTML(
    </tbody>
  </table>
)HTML";

    // Device information frame
  html += chipInfoHTML();

html += "<div class='foot'>Hyst: ON >= 2.0 V, OFF <= 1.3 V. GPIO32/33 have internal pulldown; GPIO34/35 need ~100 kohm to GND. Never exceed 3.3 V.<br>Sketch: "
       + String(SKETCH_FILE) + " | Build: " + String(SKETCH_BUILD) + "</div>\n";

  html += htmlFooterEndWithJS();
  server.send(200, "text/html", html);
}

// ---------------- Settings page & actions ----------------
void handleSettingsPage() {
  if (!requireAuth()) return;

  String html = htmlHeaderStart(false);

  html += "<div class='nav'><a href='/'>\xE2\x86\x90 Back</a></div>";
  html += "<h1>Settings</h1>";

  // --- Gains / zeros ---
  html += "<form class='frm' method='POST' action='/save'>";
  html += "<h2>Calibration</h2>";
  html += "<div class='hint'>Gain multiplies the measured ESP32 pin voltage to the real voltage (after divider). Large dividers may need gains > 1.25 (allowed up to 50).</div>";
  html += "<div class='row'>";
  for (int i = 0; i < NCHAN; ++i) {
    html += "<div>";
    html += "<label>CH" + String(i) + " gain (GPIO " + String(chans[i].pin) + " - " + String(chans[i].name) + ")</label>";
    html += "<input name='g" + String(i) + "' type='number' step='0.001' min='0.05' max='50' value='" + String(chans[i].gain, 3) + "'>";
    html += "</div>";
  }
  html += "</div>";
  html += "<div class='chk'>";
  html += "<input id='useZeros' name='useZeros' type='checkbox'";
  if (g_useSavedZeros) html += " checked";
  html += ">";
  html += "<label for='useZeros'>Use saved zero offsets at boot</label>";
  html += "</div>";
  html += "<div class='btns'>";
  html += "<button type='submit'>Save gains / toggle</button>";
  html += "<a class='btn' href='/recal'>Recalibrate zeros (measure now)</a>";
  html += "<a class='btn' href='/saveZeros'>Save current zeros</a>";
  html += "<a class='btn' href='/clearZeros'>Clear saved zeros</a>";
  html += "</div>";
  html += "</form>";

  // --- Channels (pin + name) ---
  html += "<form class='frm' method='POST' action='/chsave'>";
  html += "<h2>Channels</h2>";
  html += "<div class='row'>";
  for (int i = 0; i < NCHAN; ++i) {
    html += "<div>";
    html += "<label>CH" + String(i) + " GPIO (ADC1 only: 32-39)</label>";
    html += "<input name='cp" + String(i) + "' type='number' min='32' max='39' value='" + String(chans[i].pin) + "'>";
    html += "<label>CH" + String(i) + " Name</label>";
    html += "<input name='cn" + String(i) + "' type='text' maxlength='15' value='" + String(chans[i].name) + "'>";
    html += "</div>";
  }
  html += "</div>";
  html += "<div class='btns'><button type='submit'>Save channels</button></div>";
  html += "<div class='warn'>Use ADC1 only (32-39). GPIO34-39 need external pulldown (~100 k\xCE\xA9) because they have no internal pulls.</div>";
  html += "</form>";

  
  // --- Wi-Fi APs ---
  html += "<form class='frm' method='POST' action='/wifisave'>";
  html += "<h2>Wi-Fi access points</h2>";
  html += "<div class='row'>";
  for (int i = 0; i < MAX_WIFI_APS; ++i) {
    String ss = (i < g_wifiApCount) ? String(g_wifiAps[i].ssid) : String("");
    String pw = (i < g_wifiApCount) ? String(g_wifiAps[i].pass) : String("");
    html += "<div>";
    html += "<label>AP " + String(i+1) + " SSID</label>";
    html += "<input name='ws" + String(i) + "' type='text' maxlength='32' value='" + ss + "'>";
    html += "<label>AP " + String(i+1) + " Password</label>";
    html += "<input name='wp" + String(i) + "' type='password' maxlength='64' value='" + pw + "'>";
    html += "</div>";
  }
  html += "</div>";
  html += "<div class='btns'><button type='submit'>Save Wi-Fi list (reconnect)</button></div>";
  html += "<div class='warn'>Tip: leave SSID empty to disable a slot. After saving, ESP32 will disconnect and reconnect using the new list (no reboot).</div>";
  html += "</form>";

// --- MySQL Logger ---
  html += "<form class='frm' method='POST' action='/mysqlsave'>";
  html += "<h2>MySQL Logger</h2>";
  html += "<div class='chk'><input id='mqen' name='mqen' type='checkbox'";
  if (mysqlCfg.enabled) html += " checked";
  html += "><label for='mqen'>Enable MySQL logging</label></div>";

  html += "<label>Log interval (minutes)</label>";
  html += "<input name='mqint' type='number' min='1' max='1440' value='" + String(mysqlCfg.interval_min) + "'>";

  html += "<label>Host / IP</label><input name='mqhost' type='text' value='" + String(mysqlCfg.host) + "'>";
  html += "<label>Port</label><input name='mqport' type='number' min='1' max='65535' value='" + String(mysqlCfg.port) + "'>";
  html += "<label>User</label><input name='mquser' type='text' value='" + String(mysqlCfg.user) + "'>";
  html += "<label>Password</label><input name='mqpass' type='password' value='" + String(mysqlCfg.pass) + "'>";
  html += "<label>Database</label><input name='mqdb' type='text' value='" + String(mysqlCfg.db) + "'>";
  html += "<label>Table</label><input name='mqtab' type='text' value='" + String(mysqlCfg.table) + "'>";

  html += "<div class='btns'>";
  html += "<button type='submit'>Save MySQL settings</button>";
  html += "<a class='btn' href='/mysqltest'>Test insert now</a>";
  html += "<a class='btn' href='/mysqldisc'>Disconnect</a>";
  html += "</div>";
  html += "<div class='warn'>Plain TCP MySQL. Use LAN/VPN, not public Internet.</div>";
  html += "</form>";

  // --- Latest files ---
  html += "<form class='frm' method='POST' action='/ltsave'>";
  html += "<h2>Latest files</h2>";
  html += "<div class='chk'><input id='lten' name='lten' type='checkbox'";
  if (latestCfg.enabled) html += " checked";
  html += "><label for='lten'>Enable /latest.csv</label></div>";

  html += "<label>Write interval (minutes)</label>";
  html += "<input name='ltint' type='number' min='1' max='1440' value='" + String(latestCfg.interval_min) + "'>";
  html += "<div class='btns'><button type='submit'>Save latest settings</button></div>";
  html += "<div class='warn'>Read: http://esp32.local/latest.csv (mDNS) or via IP.</div>";
  html += "</form>";

  // --- WebUI Auth ---
  html += "<form class='frm' method='POST' action='/authsave'>";
  html += "<h2>WebUI Login</h2>";
  html += "<div class='chk'><input id='auth_en' name='auth_en' type='checkbox'";
  if (g_authEnabled) html += " checked";
  html += "><label for='auth_en'>Enable HTTP Basic Auth for Settings</label></div>";
  html += "<label>Username</label><input name='au' type='text' value='" + g_authUser + "'>";
  html += "<label>New password</label><input name='ap' type='password' value=''>";
  html += "<div class='btns'><button type='submit'>Save login</button></div>";
  html += "<div class='warn'>Leave password empty to keep current. Hold BOOT during reset to bypass auth for one boot.</div>";
  html += "</form>";

  html += "<div class='foot'>Build: ";
  html += String(__FILE__) + " | " + String(__DATE__) + " " + String(__TIME__);
  html += "</div>";

  html += htmlFooterEndWithJS();
  server.send(200, "text/html", html);
}

// ---------------- Help page ----------------
void handleHelpPage() {
  String html = htmlHeaderStart(false);
  html += "<div class='nav'><a href='/'>\xE2\x86\x90 Back</a><a href='/settings'>Settings</a></div>";
  html += "<h1>Help</h1>";

  html += "<div class='frm'><h2>Dashboard</h2>";
  html += "<ul class='helpul'>"
          "<li>Main page shows a compact System table (UTC/local time, IP info) and the voltage table for 4 ADC channels.</li>"
          "<li>Values refresh automatically (AJAX) without reloading the page.</li>"
          "<li>An alarm textbox summarizes the current state with priority rules.</li>"
          "</ul></div>";

  html += "<div class='frm'><h2>BME280 sensor</h2>";
  html += "<ul class='helpul'>"
          "<li>Connected via I\xc2\xb2C: SDA=GPIO21, SCL=GPIO22 (default in this sketch).</li>"
          "<li>Temperature is color-coded: &lt;0\xc2\xb0C cyan, &lt;10\xc2\xb0C blue, &lt;25\xc2\xb0C green, \xe2\x89\xa525\xc2\xb0C orange.</li>"
          "<li>Humidity (%%) and pressure (hPa) are displayed in the System table and exported to latest.csv.</li>"
          "</ul></div>";

  html += "<div class='frm'><h2>Calibration</h2>";
  html += "<ul class='helpul'>"
          "<li>Each channel has a gain factor to correct ADC scaling.</li>"
          "<li>Zero offsets can be measured (Recalibrate) and optionally applied at boot (Use saved zero offsets).</li>"
          "<li>Gains and zeros are stored persistently in NVS and tied to the channel index (CH0..CH3).</li>"
          "</ul></div>";

  html += "<div class='frm'><h2>Channels</h2>";
  html += "<ul class='helpul'>"
          "<li>Each channel can be configured with an ADC1 GPIO and a name label.</li>"
          "<li>Recommended ADC1 GPIOs: 32\xe2\x80\x9339. Note: GPIO34\xe2\x80\x9339 are input-only and need an external pulldown resistor.</li>"
          "<li>Alarm logic follows the channel names: ACmains, ACto12v, Solar12v, SolarINPUT.</li>"
          "</ul></div>";

  html += "<div class='frm'><h2>Wi-Fi access points</h2>";
  html += "<ul class='helpul'>"
          "<li>Multiple Wi-Fi APs are stored and tried in order using WiFiMulti.</li>"
          "<li>After connecting to an SSID, the device checks Internet reachability; if it fails, it tries the next AP.</li>"
          "<li>The AP list is editable in Settings and saved persistently (no reboot required; reconnect happens automatically).</li>"
          "<li>mDNS is enabled so you can use http://esp32.local/ (if your network supports it).</li>"
          "</ul></div>";

  html += "<div class='frm'><h2>MySQL logger</h2>";
  html += "<ul class='helpul'>"
          "<li>Configurable remote MySQL logging (host, port, user, password, database, table).</li>"
          "<li>Logging interval is configurable in minutes.</li>"
          "<li>Database and table can be auto-created if missing (requires CREATE privileges).</li>"
          "<li>Records include date (dd-mm-yyyy), time (hh:mm:ss UTC), temperature, pressure, humidity.</li>"
          "</ul></div>";

  html += "<div class='frm'><h2>Latest files</h2>";
  html += "<ul class='helpul'>"
          "<li>Every N minutes (configurable), the device writes latest.csv to LittleFS.</li>"
          "<li>Third-party apps can fetch: http://esp32.local/latest.csv</li>"
          "<li>CSV contains UTC/local time, BME280 values, channel voltages/states and current alarm text.</li>"
          "</ul></div>";

  html += "<div class='frm'><h2>WebUI login</h2>";
  html += "<ul class='helpul'>"
          "<li>Settings and write-actions are protected with HTTP Basic Auth (browser login prompt).</li>"
          "<li>When not logged in, the nav shows \xf0\x9f\x94\x92 locked next to Settings.</li>"
          "<li>Fail-safe: hold BOOT (GPIO0) during reset/power-up to bypass auth for that boot only.</li>"
          "</ul></div>";

  html += "<div class='frm'><h2>Build and chip info</h2>";
  html += "<ul class='helpul'>"
          "<li>The footer shows sketch filename and build timestamp.</li>"
          "<li>A chip info frame shows SoC model/revision/cores, features, CPU frequency, and MAC address.</li>"
          "</ul></div>";

  html += "<div class='frm'><h2>Auto SoftAP (no Wi-Fi configured)</h2>";
  html += "<p>If no Wi-Fi SSIDs are configured in Settings, the ESP32 starts a SoftAP so you can connect and configure Wi-Fi: <b>EspStation</b> / <b>123456</b>.</p>";
  html += "<p>Open <code>http://192.168.4.1/</code> after connecting to the AP.</p>";
  html += "</div>";
  html += htmlFooterEndWithJS(); // no extra JS needed, keep consistent footer
  server.send(200, "text/html", html);
}

void handleSave() {
  if (!requireAuth()) return;

  auto clamp = [](float v){ if(v<0.05f) v=0.05f; if(v>50.0f) v=50.0f; return v; };
  float arr[4];
  for (int i = 0; i < NCHAN; ++i) {
    String key = String("g") + String(i);
    float gv = server.hasArg(key) ? server.arg(key).toFloat() : chans[i].gain;
    arr[i] = clamp(gv);
  }
  saveGainsToNVS(arr);

  bool wantUseZeros = server.hasArg("useZeros");
  setUseZeros(wantUseZeros);

  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleChSave() {
  if (!requireAuth()) return;

  bool changedPins = false;
  bool changedNames = false;

  for (int i = 0; i < NCHAN; ++i) {
    String ap = "cp" + String(i);
    String an = "cn" + String(i);

    if (server.hasArg(ap)) {
      int newPin = server.arg(ap).toInt();
      if (isADC1Pin(newPin) && newPin != chans[i].pin) {
        chans[i].pin = newPin;
        changedPins = true;
      }
    }

    if (server.hasArg(an)) {
      String nm = server.arg(an);
      nm.trim();
      if (nm.length() > 0) {
        char tmp[16];
        nm.toCharArray(tmp, sizeof(tmp));
        if (strncmp(tmp, chans[i].name, sizeof(chans[i].name)) != 0) {
          strncpy(chans[i].name, tmp, sizeof(chans[i].name));
          chans[i].name[sizeof(chans[i].name)-1] = 0;
          changedNames = true;
        }
      }
    }
  }

  if (changedPins || changedNames) {
    saveChannelsToNVS();
  }

  if (changedPins) {
    // Pins changed => zeros are invalid. Reset and re-apply pin config.
    for (int i = 0; i < NCHAN; ++i) { chans[i].zero_mv = 0; chans[i].stateOn = false; }
    applyAdcPinConfig();
    // Optional: quick baseline capture
    for (int i = 0; i < NCHAN; ++i) {
      uint16_t z = readMilliVoltsAvg(chans[i].pin, 32);
      chans[i].zero_mv = (z < 400) ? z : 0;
    }
    if (g_useSavedZeros) saveZerosToNVS();
  }

  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleWiFiSave() {
  if (!requireAuth()) return;

  // Build list from form fields
  int count = 0;
  for (int i = 0; i < MAX_WIFI_APS; ++i) {
    String ks = "ws" + String(i);
    String kp = "wp" + String(i);
    String ss = server.hasArg(ks) ? server.arg(ks) : "";
    String pw = server.hasArg(kp) ? server.arg(kp) : "";
    ss.trim();
    if (ss.length() == 0) continue;

    ss.toCharArray(g_wifiAps[count].ssid, sizeof(g_wifiAps[count].ssid));
    pw.toCharArray(g_wifiAps[count].pass, sizeof(g_wifiAps[count].pass));
    count++;
    if (count >= MAX_WIFI_APS) break;
  }
  g_wifiApCount = count;

  // If user cleared all, revert to defaults
  if (g_wifiApCount == 0) {
    int dn = (int)(sizeof(DEFAULT_APS)/sizeof(DEFAULT_APS[0]));
    if (dn > MAX_WIFI_APS) dn = MAX_WIFI_APS;
    for (int i = 0; i < dn; ++i) g_wifiAps[i] = DEFAULT_APS[i];
    g_wifiApCount = dn;
  }

  saveWifiApsToNVS();

  // Start reconnect without reboot (state machine runs in loop)
  startWifiReconnect();

  // Respond (page will show live Wi-Fi status)
  String html = htmlHeaderStart(false);
  html += "<div class='nav'><a href='/settings'>&larr; Back</a></div>";
  html += "<h1>Wi-Fi saved</h1>";
  html += "<div class='warn'>Saved Wi-Fi AP list. Reconnecting now (no reboot)...</div>";
  html += "<div class='alarm ok' style='margin-top:10px' id='wstat'>checking...</div>";
  html += R"HTML(
<script>
async function poll(){
  try{
    const r = await fetch('/wifistatus.json', {cache:'no-store'});
    const d = await r.json();
    const el = document.getElementById('wstat');
    if(!el) return;
    if(d.connected){
      el.className = 'alarm ' + (d.internet ? 'ok' : 'warn');
      el.textContent = 'SSID: ' + d.ssid + ' | IP: ' + d.ip + (d.internet ? ' | Internet OK' : ' | no Internet');
    }else{
      el.className = 'alarm warn';
      el.textContent = 'Not connected yet...';
    }
  }catch(e){}
}
setInterval(poll, 1000);
poll();
</script>
)HTML";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}


void handleWiFiStatusJSON() {
  // No auth required; it's only connection status
  String json = "{";
  bool conn = (WiFi.status() == WL_CONNECTED);
  json += "\"connected\":" + String(conn ? "true" : "false") + ",";
  json += "\"pending\":" + String(g_wifiReconnectPending ? "true" : "false") + ",";
  json += "\"ssid\":\"" + (conn ? WiFi.SSID() : String("")) + "\",";
  json += "\"ip\":\"" + (conn ? WiFi.localIP().toString() : String("0.0.0.0")) + "\",";
  // lightweight internet check (may take up to timeout inside hasInternet)
  bool inet = conn ? hasInternet(1200) : false;
  json += "\"internet\":" + String(inet ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleRecal() {
  if (!requireAuth()) return;

  for (int i = 0; i < NCHAN; ++i) {
    uint16_t z = readMilliVoltsAvg(chans[i].pin, 32);
    chans[i].zero_mv = (z < 400) ? z : 0;
  }
  server.sendHeader("Location", "/settings");
  server.send(303);
}
void handleSaveZeros() { saveZerosToNVS(); server.sendHeader("Location", "/settings"); server.send(303); }
void handleClearZeros(){ clearZerosInNVS(); server.sendHeader("Location", "/settings"); server.send(303); }



// ---------------- HTTP: MySQL logger settings/actions ----------------

void handleMySQLSave() {
  if (!requireAuth()) return;

  mysqlCfg.enabled = server.hasArg("mqen");

  if (server.hasArg("mqint")) mysqlCfg.interval_min = (uint16_t)server.arg("mqint").toInt();
  if (mysqlCfg.interval_min < 1) mysqlCfg.interval_min = 1;
  if (mysqlCfg.interval_min > 1440) mysqlCfg.interval_min = 1440;
  g_mysqlLogPeriodMs = (uint32_t)mysqlCfg.interval_min * 60UL * 1000UL;

  if (server.hasArg("mqhost")) server.arg("mqhost").toCharArray(mysqlCfg.host, sizeof(mysqlCfg.host));
  if (server.hasArg("mqport")) mysqlCfg.port = (uint16_t)server.arg("mqport").toInt();
  if (server.hasArg("mquser")) server.arg("mquser").toCharArray(mysqlCfg.user, sizeof(mysqlCfg.user));
  if (server.hasArg("mqpass")) server.arg("mqpass").toCharArray(mysqlCfg.pass, sizeof(mysqlCfg.pass));
  if (server.hasArg("mqdb"))   server.arg("mqdb").toCharArray(mysqlCfg.db, sizeof(mysqlCfg.db));
  if (server.hasArg("mqtab"))  server.arg("mqtab").toCharArray(mysqlCfg.table, sizeof(mysqlCfg.table));

  if (mysqlCfg.port == 0) mysqlCfg.port = 3306;

  saveMySQLToNVS();
  mysqlDisconnect(); // force reconnect + re-ensure DB/table next time

  server.sendHeader("Location", "/settings");
  server.send(303);
}




void handleAuthSave() {
  if (!requireAuth()) return;

  bool enabled = server.hasArg("auth_en");
  String u = server.hasArg("au") ? server.arg("au") : g_authUser;
  String p = server.hasArg("ap") ? server.arg("ap") : "";

  u.trim();
  if (u.length() < 1) u = g_authUser;

  // If password empty, keep existing
  if (p.length() == 0) p = g_authPass;

  saveAuthToNVS(enabled, u, p);

  server.sendHeader("Location", "/settings");
  server.send(303);
}


void handleMySQLTest() {
  if (!requireAuth()) return;

  // force immediate try (does not change the periodic schedule)
  g_lastMySQLLogMs = 0;
  logBME280ToMySQLIfDue();

  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleMySQLDisc() {
  mysqlDisconnect();
  server.sendHeader("Location", "/settings");
  server.send(303);
}

void handleLtSave() {
  if (!requireAuth()) return;

  latestCfg.enabled = server.hasArg("lten");
  if (server.hasArg("ltint")) latestCfg.interval_min = (uint16_t)server.arg("ltint").toInt();
  if (latestCfg.interval_min < 1) latestCfg.interval_min = 1;
  if (latestCfg.interval_min > 1440) latestCfg.interval_min = 1440;

  saveLatestToNVS();

  // Trigger a new write soon after saving
  g_lastLatestWriteMs = 0;

  server.sendHeader("Location", "/settings");
  server.send(303);
}

// ---------------- Setup / Loop ----------------

// ---------------- Background tasks ----------------
static void samplerTask(void* arg) {
  const TickType_t delayTicks = pdMS_TO_TICKS(250); // 4 Hz sampling
  for (;;) {
    sampleAllNowInternal();
    g_lastSampleMs = millis();
    vTaskDelay(delayTicks);
  }
}

static void loggerTask(void* arg) {
  const TickType_t delayTicks = pdMS_TO_TICKS(500);
  for (;;) {
    // Run network/file operations off the main loop to avoid web UI stalls
    logBME280ToMySQLIfDue();
    writeLatestFilesIfDue();
    vTaskDelay(delayTicks);
  }
}

void setup() {
  Serial.begin(115200);

  // I2C (BME280)
  Wire.begin(21, 22);
  delay(10);
  bme_ok = bme.begin(0x76);
  if (!bme_ok) bme_ok = bme.begin(0x77);
  if (bme_ok) {
    bme.setSampling(
      Adafruit_BME280::MODE_NORMAL,
      Adafruit_BME280::SAMPLING_X1,   // temp
      Adafruit_BME280::SAMPLING_X1,   // pressure
      Adafruit_BME280::SAMPLING_X1,   // humidity
      Adafruit_BME280::FILTER_OFF,
      Adafruit_BME280::STANDBY_MS_1000
    );
    Serial.println("BME280 detected.");
  } else {
    Serial.println("BME280 not found at 0x76/0x77.");
  }

  // ADC config
  analogReadResolution(12);
  for (int i = 0; i < NCHAN; ++i) {
    if (chans[i].pin == 32 || chans[i].pin == 33) pinMode(chans[i].pin, INPUT_PULLDOWN);
    else                                          pinMode(chans[i].pin, INPUT); // 34/35 need ext pulldown
    analogSetPinAttenuation(chans[i].pin, ADC_11db);
  }

  setupWifi();

  // NTP (UTC)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");

  // Load persisted settings (gains + useZeros flag + saved zeros)
  migrateOldPinKeysToChannelKeysIfNeeded();
  loadChannelsFromNVS();
  applyAdcPinConfig();
  loadSettingsFromNVS();
  loadMySQLFromNVS();
  loadLatestFromNVS();

  // Filesystem for /latest.csv
  g_fs_ok = LittleFS.begin(true);
  if (!g_fs_ok) Serial.println("[FS] LittleFS mount failed");

  // mDNS so http://esp32.local works
  MDNS.begin("esp32");

  loadAuthFromNVS();

  pinMode(BOOT_PIN, INPUT_PULLUP);
  if (digitalRead(BOOT_PIN) == LOW) {
    g_authBypassThisBoot = true;
    Serial.println("[AUTH] BOOT held: bypassing settings auth for this boot.");
  }


  // If not using saved zeros or none present, measure baseline at boot
  bool needMeasure = !g_useSavedZeros;
  if (!needMeasure) {
    for (int i = 0; i < NCHAN; ++i) {
      if (prefs.begin("adcmon", true)) {
        uint32_t z = prefs.getUInt(chans[i].nvsKeyZero, 0);
        prefs.end();
        if (z == 0) { needMeasure = true; break; }
      }
    }
  }
  if (needMeasure) {
    delay(50);
    for (int i = 0; i < NCHAN; ++i) {
      uint16_t z = readMilliVoltsAvg(chans[i].pin, 32);
      chans[i].zero_mv = (z < 400) ? z : 0;
    }
    if (g_useSavedZeros) saveZerosToNVS();
  }

  // Routes
  server.on("/", handleRoot);
    server.on("/help", HTTP_GET, handleHelpPage);
server.on("/settings", HTTP_GET, handleSettingsPage);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/chsave", HTTP_POST, handleChSave);
  server.on("/wifisave", HTTP_POST, handleWiFiSave);
  server.on("/wifistatus.json", HTTP_GET, handleWiFiStatusJSON);
  server.on("/recal", HTTP_GET, handleRecal);
  server.on("/saveZeros", HTTP_GET, handleSaveZeros);
  server.on("/clearZeros", HTTP_GET, handleClearZeros);
  server.on("/authsave", HTTP_POST, handleAuthSave);
  server.on("/data.json", HTTP_GET, handleDataJSON);
  server.on("/mysqlsave", HTTP_POST, handleMySQLSave);
  server.on("/mysqltest", HTTP_GET, handleMySQLTest);
  server.on("/mysqldisc", HTTP_GET, handleMySQLDisc);
  server.on("/ltsave", HTTP_POST, handleLtSave);
  server.on("/latest.csv", HTTP_GET, handleLatestCSV);  server.begin();

// Start background tasks (dual-core): sampling + logging on core 0
xTaskCreatePinnedToCore(samplerTask, "samplerTask", 4096, nullptr, 2, &g_samplerTask, 0);
xTaskCreatePinnedToCore(loggerTask,  "loggerTask",  6144, nullptr, 1, &g_loggerTask,  0);

  Serial.println("Web server started");
}

void loop() {
  server.handleClient();
  processWifiReconnect();}
// ---------------- latest.csv generation + periodic write ----------------

static int idxByName(const char* wanted) {
  for (int i=0;i<NCHAN;i++){
    if (strcasecmp(chans[i].name, wanted) == 0) return i;
  }
  return -1;
}

String makeLatestCSV() {
  float v[4]; bool on[4]; float t,rh,p; String amsg, alvl;
  copyCache(v,on,t,rh,p,amsg,alvl);

  int i_acm   = idxByName("ACmains");
  int i_act   = idxByName("ACto12v");
  int i_sol12 = idxByName("Solar12v");
  int i_solin = idxByName("SolarINPUT");

  String s = "utc,local,temp_c,hum_pct,press_hpa,acmains_v,acmains_state,acto12v_v,acto12v_state,solar12v_v,solar12v_state,solarinput_v,solarinput_state,alarm\n";
  s += utcNowString(); s += ",";
  s += localUTCplus2(); s += ",";

  if (isnan(t))  s += "n/a"; else s += String(t,1);
  s += ",";
  if (isnan(rh)) s += "n/a"; else s += String(rh,1);
  s += ",";
  if (isnan(p))  s += "n/a"; else s += String(p,1);
  s += ",";

  auto appendChan = [&](int idx){
    if (idx < 0 || idx >= 4) { s += "n/a,n/a,"; return; }
    s += String(v[idx],3); s += ",";
    s += (on[idx] ? "ON" : "OFF"); s += ",";
  };

  appendChan(i_acm);
  appendChan(i_act);
  appendChan(i_sol12);
  appendChan(i_solin);

  s += "\""+amsg+"\"";
  s += "\n";
  return s;
}



static void writeLatestFilesIfDue() {
  if (!latestCfg.enabled) return;
  uint32_t periodMs = (uint32_t)latestCfg.interval_min * 60UL * 1000UL;
  if (millis() - g_lastLatestWriteMs < periodMs) return;
  if (!g_fs_ok) { g_lastLatestWriteMs = millis(); return; } // avoid tight loop if FS missing

  String csv = makeLatestCSV();
  File f = LittleFS.open("/latest.csv", "w");
  if (f) { f.print(csv); f.close(); }

  g_lastLatestWriteMs = millis();
}


void handleLatestCSV() {
  // Always generate dynamically from cached data (prevents stale CSV).
  server.send(200, "text/csv", makeLatestCSV());
}


