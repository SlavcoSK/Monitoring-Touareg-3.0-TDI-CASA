/*
 ╔═════════════════════════════════════════════════════════════════╗
 ║  VW Touareg 7P - 3.0 TDI CASA - Scanner v2.6 + Nextion HMI v2.9        ║
 ║  HW: ESP32-C5-DevKitC-1 + MCP2518FD + ATA6563 (20MHz Xtal)  ║
 ║  VIN: WVGZZZ7PZBD******                                        ║
 ║  Motor ECU: 7P0 907 401 / EDC17 H17 0011                      ║
 ║                                                                 ║
 ║  v2.1 ZMENY oproti v2.0:                                       ║
 ║   - OPRAVA: PIN_CAN_INT 0 -> 27 (GPIO0 je strapping pin,       ║
 ║     spôsoboval nespoľahlivý/žiadny príjem po nahratí do auta;  ║
 ║     GPIO27 bol overený ako funkčný v samostatnom teste)        ║
 ║   - Pridaný generický ISO-TP multi-frame prijímač (First       ║
 ║     Frame + Flow Control + Consecutive Frames) - potrebné      ║
 ║     pre VIN (20 bajtov) aj dlhšie UDS DID odpovede              ║
 ║   - Nové príkazy: v = VIN, e = Extended Session, t = Tester    ║
 ║     Present (predtým chýbali, hoci session/tester present      ║
 ║     interná logika existovala len vnútri DID scanneru)         ║
 ║   - OPRAVA v2.6: include vrátený na SoftwareSerial.h (skutočný  ║
 ║     názov header súboru knižnice EspSoftwareSerial - pôvodné    ║
 ║     "EspSoftwareSerial.h" v tomto riadku bola chyba, spôsobovala║
 ║     kompilačnú chybu "fatal error: No such file or directory") ║
 ║   - OBD_TIMEOUT_MS 150 -> 350 (EDC17 UDS potrebuje viac času)  ║
 ║   - read_did22() prerobené na použitie ISO-TP multi-frame      ║
 ║     receiveru (predtým fungovalo len pre odpovede <= 7 bajtov) ║
 ║                                                                 ║
 ║  v2.2 ZMENY oproti v2.1 (spresnenie pre EDC17CP44):            ║
 ║   - Automatický Tester Present každé 2s v loop(), kým je       ║
 ║     session_active - session sa už nezavrie počas normálnej    ║
 ║     prevádzky, nielen počas DID scanu                          ║
 ║   - cmd_read_vin() teraz pred VIN vždy otvorí extended session ║
 ║     + pošle tester present (niektoré EDC17 vracajú VIN iba     ║
 ║     v otvorenej diagnostickej session)                         ║
 ║   - Odstránený "while (Serial.available()) Serial.read();"     ║
 ║     flush v handle_serial_cmd() - mohol zožrať ďalší znak       ║
 ║   - pinMode(PIN_CAN_INT, INPUT_PULLUP) -> INPUT (ATA6563 má    ║
 ║     zvyčajne vlastný pull-up na INT linke)                     ║
 ║   - Limit total_len v ISO-TP prijímači 60 -> 250 bajtov a       ║
 ║     buffery buf[32] -> buf[256] v read_did22()/read_vin()      ║
 ║     (nielen VIN, aj niektoré DID môžu byť dlhšie)               ║
 ║   - Nový príkaz 'x' = návrat do Default Session (UDS 0x10 0x01)║
 ║                                                                 ║
 ║  v2.3 ZMENY oproti v2.2 (spätná väzba pre EDC17CP44):          ║
 ║   - open_extended_session() teraz kontroluje aj rx_buf[2]==0x03║
 ║     (potvrdenie správnej session, nie len 0x50 echo)           ║
 ║   - wait_response()/wait_frame() teraz rozpoznajú Response      ║
 ║     Pending (7F <SID> 78) a predĺžia čakanie namiesto toho,    ║
 ║     aby to brali ako finálnu (chybnú) odpoveď                  ║
 ║   - Prijímanie odpovedí z rozsahu 0x7E8-0x7EB (CASA niekedy    ║
 ║     odpovedá aj z iného ID než 0x7E8)                          ║
 ║   - ISO-TP First Frame kopírovanie cez min() namiesto ručnej   ║
 ║     kontroly                                                    ║
 ║   - read_did22() nahradené read_did22_ex(): rozlišuje OK /      ║
 ║     NEGATIVE (7F SID NRC) / TIMEOUT, DID scanner teraz loguje  ║
 ║     aj negatívne odpovede (pomáha pri mapovaní DPF/EGT PIDov)  ║
 ║   - Odstránený strop dlen<=4 bajty - DID scanner teraz číta až ║
 ║     32 bajtov na DID (predtým orezával dlhšie DPF/EGT dáta)    ║
 ║                                                                 ║
 ║  v2.4 ZMENY oproti v2.3 (spätná väzba pre EDC17CP44):          ║
 ║   - ISO-TP CF timeout sa teraz resetuje po KAŽDOM prijatom     ║
 ║     Consecutive Frame (predtým len raz na začiatku First Frame)║
 ║   - Kontrola sekvenčného čísla CF (SN) - pri poškodenej/       ║
 ║     nesprávnej sekvencii sa radšej vráti chyba než poskladaný  ║
 ║     nesprávny dátový blok                                       ║
 ║   - Response Pending (7F <SID> 78) sa spracúva aj počas         ║
 ║     čakania na Consecutive Frames, nielen na prvú odpoveď      ║
 ║   - Nová flush_can_rx(): vyprázdni RX frontu pred každým novým ║
 ║     UDS requestom (send_did22, send_pid01, read_vin,           ║
 ║     open_extended_session) - zabraňuje, aby "zaostalá"          ║
 ║     odpoveď (napr. na Tester Present) bola omylom prečítaná    ║
 ║     ako odpoveď na nasledujúci request                          ║
 ║   - read_vin() teraz kontroluje resp_did == 0xF190              ║
 ║   - DID scanner delay 5ms -> 2ms (rýchlejší priechod 0x0700-   ║
 ║     0x11FF)                                                     ║
 ║                                                                 ║
 ║  v2.5 ZMENY oproti v2.4 (spätná väzba pre EDC17CP44):          ║
 ║   - flush_can_rx() teraz časovo obmedzené (FLUSH_MAX_MS=5ms)   ║
 ║     namiesto vyprázdňovania "do posledného rámca" - pri         ║
 ║     rušnej zbernici by inak mohol flush blokovať príliš dlho   ║
 ║   - Kontrola dĺžky rámca (len<2) aj v ISO-TP Consecutive Frame ║
 ║     slučke - chráni pred zarušenými rámcami s DLC=1            ║
 ║   - wait_response() má voliteľnú SID validáciu (expected_sid)  ║
 ║     - zahodí rámec, ktorý patrí k inému/staršiemu requestu     ║
 ║   - Adaptívny delay v DID skeneri: pri NRC 0x21                ║
 ║     (busyRepeatRequest) sa čaká 20ms namiesto 2ms               ║
 ║   - DID scanner na konci vypíše počty NRC 0x21/0x22/0x33 -      ║
 ║     0x22 a 0x33 znamenajú, že DID PRAVDEPODOBNE EXISTUJE,      ║
 ║     len je momentálne nedostupný alebo vyžaduje Security Access║
 ║   - Opravená zabudnutá stará verzia (v2.1) v banneri a         ║
 ║     LogView hlavičke                                            ║
 ║   - Poznámka: FlowControl WAIT(0x31)/OVERFLOW(0x32) sa          ║
 ║     nespracúvajú - nie sú relevantné, keďže všetky naše         ║
 ║     requesty sa zmestia do jedného CAN rámca (Single Frame),   ║
 ║     takže ECU od nás flow control nikdy nepotrebuje             ║
 ║                                                                 ║
 ║  v2.6 ZMENY oproti v2.5 (finálne doladenie pred testom v aute):║
 ║   - wait_frame() teraz má rovnakú len<2 ochranu ako            ║
 ║     wait_response() - posledná chýbajúca obrana proti           ║
 ║     zarušeným DLC=1 rámcom                                      ║
 ║   - VIN timeout 1000ms -> 1500ms (niektoré CASA po prebudení   ║
 ║     reagujú pomalšie)                                           ║
 ║   - Tester Present v DID skeneri teraz aj každých 25 DID        ║
 ║     (predtým len časovo po 2s) - session sa nezavrie ani pri   ║
 ║     ECU, čo ju zatvárajú skôr než za 2s                         ║
 ║   - Eskalačný delay: ak sa počas skenu nazbiera > 500 NRC21     ║
 ║     (busyRepeatRequest), scanner natrvalo prejde na delay(50)  ║
 ║     pre zvyšok behu namiesto opakovaného krátkeho spamovania   ║
 ║   - DID scan výstup teraz vo formáte "Millis;DID;Status;Info"  ║
 ║     (CSV s časovou značkou) - umožňuje priamo porovnať logy    ║
 ║     z voľnobehu / 2500 ot./min / aktívnej DPF regenerácie      ║
 ╚═════════════════════════════════════════════════════════════════╝

 ZAPOJENIE MODUL MCP2518FD+ATA6563 → ESP32-C5-DevKitC-1 v1.2:
 ┌──────────────┬──────────────┬─────────────────────┐
 │ Modul pin    │ ESP32-C5 GPIO│ Pozicia na doske    │
 ├──────────────┼──────────────┼─────────────────────┤
 │ SCK          │ GPIO23       │ J3 pin 5            │
 │ SDI (MOSI)   │ GPIO24       │ J3 pin 4            │
 │ SDO (MISO)   │ GPIO25       │ J1 pin 13           │
 │ nCS          │ GPIO26       │ J1 pin 12           │
 │ INT          │ GPIO27       │ pozri poznamku nizsie│
 │ 5V           │ 5V           │ J1 pin 14           │
 │ 3V3          │ 3.3V         │ J1 pin 1            │
 │ GND          │ GND          │ J1 pin 15 (G)       │
 └──────────────┴──────────────┴─────────────────────┘
   POZOR: obe napajania (5V aj 3V3) musia byt zapojene!
   120R terminator na module NECHAT ODPOJENY (auto ma vlastnu terminaciu).
   INT pin je teraz na GPIO27 (nie GPIO0) - GPIO0 je strapping pin
   a spôsoboval problémy s príjmom po reštarte/nahratí v aute.

 OBD2 KONEKTOR → modul (svorkovnica H/L/G):
   Pin 6  (CAN H) → modul CANH (H)
   Pin 14 (CAN L) → modul CANL (L)
   Pin 4  (GND)   → modul GND  (G)
   (napajanie modulu 5V/3V3 z ESP dosky, nie z auta)

 NEXTION NX4832K035 → ESP32-C5 (SOFTVEROVY UART cez EspSoftwareSerial):
   Nextion RX  → GPIO15 (ESP TX)   [J3 pin 6]
   Nextion TX  → GPIO1  (ESP RX)   [J1 pin 6]
   5V          → 5V
   GND         → GND

 KNIŽNICE (Arduino IDE → Library Manager):
   - ACAN2517 by Pierre Molinaro   (NIE ACAN2517FD! - chceme klasicky CAN)
   - EspSoftwareSerial by Peter Lerup (plerup)

 SÉRIOVÝ PORT (115200 baud):
   - Ľudsky čitateľný výstup pri štarte
   - LogView CSV po spustení motora

 SÉRIOVÉ PRÍKAZY (odošli cez Serial Monitor):
   's' → Spusti DID skener (Mode 22, nájde VAG PIDs, motor musí bežať)
   'r' → Raw CAN monitor (30 sekúnd)
   'p' → Vypíš aktuálne hodnoty
   'd' → Debug mode ON/OFF
   'v' → Prečítaj VIN (Mode 22, DID 0xF190, ISO-TP multi-frame)
   'e' → Otvor Extended Diagnostic Session (UDS 0x10 0x03)
   't' → Pošli Tester Present (0x3E 0x80) manuálne
*/

#include <SPI.h>
#include <ACAN2517.h>
#include <SoftwareSerial.h>   // POZOR: knižnica sa v Library Manageri volá "EspSoftwareSerial",
                               // ale jej hlavičkový súbor sa aj tak volá SoftwareSerial.h

// Výsledok čítania UDS DID (Mode 22). MUSÍ byť definovaný tu, hneď za
// #include riadkami - Arduino IDE vkladá automaticky generované prototypy
// funkcií presne na toto miesto (za posledný #include/#define blok, pred
// prvý skutočný príkaz v súbore), takže akýkoľvek vlastný typ použitý v
// návratovej hodnote funkcie musí byť definovaný ešte skôr.
enum DidResult { DID_OK, DID_NEGATIVE, DID_TIMEOUT, DID_INVALID };

static EspSoftwareSerial::UART nextionSerial;

// ═══════════════════════════════════════════════════════════════
// KONFIGURÁCIA PINOV - ESP32-C5-DevKitC-1
// ═══════════════════════════════════════════════════════════════
#define PIN_SCK      23
#define PIN_MISO     25   // SDO na module
#define PIN_MOSI     24   // SDI na module
#define PIN_CAN_CS   26   // nCS
#define PIN_CAN_INT  27   // INT - OPRAVENÉ z GPIO0 (strapping pin) na GPIO27
#define PIN_NEX_TX   15
#define PIN_NEX_RX    1

// ═══════════════════════════════════════════════════════════════
// MCP2518FD cez ACAN2517 - oscilator 20 MHz, 500 kbps klasicky CAN
// ═══════════════════════════════════════════════════════════════
ACAN2517 can (PIN_CAN_CS, SPI, PIN_CAN_INT);

// ═══════════════════════════════════════════════════════════════
// KONFIGURÁCIA CAN / OBD2
// ═══════════════════════════════════════════════════════════════
#define OBD_TIMEOUT_MS  350   // zvýšené z 150 - EDC17 UDS potrebuje viac času
#define POLL_DELAY_MS    80
#define LOG_INTERVAL_MS 1000

#define CAN_OBD_BCAST   0x7DFul
#define CAN_ECM_REQ     0x7E0ul
#define CAN_ECM_RESP    0x7E8ul
#define CAN_ECM_RESP_LO 0x7E8ul
#define CAN_ECM_RESP_HI 0x7EBul   // CASA niekedy odpovedá aj z 0x7E9-0x7EB

static inline bool is_ecm_resp_id(uint32_t id) {
  return (id >= CAN_ECM_RESP_LO && id <= CAN_ECM_RESP_HI);
}
#define CAN_TCU_REQ     0x7E1ul
#define CAN_TCU_RESP    0x7E9ul

#define SVC_MODE01      0x01
#define SVC_MODE22      0x22
#define RESP_MODE01     0x41
#define RESP_MODE22     0x62
#define RESP_NEGATIVE   0x7F

// ═══════════════════════════════════════════════════════════════
// MODE 01 - ŠTANDARDNÉ OBD2 PIDs
// ═══════════════════════════════════════════════════════════════
struct PID_Def {
  uint8_t     pid;
  const char* name;
  const char* unit;
  uint8_t     bytes;
};

static const PID_Def PID_LIST[] = {
  { 0x0C, "RPM",            "/min",       2 },
  { 0x0D, "Speed",          "km/h",       1 },
  { 0x05, "Coolant",        "°C",         1 },
  { 0x0F, "IAT",            "°C",         1 },
  { 0x10, "MAF_gs",         "g/s",        2 },
  { 0x0B, "MAP_kPa",        "kPa",        1 },
  { 0x33, "Baro_kPa",       "kPa",        1 },
  { 0x5C, "Oil_temp",       "°C",         1 },
  { 0x23, "Rail_bar",       "bar",        2 },
  { 0x04, "Eng_load",       "%",          1 },
  { 0x5E, "Fuel_rate",      "l/h",        2 },
  { 0x2C, "EGR_cmd",        "%",          1 },
  { 0x2D, "EGR_err",        "%",          1 },
  { 0x11, "Throttle",       "%",          1 },
  { 0x1F, "Runtime",        "s",          2 },
};
static const uint8_t PID_COUNT = sizeof(PID_LIST) / sizeof(PID_LIST[0]);

static float   pid_vals[PID_COUNT];
static bool    pid_ok[PID_COUNT];
static uint32_t pid_ts[PID_COUNT];

// ═══════════════════════════════════════════════════════════════
// MODE 22 - VAG UDS DIDs (EDC17 - overované počas testovania)
// ═══════════════════════════════════════════════════════════════
struct DID_Def {
  uint16_t    did;
  const char* name;
  const char* unit;
  bool        verified;
};

static const DID_Def DID_LIST[] = {
  { 0x07D1, "MAF_req_mgstroke",        "mg/str",   false },
  { 0x07D2, "MAP_act_hPa",             "hPa",      false },
  { 0x07D3, "MAP_req_hPa",             "hPa",      false },
  { 0x107E, "Boost_act_hPa",           "hPa",      false },
  { 0x107F, "Boost_req_hPa",           "hPa",      false },
  { 0x1080, "VGT_act_pct",             "%",        false },
  { 0x07E0, "Inj_qty_main_mgstroke",   "mg/str",   false },
  { 0x07E1, "Inj_qty_req_mgstroke",    "mg/str",   false },
  { 0x07EA, "Rail_req_bar",            "bar",      false },
  { 0x1082, "Lambda_act",              "lambda",   false },
  { 0x1083, "Lambda_req",              "lambda",   false },
  { 0x1084, "O2_voltage_mV",           "mV",       false },
  { 0x1085, "EGT1_za_turbom_C",       "°C",        false },
  { 0x1086, "EGT2_EGR_chladič_C",     "°C",        false },
  { 0x1087, "EGT3_pred_DPF_C",        "°C",        false },
  { 0x1088, "Gearbox_oil_degC",       "°C",        false },
  { 0x1089, "DPF_dp_hPa",            "hPa",       false },
  { 0x108A, "DPF_soot_calc_g",       "g",         false },
  { 0x108B, "DPF_soot_meas_g",       "g",         false },
  { 0x108C, "DPF_ash_g",             "g",         false },
  { 0x108D, "DPF_dist_km",           "km",        false },
  { 0x108E, "DPF_time_s",            "s",         false },
  { 0x108F, "DPF_regen_active",      "bool",      false },
  { 0x1090, "EGR_pos_act_pct",       "%",         false },
  { 0x1091, "EGR_pos_req_pct",       "%",         false },
};
static const uint8_t DID_COUNT = sizeof(DID_LIST) / sizeof(DID_LIST[0]);

static float   did_vals[DID_COUNT] __attribute__((unused));
static bool    did_ok[DID_COUNT]   __attribute__((unused));

// ═══════════════════════════════════════════════════════════════
// HLAVNÁ DÁTOVÁ ŠTRUKTÚRA
// ═══════════════════════════════════════════════════════════════
struct EngineData {
  float    rpm;
  float    speed_kmh;
  float    coolant_c;
  float    oil_c;
  float    iat_c;
  float    engine_load_pct;
  uint32_t runtime_s;

  float    maf_actual_gs;
  float    maf_req_mgstroke;
  float    map_abs_kpa;
  float    baro_kpa;
  float    boost_bar;
  float    boost_req_bar;
  float    vgt_pct;

  float    egr_buzenie_pct;
  float    egr_otvorenie_pct;
  float    egr_err_pct;
  float    throttle_pct;

  float    rail_bar;
  float    rail_req_bar;
  float    fuel_rate_lh;
  float    inj_main_mgstroke;
  float    inj_req_mgstroke;
  float    post_inj_mgstroke;   // po namapovaní DID; počas regenerácie

  float    lambda_act;
  float    lambda_req;
  float    afr_from_lambda;
  float    afr_calculated;

  float    egt1_c;
  float    egt2_c;
  float    egt3_c;
  float    gearbox_c;

  float    dpf_dp_hpa;
  float    dpf_soot_calc_g;
  float    dpf_soot_meas_g;
  float    dpf_ash_g;
  float    dpf_dist_km;
  uint32_t dpf_time_s;
  bool     dpf_regen_active;

  bool     valid;
  uint32_t last_ms;
};

static EngineData E = {};

// ═══════════════════════════════════════════════════════════════
// GLOBÁLNE PREMENNÉ
// ═══════════════════════════════════════════════════════════════
static uint8_t  rx_buf[8];
static uint32_t rx_id;
static uint8_t  rx_len;

static bool     can_ok       = false;
static bool     debug_mode   = false;
static bool     log_active   = false;

static uint8_t  poll_idx     = 0;
static uint32_t last_poll_ms = 0;
static uint32_t last_log_ms  = 0;

static char     last_vin[18] = "";

static bool     session_active        = false;
static uint32_t last_tester_present_ms = 0;

// Nextion HMI v2.9
static uint8_t  nex_page = 0;
static uint8_t  nex_return_page = 0;
static bool     nex_regen_prev = false;
static uint32_t last_nextion_ms = 0;
static uint8_t  nex_rx_state = 0;

// ═══════════════════════════════════════════════════════════════
// CAN VRSTVA - postavena na kniznici ACAN2517 (MCP2518FD)
// ═══════════════════════════════════════════════════════════════
static bool mcp_send(uint32_t id, uint8_t len, const uint8_t* data) {
  if (len > 8) len = 8;

  CANMessage frame;
  frame.id  = id;
  frame.ext = false;      // OBD2 pouziva standardne 11-bit ID (0x7DF, 0x7E0...)
  frame.rtr = false;
  frame.len = len;
  for (uint8_t i = 0; i < len; i++) frame.data[i] = data[i];

  return can.tryToSend(frame);
}

static bool mcp_receive(uint32_t* out_id, uint8_t* out_len, uint8_t* out_data) {
  CANMessage frame;
  if (!can.receive(frame)) {
    return false;
  }

  *out_id  = frame.id;
  uint8_t len = frame.len;
  if (len > 8) len = 8;
  *out_len = len;
  for (uint8_t i = 0; i < len; i++) out_data[i] = frame.data[i];

  return true;
}

// ═══════════════════════════════════════════════════════════════
// SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 2000);

  print_banner();

  // Nextion - SOFTVEROVY UART (obchadza hardverovy Serial1/UART1,
  // ktory padal po pouziti SPI kvoli core-level chybe na ESP32-C5)
  nextionSerial.begin(9600, EspSoftwareSerial::SWSERIAL_8N1, PIN_NEX_RX, PIN_NEX_TX, false);
  delay(100);

  pinMode(PIN_CAN_CS, OUTPUT);
  digitalWrite(PIN_CAN_CS, HIGH);

  pinMode(PIN_CAN_INT, INPUT);   // ATA6563 moduly zvyčajne majú vlastný pull-up

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CAN_CS);
  delay(100);

  Serial.print(F("[CAN] Init MCP2518FD (ACAN2517, 500kbps, 20MHz, INT=GPIO27)... "));
  can_ok = can_init();

  if (can_ok) {
    Serial.println(F("OK"));
    nextion_goto_page(0);
    nextion_txt("t_state", "CAN OK");
  } else {
    Serial.println(F("CHYBA!"));
    Serial.println(F("[CAN] Skontroluj:"));
    Serial.println(F("      - Zapojenie SPI pinov (SCK23/SDI24/SDO25/nCS26/INT27)"));
    Serial.println(F("      - Obe napajania: 5V aj 3V3 na module"));
    Serial.println(F("      - Oscilator modulu: 20 MHz?"));
    nextion_goto_page(0);
    nextion_txt("t_state", "CAN CHYBA");
  }

  print_logview_header();

  Serial.println();
  Serial.println(F("[CMD] Príkazy: s=DID sken | r=Raw CAN | p=Print | d=Debug toggle | v=VIN | e=Ext.session | t=TesterPresent | x=DefaultSession"));
  Serial.println();
}

// ═══════════════════════════════════════════════════════════════
// LOOP
// ═══════════════════════════════════════════════════════════════
void loop() {
  handle_serial_cmd();
  nextion_service();

  if (!can_ok) {
    static uint32_t retry_t = 0;
    if (millis() - retry_t > 3000) {
      retry_t = millis();
      can_ok = can_init();
      if (can_ok) Serial.println(F("[CAN] Reconnected OK"));
    }
    if (millis() - last_nextion_ms >= 500) {
      last_nextion_ms = millis();
      nextion_update_all();
    }
    return;
  }

  if (session_active && (millis() - last_tester_present_ms > 2000)) {
    last_tester_present_ms = millis();
    send_tester_present();
  }

  if (millis() - last_poll_ms >= POLL_DELAY_MS) {
    last_poll_ms = millis();
    poll_next_pid();
  }

  if (millis() - last_nextion_ms >= 250) {
    last_nextion_ms = millis();
    calculate_derived();
    nextion_update_all();
  }

  if (millis() - last_log_ms >= LOG_INTERVAL_MS) {
    last_log_ms = millis();
    calculate_derived();

    log_active = (E.rpm > 100.0f);
    if (log_active) {
      print_logview_csv();
    } else {
      print_idle_status();
    }
  }
}

// ═══════════════════════════════════════════════════════════════
// CAN INICIALIZÁCIA
// ═══════════════════════════════════════════════════════════════
bool can_init() {
  ACAN2517Settings settings(ACAN2517Settings::OSC_20MHz, 500UL * 1000UL);
  settings.mRequestedMode = ACAN2517Settings::Normal20B;

  const uint32_t errorCode = can.begin(settings, [] { can.isr(); });

  if (errorCode != 0) {
    if (debug_mode) {
      Serial.printf("[CAN] begin() errorCode = 0x%lX\n", (unsigned long)errorCode);
    }
    return false;
  }
  return true;
}

// ═══════════════════════════════════════════════════════════════
// POLL NASLEDUJÚCI PID (Mode 01)
// ═══════════════════════════════════════════════════════════════
void poll_next_pid() {
  if (poll_idx >= PID_COUNT) {
    poll_idx = 0;
    return;
  }

  uint8_t pid = PID_LIST[poll_idx].pid;

  if (send_pid01(pid)) {
    bool got = wait_response(OBD_TIMEOUT_MS, CAN_ECM_RESP, RESP_MODE01);
    if (got && rx_buf[1] == RESP_MODE01 && rx_buf[2] == pid) {
      float val = decode_pid01(pid, &rx_buf[3]);
      pid_vals[poll_idx] = val;
      pid_ok[poll_idx]   = true;
      pid_ts[poll_idx]   = millis();
      store_pid(pid, val);

      if (debug_mode) {
        Serial.printf("[PID 0x%02X] %-12s = %8.2f %s\n",
                      pid, PID_LIST[poll_idx].name, val,
                      PID_LIST[poll_idx].unit);
      }
    } else {
      pid_ok[poll_idx] = false;
      if (debug_mode) {
        Serial.printf("[PID 0x%02X] %-12s = TIMEOUT\n", pid, PID_LIST[poll_idx].name);
      }
    }
  }

  poll_idx++;
}

// ═══════════════════════════════════════════════════════════════
// VYPRÁZDNENIE RX QUEUE - odstráni "zaostalé" rámce (napr. 7E 00
// odpoveď na Tester Present), ktoré by inak omylom prečítal
// wait_response()/wait_frame() ako odpoveď na ĎALŠÍ request.
// Časovo obmedzené (max. FLUSH_MAX_MS), aby pri rušnej zbernici
// (napr. veľa broadcast rámcov iných ECU) flush neblokoval dlho.
// ═══════════════════════════════════════════════════════════════
#define FLUSH_MAX_MS 5

void flush_can_rx() {
  uint32_t t0 = millis();
  uint32_t id;
  uint8_t  len;
  uint8_t  data[8];

  while ((millis() - t0) < FLUSH_MAX_MS) {
    if (!mcp_receive(&id, &len, data)) break;
    yield();
  }
}

// ═══════════════════════════════════════════════════════════════
// ODOSLANIE OBD2 MODE 01 REQUEST
// ═══════════════════════════════════════════════════════════════
bool send_pid01(uint8_t pid) {
  flush_can_rx();
  uint8_t msg[8] = { 0x02, SVC_MODE01, pid, 0x00, 0x00, 0x00, 0x00, 0x00 };
  return mcp_send(CAN_OBD_BCAST, 8, msg);
}

// ═══════════════════════════════════════════════════════════════
// ODOSLANIE VAG MODE 22 REQUEST (UDS)
// ═══════════════════════════════════════════════════════════════
bool send_did22(uint16_t did) {
  flush_can_rx();
  uint8_t dh = (did >> 8) & 0xFF;
  uint8_t dl = did & 0xFF;
  uint8_t msg[8] = { 0x03, SVC_MODE22, dh, dl, 0x00, 0x00, 0x00, 0x00 };
  return mcp_send(CAN_ECM_REQ, 8, msg);
}

// ═══════════════════════════════════════════════════════════════
// ČAKANIE NA CAN ODPOVEĎ (jeden rámec, ukladá do globálneho rx_buf)
// ═══════════════════════════════════════════════════════════════
// expected_sid: 0x00 = nekontroluj SID (akceptuj čokoľvek z daného ID rozsahu).
// Inak akceptuje len rámce, kde data[1]==expected_sid (positive response)
// alebo data[1]==0x7F (negatívna odpoveď - tú si vyhodnotí volajúci sám).
bool wait_response(uint32_t timeout_ms, uint32_t expected_id, uint8_t expected_sid) {
  uint32_t t0 = millis();
  uint32_t id;
  uint8_t len;
  uint8_t data[8];

  while (millis() - t0 < timeout_ms) {
    if (mcp_receive(&id, &len, data)) {
      bool match = (id == expected_id) ||
                   (expected_id == CAN_ECM_RESP && is_ecm_resp_id(id));
      if (match) {
        if (len < 2) { yield(); continue; }   // príliš krátky/zarušený rámec

        // Response Pending (7F <SID> 78) - ECU ešte pracuje, netreba to brať
        // ako konečnú odpoveď, len predĺžiť čakanie
        if (len >= 4 && data[1] == 0x7F && data[3] == 0x78) {
          t0 = millis();
          continue;
        }

        // Voliteľná SID validácia - zahoď rámec, ktorý zjavne patrí
        // k inému (staršiemu) requestu
        if (expected_sid != 0x00 && data[1] != expected_sid && data[1] != 0x7F) {
          continue;
        }

        rx_id  = id;
        rx_len = len;
        memcpy(rx_buf, data, len);
        return true;
      }
    }
    yield();
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════
// ISO-TP: čakanie na JEDEN konkrétny rámec (bez dotyku globálneho rx_buf)
// ═══════════════════════════════════════════════════════════════
bool wait_frame(uint32_t timeout_ms, uint32_t expected_id, uint8_t* out_data, uint8_t* out_len) {
  uint32_t t0 = millis();
  uint32_t id;
  uint8_t len;
  uint8_t data[8];

  while (millis() - t0 < timeout_ms) {
    if (mcp_receive(&id, &len, data)) {
      bool match = (id == expected_id) ||
                   (expected_id == CAN_ECM_RESP && is_ecm_resp_id(id));
      if (match) {
        if (len < 2) { yield(); continue; }   // zarušený/príliš krátky rámec (napr. DLC=1)

        // Response Pending (7F <SID> 78) - predĺž čakanie, nevracaj ako finálnu odpoveď
        if (len >= 4 && data[1] == 0x7F && data[3] == 0x78) {
          t0 = millis();
          continue;
        }
        *out_len = len;
        memcpy(out_data, data, len);
        return true;
      }
    }
    yield();
  }
  return false;
}

// ═══════════════════════════════════════════════════════════════
// ISO-TP: odoslanie Flow Control rámca (Continue To Send, BS=0, STmin=0)
// ═══════════════════════════════════════════════════════════════
void send_flow_control(uint32_t req_id) {
  uint8_t fc[8] = { 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  mcp_send(req_id, 8, fc);
}

// ═══════════════════════════════════════════════════════════════
// ISO-TP: generický príjem odpovede (Single Frame ALEBO First Frame +
// Flow Control + Consecutive Frames). Používa sa pre VIN a dlhšie DID.
// out_data musí mať aspoň 32 bajtov miesta.
// ═══════════════════════════════════════════════════════════════
bool read_uds_multiframe(uint32_t req_id, uint32_t resp_id,
                          uint8_t* out_data, uint16_t* out_len,
                          uint32_t timeout_ms) {
  uint8_t data[8];
  uint8_t len;

  if (!wait_frame(timeout_ms, resp_id, data, &len)) return false;

  uint8_t pci = (data[0] >> 4) & 0x0F;

  if (pci == 0x0) {
    // Single Frame
    uint8_t sf_len = data[0] & 0x0F;
    if (sf_len > 7) sf_len = 7;
    memcpy(out_data, &data[1], sf_len);
    *out_len = sf_len;
    return true;
  }

  if (pci == 0x1) {
    // First Frame - dlzka je v spodnych 12 bitoch (2 bajty PCI)
    uint16_t total_len = ((uint16_t)(data[0] & 0x0F) << 8) | data[1];
    if (total_len > 250) total_len = 250;   // bezpečný strop

    uint16_t copied = min((uint16_t)6, total_len);
    memcpy(out_data, &data[2], copied);

    // pošli Flow Control aby ECU pokračovalo v posielaní
    send_flow_control(req_id);

    uint32_t t0 = millis();
    uint8_t  expected_sn = 1;
    while (copied < total_len && (millis() - t0) < timeout_ms) {
      uint32_t id;
      if (mcp_receive(&id, &len, data)) {
        bool match = (id == resp_id) ||
                     (resp_id == CAN_ECM_RESP && is_ecm_resp_id(id));
        if (!match) { yield(); continue; }

        // Response Pending medzi rámcami CF (zriedkavé, ale niektoré EDC17 to robia)
        if (len >= 4 && data[1] == 0x7F && data[3] == 0x78) {
          t0 = millis();
          yield();
          continue;
        }

        if (len < 2) { yield(); continue; }   // zarušený/príliš krátky rámec (napr. DLC=1)

        if (((data[0] >> 4) & 0x0F) == 0x2) {
          uint8_t sn = data[0] & 0x0F;
          if (sn != expected_sn) {
            // poškodená/nesprávna sekvencia - zahoď čo máme, radšej timeout
            // ako poskladať nesprávny frame
            return false;
          }
          expected_sn = (expected_sn + 1) & 0x0F;

          uint8_t chunk = 7;
          if (copied + chunk > total_len) chunk = total_len - copied;
          memcpy(out_data + copied, &data[1], chunk);
          copied += chunk;
          t0 = millis();   // reset timeoutu po každom úspešnom CF
        }
      }
      yield();
    }

    *out_len = copied;
    return copied >= total_len;
  }

  return false; // Flow Control alebo neznamy PCI - nie je validna odpoved
}

// ═══════════════════════════════════════════════════════════════
// DEKÓDOVANIE MODE 01 PIDs → fyzikálne hodnoty
// ═══════════════════════════════════════════════════════════════
float decode_pid01(uint8_t pid, uint8_t* d) {
  uint8_t A = d[0], B = d[1];

  switch (pid) {
    case 0x04: return A * 100.0f / 255.0f;
    case 0x05: return (float)A - 40.0f;
    case 0x0B: return (float)A;
    case 0x0C: return (256.0f * A + B) / 4.0f;
    case 0x0D: return (float)A;
    case 0x0F: return (float)A - 40.0f;
    case 0x10: return (256.0f * A + B) / 100.0f;
    case 0x11: return A * 100.0f / 255.0f;
    case 0x1F: return 256.0f * A + B;
    case 0x23: return (256.0f * A + B) * 10.0f / 100.0f;  // kPa -> bar
    case 0x2C: return A * 100.0f / 255.0f;
    case 0x2D: return A * 100.0f / 128.0f - 100.0f;
    case 0x33: return (float)A;
    case 0x5C: return (float)A - 40.0f;
    case 0x5E: return (256.0f * A + B) * 0.05f;
    default:   return 0.0f;
  }
}

// ═══════════════════════════════════════════════════════════════
// ULOŽENIE PID HODNÔT DO ŠTRUKTÚRY
// ═══════════════════════════════════════════════════════════════
void store_pid(uint8_t pid, float v) {
  switch (pid) {
    case 0x0C: E.rpm             = v;           break;
    case 0x0D: E.speed_kmh       = v;           break;
    case 0x05: E.coolant_c       = v;           break;
    case 0x0F: E.iat_c           = v;           break;
    case 0x10: E.maf_actual_gs   = v;           break;
    case 0x0B: E.map_abs_kpa     = v;           break;
    case 0x33: E.baro_kpa        = v;           break;
    case 0x5C: E.oil_c           = v;           break;
    case 0x23: E.rail_bar        = v;           break;
    case 0x04: E.engine_load_pct = v;           break;
    case 0x5E: E.fuel_rate_lh    = v;           break;
    case 0x2C:
      E.egr_buzenie_pct   = v;
      E.egr_otvorenie_pct = 100.0f - v;
      break;
    case 0x2D: E.egr_err_pct = v; break;
    case 0x11: E.throttle_pct    = v;           break;
    case 0x1F: E.runtime_s       = (uint32_t)v; break;
  }
  E.valid   = true;
  E.last_ms = millis();
}

// ═══════════════════════════════════════════════════════════════
// VÝPOČET ODVOZENÝCH HODNÔT
// ═══════════════════════════════════════════════════════════════
void calculate_derived() {
  E.boost_bar = (E.map_abs_kpa - E.baro_kpa) / 100.0f;

  if (E.fuel_rate_lh > 0.05f && E.maf_actual_gs > 0.1f) {
    float fuel_gs = E.fuel_rate_lh * 835.0f / 3600.0f;
    float lambda  = E.maf_actual_gs / (fuel_gs * 14.5f);
    E.afr_calculated = lambda * 14.5f;
  } else {
    E.afr_calculated = 0.0f;
  }

  if (E.lambda_act > 0.1f) {
    E.afr_from_lambda = E.lambda_act * 14.5f;
  }
}

// ═══════════════════════════════════════════════════════════════
// MODE 22 - ČÍTANIE JEDNÉHO DID (ISO-TP multi-frame, detekcia 7F NRC)
// (DidResult enum je deklarovaný na začiatku súboru - Arduino IDE
//  generuje prototypy funkcií na vrchol súboru, takže typ použitý
//  v návratovej hodnote MUSÍ byť definovaný pred nimi)
// ═══════════════════════════════════════════════════════════════
DidResult read_did22_ex(uint16_t did, uint8_t* out_data, uint8_t* out_len,
                         uint8_t* neg_sid, uint8_t* neg_nrc) {
  if (!send_did22(did)) return DID_TIMEOUT;

  uint8_t buf[256];
  uint16_t len16;
  if (!read_uds_multiframe(CAN_ECM_REQ, CAN_ECM_RESP, buf, &len16, OBD_TIMEOUT_MS)) {
    return DID_TIMEOUT;
  }

  if (len16 < 2) return DID_INVALID;

  // Negatívna odpoveď: 7F <SID> <NRC>  (napr. 7F 22 31 = requestOutOfRange)
  if (buf[0] == 0x7F) {
    if (neg_sid) *neg_sid = buf[1];
    if (neg_nrc) *neg_nrc = (len16 >= 3) ? buf[2] : 0xFF;
    return DID_NEGATIVE;
  }

  if (len16 < 3) return DID_INVALID;
  if (buf[0] != RESP_MODE22) return DID_INVALID;

  uint16_t resp_did = ((uint16_t)buf[1] << 8) | buf[2];
  if (resp_did != did) return DID_INVALID;

  uint8_t dlen = len16 - 3;
  if (dlen > 32) dlen = 32;   // out_data musí mať aspoň 32 bajtov (viď run_did_scanner)
  for (uint8_t i = 0; i < dlen; i++) out_data[i] = buf[3 + i];

  *out_len = dlen;
  return DID_OK;
}

// Jednoduchý wrapper zachovaný kvôli spätnej kompatibilite (napr. budúce PID mapovanie)
bool read_did22(uint16_t did, uint8_t* out_data, uint8_t* out_len) {
  uint8_t neg_sid, neg_nrc;
  return read_did22_ex(did, out_data, out_len, &neg_sid, &neg_nrc) == DID_OK;
}

// ═══════════════════════════════════════════════════════════════
// ČÍTANIE VIN (DID 0xF190) cez ISO-TP multi-frame
// ═══════════════════════════════════════════════════════════════
bool read_vin(char* vin_out, uint8_t vin_out_size) {
  flush_can_rx();
  uint8_t req[8] = { 0x03, SVC_MODE22, 0xF1, 0x90, 0x00, 0x00, 0x00, 0x00 };
  if (!mcp_send(CAN_ECM_REQ, 8, req)) return false;

  uint8_t buf[256];
  uint16_t len16;
  if (!read_uds_multiframe(CAN_ECM_REQ, CAN_ECM_RESP, buf, &len16, 1500)) return false;

  if (len16 < 3) return false;
  if (buf[0] != RESP_MODE22) return false;

  uint16_t resp_did = ((uint16_t)buf[1] << 8) | buf[2];
  if (resp_did != 0xF190) return false;

  uint8_t vlen = len16 - 3;
  if (vlen > (vin_out_size - 1)) vlen = vin_out_size - 1;

  memcpy(vin_out, &buf[3], vlen);
  vin_out[vlen] = '\0';
  return true;
}

// ═══════════════════════════════════════════════════════════════
// LOGVIEW CSV HLAVIČKA
// ═══════════════════════════════════════════════════════════════
void print_logview_header() {
  Serial.println();
  Serial.println(F("//LogView"));
  Serial.println(F("// Touareg 3.0 TDI CASA - CAN Scanner v2.6"));
  Serial.println(F("// VIN: WVGZZZ7PZBD******"));
  Serial.println(F("// ECU: 7P0 907 401 / EDC17 H17 0011"));
  Serial.println(F("// Mode 01 PIDs + vypočítané hodnoty"));
  Serial.println(F("//"));
  Serial.println(
    F("Time_ms;RPM;Speed_kmh;Coolant_C;Oil_C;IAT_C;"
      "MAF_gs;MAP_kPa;Baro_kPa;Boost_bar;"
      "Rail_bar;Fuel_rate_lh;AFR_calc;"
      "EGR_buz_pct;EGR_otv_pct;EGR_err_pct;Throttle_pct;"
      "EGT1_turbo_C;EGT2_EGR_C;EGT3_DPF_C;"
      "DPF_dp_hPa;DPF_soot_g;DPF_regen;"
      "Eng_load_pct;Runtime_s")
  );
}

// ═══════════════════════════════════════════════════════════════
// LOGVIEW CSV RIADOK
// ═══════════════════════════════════════════════════════════════
void print_logview_csv() {
  char buf[280];
  snprintf(buf, sizeof(buf),
    "%lu;%.0f;%.0f;%.1f;%.1f;%.1f;"
    "%.2f;%.0f;%.0f;%.3f;"
    "%.1f;%.2f;%.2f;"
    "%.1f;%.1f;%.1f;%.1f;"
    "%.0f;%.0f;%.0f;"
    "%.1f;%.1f;%d;"
    "%.1f;%lu",
    millis(),
    E.rpm, E.speed_kmh, E.coolant_c, E.oil_c, E.iat_c,
    E.maf_actual_gs, E.map_abs_kpa, E.baro_kpa, E.boost_bar,
    E.rail_bar, E.fuel_rate_lh, E.afr_calculated,
    E.egr_buzenie_pct, E.egr_otvorenie_pct, E.egr_err_pct, E.throttle_pct,
    E.egt1_c, E.egt2_c, E.egt3_c,
    E.dpf_dp_hpa, E.dpf_soot_calc_g, (int)E.dpf_regen_active,
    E.engine_load_pct, E.runtime_s
  );
  Serial.println(buf);
}

// ═══════════════════════════════════════════════════════════════
// VÝPIS STAVU KEĎ MOTOR STOJÍ
// ═══════════════════════════════════════════════════════════════
void print_idle_status() {
  static uint8_t cnt = 0;
  if (++cnt < 5) return;
  cnt = 0;

  if (!E.valid) {
    Serial.println(F("[WAIT] Žiadne OBD2 dáta - zapni kľúč / motor"));
    return;
  }

  Serial.println(F("─────────────────────────────────────────────"));
  Serial.printf("[STATUS] Motor: STOJÍ | Kľúč: ON\n");
  Serial.printf("[STATUS] Coolant: %.1f°C | Oil: %.1f°C | IAT: %.1f°C\n",
                E.coolant_c, E.oil_c, E.iat_c);
  Serial.printf("[STATUS] Baro: %.0f kPa | MAP: %.0f kPa\n",
                E.baro_kpa, E.map_abs_kpa);
  Serial.println(F("[STATUS] Spusti motor pre plné logovanie..."));
}

// ═══════════════════════════════════════════════════════════════
// NEXTION HMI v2.9 - 8 logických obrazoviek
// ═══════════════════════════════════════════════════════════════
void nextion_cmd(const char* cmd) {
  nextionSerial.print(cmd);
  nextionSerial.write(0xFF); nextionSerial.write(0xFF); nextionSerial.write(0xFF);
}

void nextion_txt(const char* field, const char* val) {
  char buf[96];
  snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", field, val ? val : "--");
  nextion_cmd(buf);
}

void nextion_num(const char* field, float val, uint8_t dec) {
  if (!isfinite(val)) { nextion_txt(field, "--"); return; }
  char tmp[24], buf[64];
  dtostrf(val, 1, dec, tmp);
  snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", field, tmp);
  nextion_cmd(buf);
}

void nextion_color(const char* field, uint16_t rgb565) {
  char buf[48];
  snprintf(buf, sizeof(buf), "%s.pco=%u", field, rgb565);
  nextion_cmd(buf);
}

void nextion_progress(const char* field, float pct) {
  if (!isfinite(pct)) pct = 0.0f;
  pct = constrain(pct, 0.0f, 100.0f);
  char buf[48];
  snprintf(buf, sizeof(buf), "%s.val=%d", field, (int)lroundf(pct));
  nextion_cmd(buf);
}

void nextion_picture(const char* field, uint16_t picId) {
  char buf[48];
  snprintf(buf, sizeof(buf), "%s.pic=%u", field, picId);
  nextion_cmd(buf);
}

void nextion_goto_page(uint8_t page) {
  if (page > 7) return;
  char buf[16];
  snprintf(buf, sizeof(buf), "page %u", page);
  nextion_cmd(buf);
  nex_page = page;
}

// Nextion príkaz "sendme" vracia: 0x66, page_id, FF FF FF.
void nextion_service() {
  while (nextionSerial.available()) {
    uint8_t b = (uint8_t)nextionSerial.read();
    if (nex_rx_state == 0 && b == 0x66) {
      nex_rx_state = 1;
    } else if (nex_rx_state == 1) {
      if (b <= 7) nex_page = b;
      nex_rx_state = 0;
    }
  }
}

static bool nex_known(float v) { return isfinite(v) && fabsf(v) > 0.0001f; }

void nextion_update_all() {
  // Automatické otvorenie DPF stránky počas regenerácie.
  if (E.dpf_regen_active && !nex_regen_prev) {
    nex_return_page = nex_page;
    nextion_goto_page(5);
  } else if (!E.dpf_regen_active && nex_regen_prev && nex_page == 5) {
    nextion_goto_page(nex_return_page);
  }
  nex_regen_prev = E.dpf_regen_active;

  const uint16_t C_WHITE=65535, C_GREEN=2016, C_YELLOW=65504, C_RED=63488;

  switch (nex_page) {
    case 0: // PREHĽAD
      nextion_num("t_rpm", E.rpm, 0);
      nextion_num("t_speed", E.speed_kmh, 0);
      nextion_num("t_cool", E.coolant_c, 0);
      nextion_num("t_oil", E.oil_c, 0);
      nextion_num("t_boost", E.boost_bar, 2);
      if (nex_known(E.dpf_soot_calc_g)) nextion_num("t_soot", E.dpf_soot_calc_g, 1); else nextion_txt("t_soot", "--");
      if (!can_ok) { nextion_txt("t_state", "CAN ODPOJENÝ"); nextion_color("t_state", C_RED); }
      else if (E.dpf_regen_active) { nextion_txt("t_state", "DPF REGENERÁCIA"); nextion_color("t_state", C_YELLOW); }
      else if (E.rpm > 100.0f) { nextion_txt("t_state", "MOTOR BEŽÍ"); nextion_color("t_state", C_GREEN); }
      else { nextion_txt("t_state", E.valid ? "KĽÚČ ZAPNUTÝ" : "ČAKÁM NA ECU"); nextion_color("t_state", C_WHITE); }
      break;

    case 1: { // SPAĽOVANIE / AFR
      // MAF act je cez Mode 01 v g/s. Požadovaný mg/str ostáva -- do potvrdenia DID.
      nextion_num("t_mafA", E.maf_actual_gs, 1);
      if (nex_known(E.maf_req_mgstroke)) nextion_num("t_mafR", E.maf_req_mgstroke, 0); else nextion_txt("t_mafR", "--");
      if (nex_known(E.inj_main_mgstroke)) nextion_num("t_fuelA", E.inj_main_mgstroke, 1); else nextion_txt("t_fuelA", "--");
      if (nex_known(E.inj_req_mgstroke)) nextion_num("t_fuelR", E.inj_req_mgstroke, 1); else nextion_txt("t_fuelR", "--");
      if (nex_known(E.lambda_act)) nextion_num("t_lamA", E.lambda_act, 2); else nextion_txt("t_lamA", "--");
      if (nex_known(E.lambda_req)) nextion_num("t_lamR", E.lambda_req, 2); else nextion_txt("t_lamR", "--");
      if (nex_known(E.afr_from_lambda)) nextion_num("t_afr", E.afr_from_lambda, 1);
      else if (nex_known(E.afr_calculated)) nextion_num("t_afr", E.afr_calculated, 1);
      else nextion_txt("t_afr", "--");
      if (nex_known(E.lambda_act) && nex_known(E.lambda_req)) {
        float dev = (E.lambda_act - E.lambda_req) / E.lambda_req * 100.0f;
        nextion_num("t_afrDev", dev, 1);
        nextion_txt("t_combState", fabsf(dev) <= 12.0f ? "POMER VZDUCH/PALIVO V NORME" : "ODCHÝLKA LAMBDA > 12 %");
        nextion_color("t_combState", fabsf(dev) <= 12.0f ? C_GREEN : C_YELLOW);
      } else {
        nextion_txt("t_afrDev", "--");
        nextion_txt("t_combState", "LAMBDA/DID ZATIAĽ NENAMAPOVANÉ");
        nextion_color("t_combState", C_WHITE);
      }
      break;
    }

    case 2: // VSTREKY / PALIVO
      // Korekcie valcov zatiaľ nie sú vo v2.6 identifikované, preto žiadne falošné nuly.
      nextion_txt("t_inj1", "--"); nextion_txt("t_inj2", "--"); nextion_txt("t_inj3", "--");
      nextion_txt("t_inj4", "--"); nextion_txt("t_inj5", "--"); nextion_txt("t_inj6", "--");
      nextion_num("t_railA", E.rail_bar, 0);
      if (nex_known(E.rail_req_bar)) nextion_num("t_railR", E.rail_req_bar, 0); else nextion_txt("t_railR", "--");
      if (nex_known(E.inj_main_mgstroke)) nextion_num("t_injQty", E.inj_main_mgstroke, 1); else nextion_txt("t_injQty", "--");
      break;

    case 3: // VZDUCH / TURBO
      nextion_num("t_boostA", E.boost_bar, 2);
      if (nex_known(E.boost_req_bar)) nextion_num("t_boostR", E.boost_req_bar, 2); else nextion_txt("t_boostR", "--");
      nextion_num("t_map", E.map_abs_kpa, 0);
      nextion_num("t_baro", E.baro_kpa, 0);
      nextion_num("t_mafGs", E.maf_actual_gs, 1);
      if (nex_known(E.vgt_pct)) nextion_num("t_vgt", E.vgt_pct, 0); else nextion_txt("t_vgt", "--");
      nextion_num("t_iat", E.iat_c, 0);
      if (nex_known(E.boost_req_bar)) {
        float dev = E.boost_bar - E.boost_req_bar;
        nextion_txt("t_airState", fabsf(dev) < 0.20f ? "PLNIACI TLAK V NORME" : "ODCHÝLKA PLNIACEHO TLAKU");
        nextion_color("t_airState", fabsf(dev) < 0.20f ? C_GREEN : C_YELLOW);
      } else nextion_txt("t_airState", "MODE 01 OK, POŽIADAVKY ČAKAJÚ NA DID");
      break;

    case 4: // TEPLOTY
      nextion_num("t_cool", E.coolant_c, 0);
      nextion_num("t_oil", E.oil_c, 0);
      if (nex_known(E.gearbox_c)) nextion_num("t_gbox", E.gearbox_c, 0); else nextion_txt("t_gbox", "--");
      nextion_num("t_iat", E.iat_c, 0);
      nextion_txt("t_fuelTemp", "--");
      nextion_txt("t_oilLevel", "--");
      if (nex_known(E.egt1_c)) nextion_num("t_egt1", E.egt1_c, 0); else nextion_txt("t_egt1", "--");
      if (nex_known(E.egt2_c)) nextion_num("t_egt2", E.egt2_c, 0); else nextion_txt("t_egt2", "--");
      if (nex_known(E.egt3_c)) nextion_num("t_egt3", E.egt3_c, 0); else nextion_txt("t_egt3", "--");
      break;

    case 5: { // DPF – VAG DPF štýl
      float soot_pct = nex_known(E.dpf_soot_calc_g)
        ? constrain(E.dpf_soot_calc_g / DPF_SOOT_REGEN_THRESHOLD_G * 100.0f, 0.0f, 100.0f) : 0.0f;

      if (nex_known(E.dpf_soot_calc_g)) nextion_num("t_sootCalc", E.dpf_soot_calc_g, 1);
      else nextion_txt("t_sootCalc", "--");
      if (nex_known(E.dpf_soot_meas_g)) nextion_num("t_sootMeas", E.dpf_soot_meas_g, 1);
      else nextion_txt("t_sootMeas", "--");

      if (nex_known(E.dpf_dp_hpa)) nextion_num("t_dp", E.dpf_dp_hpa, 0);
      else nextion_txt("t_dp", "--");

      if (nex_known(E.egt1_c)) nextion_num("t_egt1", E.egt1_c, 0); else nextion_txt("t_egt1", "--");
      if (nex_known(E.egt2_c)) nextion_num("t_egt2", E.egt2_c, 0); else nextion_txt("t_egt2", "--");
      if (nex_known(E.egt3_c)) nextion_num("t_egt3", E.egt3_c, 0); else nextion_txt("t_egt3", "--");

      // Dodatočná dávka bude --, kým sa z logov nepotvrdí konkrétny DID a dekódovanie.
      if (nex_known(E.post_inj_mgstroke)) nextion_num("t_postInj", E.post_inj_mgstroke, 2);
      else nextion_txt("t_postInj", "--");

      // CASA používa obrátenú logiku: ECU hodnota 100 % znamená EGR úplne zatvorený.
      if (nex_known(E.egr_buzenie_pct) || E.egr_buzenie_pct == 0.0f)
        nextion_num("t_egrClosed", E.egr_buzenie_pct, 0);
      else nextion_txt("t_egrClosed", "--");

      if (nex_known(E.dpf_dist_km)) nextion_num("t_dist", E.dpf_dist_km, 0);
      else nextion_txt("t_dist", "--");

      nextion_progress("j_soot", soot_pct);
      nextion_num("t_sootPct", soot_pct, 0);
      nextion_txt("t_regen", E.dpf_regen_active ? "AKTÍVNA" : "NEAKT.");
      nextion_color("t_regen", E.dpf_regen_active ? C_RED : C_GREEN);

      // Postupné sčervenanie filtra používa VÝHRADNE EGT3 za katalyzátorom
      // (teplota plynov na vstupe DPF). EGT1 ani EGT2 sa ako náhrada nepoužívajú,
      // pretože by to mohlo falošne naznačiť prebiehajúcu regeneráciu.
      uint8_t heat_level = 0;
      if (nex_known(E.egt3_c)) {
        if      (E.egt3_c >= 600.0f) heat_level = 5;
        else if (E.egt3_c >= 500.0f) heat_level = 4;
        else if (E.egt3_c >= 400.0f) heat_level = 3;
        else if (E.egt3_c >= 300.0f) heat_level = 2;
        else if (E.egt3_c >= 200.0f) heat_level = 1;
      }

      // Pri chýbajúcom EGT3 ostane filter sivý bez ohľadu na EGT1/EGT2.
      nextion_picture("p_bg", 5 + heat_level);
      break;
    }

    case 6: { // ZDRAVIE
      bool cool_ok = !nex_known(E.coolant_c) || E.coolant_c < 110.0f;
      bool oil_ok  = !nex_known(E.oil_c) || E.oil_c < 125.0f;
      bool dpf_ok  = !nex_known(E.dpf_soot_calc_g) || E.dpf_soot_calc_g < 25.0f;
      nextion_txt("t_hComb", nex_known(E.lambda_act) ? "OK" : "ČAKÁ");
      nextion_txt("t_hInj", "ČAKÁ");
      nextion_txt("t_hTurbo", "OK");
      nextion_txt("t_hCool", (cool_ok && oil_ok) ? "OK" : "WARN");
      nextion_txt("t_hDpf", E.dpf_regen_active ? "REGEN" : (dpf_ok ? "OK" : "WARN"));
      nextion_txt("t_hGear", nex_known(E.gearbox_c) ? (E.gearbox_c < 115.0f ? "OK" : "WARN") : "ČAKÁ");
      nextion_txt("t_hBatt", "--");
      nextion_txt("t_hCan", can_ok ? "OK" : "CHYBA");
      float score = 100.0f;
      if (!can_ok) score -= 35.0f;
      if (!cool_ok || !oil_ok) score -= 25.0f;
      if (!dpf_ok) score -= 20.0f;
      nextion_progress("j_health", score);
      break;
    }

    case 7: // DIAGNOSTIKA
      nextion_txt("t_latency", "--");
      if (E.valid) nextion_num("t_age", millis() - E.last_ms, 0); else nextion_txt("t_age", "--");
      nextion_txt("t_rx", "--"); nextion_txt("t_tx", "--");
      nextion_txt("t_session", session_active ? "EXTENDED" : "DEFAULT");
      nextion_txt("t_nex", "ONLINE");
      nextion_txt("t_vin", last_vin[0] ? last_vin : "--");
      nextion_txt("t_fw", "SCANNER v2.6 + HMI v2.9");
      break;
  }
}

// ═══════════════════════════════════════════════════════════════
// SÉRIOVÉ PRÍKAZY
// ═══════════════════════════════════════════════════════════════
void handle_serial_cmd() {
  if (!Serial.available()) return;
  char c = Serial.read();

  switch (c) {
    case 's': run_did_scanner(); break;
    case 'r': run_raw_monitor(); break;
    case 'p': print_current_values(); break;
    case 'd':
      debug_mode = !debug_mode;
      Serial.printf("[DBG] Debug mode: %s\n", debug_mode ? "ON" : "OFF");
      break;
    case 'v': cmd_read_vin(); break;
    case 'e': cmd_open_session(); break;
    case 't': cmd_tester_present(); break;
    case 'x': cmd_default_session(); break;
    default: break;
  }
}

void cmd_read_vin() {
  // Niektoré EDC17 vracajú VIN iba v otvorenej diagnostickej session
  Serial.println(F("[VIN] Otváram extended session pred čítaním VIN..."));
  open_extended_session();
  delay(50);
  send_tester_present();
  delay(20);

  Serial.print(F("[VIN] Čítam VIN (DID 0xF190)... "));
  if (read_vin(last_vin, sizeof(last_vin))) {
    Serial.println(last_vin);
  } else {
    Serial.println(F("CHYBA / TIMEOUT"));
  }
}

void cmd_open_session() {
  Serial.print(F("[SESSION] Otváram Extended Diagnostic Session (UDS 0x10 0x03)... "));
  bool ok = open_extended_session();
  Serial.println(ok ? F("OK") : F("ZLYHALO"));
}

void cmd_tester_present() {
  Serial.print(F("[TESTER] Posielam Tester Present (0x3E 0x80)... "));
  send_tester_present();
  Serial.println(F("odoslané"));
}

void cmd_default_session() {
  uint8_t msg[8] = { 0x02, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
  mcp_send(CAN_ECM_REQ, 8, msg);
  session_active = false;
  Serial.println(F("[SESSION] Default session (0x10 0x01) odoslaná"));
}

// ═══════════════════════════════════════════════════════════════
// UDS DIAGNOSTIC SESSION CONTROL (0x10) - otvorenie rozsirenej relacie
// ═══════════════════════════════════════════════════════════════
bool open_extended_session() {
  flush_can_rx();
  uint8_t msg[8] = { 0x02, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00 };
  if (!mcp_send(CAN_ECM_REQ, 8, msg)) return false;

  bool got = wait_response(OBD_TIMEOUT_MS, CAN_ECM_RESP, 0x50);
  if (got && rx_buf[1] == 0x50 && rx_buf[2] == 0x03) {
    session_active = true;
    last_tester_present_ms = millis();
    return true;
  }
  return false;
}

// Tester Present (0x3E 0x80) - suppressPositiveResponse bit je nastavený,
// ale CASA/EDC17 ho nie vždy rešpektuje a aj tak môže poslať "7E8 02 7E 00".
// Preto sa pred každým ďalším requestom volá flush_can_rx().
void send_tester_present() {
  uint8_t msg[8] = { 0x02, 0x3E, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00 };
  mcp_send(CAN_ECM_REQ, 8, msg);
}

// ═══════════════════════════════════════════════════════════════
// DID SKENER
// ═══════════════════════════════════════════════════════════════
void run_did_scanner() {
  if (!can_ok) {
    Serial.println(F("[SCAN] CAN nie je OK!"));
    return;
  }

  Serial.print(F("[SCAN] Otvaram rozsirenu diagnosticku session (UDS 0x10 0x03)... "));
  bool session_ok = open_extended_session();
  Serial.println(session_ok ? F("OK") : F("ZLYHALO (skusam pokracovat aj tak)"));

  Serial.println(F("\n╔══════════════════════════════════════════╗"));
  Serial.println(F("║  MODE 22 DID SKENER - EDC17 CASA        ║"));
  Serial.println(F("║  Rozsah: 0x0700 - 0x11FF               ║"));
  Serial.println(F("║  Motor MUSÍ bežať!                      ║"));
  Serial.println(F("╚══════════════════════════════════════════╝\n"));
  Serial.println(F("Millis;DID;Status;Info"));   // CSV formát - ľahké porovnanie idle/2500rpm/regen logov

  uint16_t found     = 0;
  uint16_t negatives = 0;
  uint16_t nrc_busy_cnt      = 0;  // 0x21 busyRepeatRequest
  uint16_t nrc_secaccess_cnt = 0;  // 0x33 securityAccessDenied - DID EXISTUJE!
  uint16_t nrc_cond_cnt      = 0;  // 0x22 conditionsNotCorrect - DID pravdepodobne existuje
  uint8_t  data[32];
  uint8_t  dlen;
  uint8_t  neg_sid, neg_nrc;
  uint32_t last_tester_present = millis();
  bool     escalated_delay = false;   // aktivuje sa, ak NRC21 začne prevládať

  for (uint16_t did = 0x0700; did <= 0x11FF; did++) {

    // Tester Present časovo (2s) AJ počtom DID (25) - niektoré EDC17
    // zatvárajú session skôr, než by stihol prejsť čisto časový interval
    if (millis() - last_tester_present > 2000 || (did % 25 == 0)) {
      last_tester_present = millis();
      send_tester_present();
      yield();
    }

    if (did % 50 == 0) {
      Serial.printf("[SCAN] Progress: 0x%04X / 0x11FF  (nájdené: %d, negat.: %d, busy: %d)\n",
                    did, found, negatives, nrc_busy_cnt);
      delay(20);
      yield();

      if (Serial.available() && Serial.peek() == 'q') {
        Serial.println(F("[SCAN] Zastavené užívateľom."));
        break;
      }

      // Ak sa NRC21 nakopilo priveľa, ECU zjavne nestíha pri rýchlom skene -
      // natrvalo prejdi na pomalší delay pre zvyšok skenu
      if (!escalated_delay && nrc_busy_cnt > 500) {
        escalated_delay = true;
        Serial.println(F("[SCAN] Veľa NRC21 (busy) - spomaľujem na delay(50) pre zvyšok skenu."));
      }
    }

    DidResult res = read_did22_ex(did, data, &dlen, &neg_sid, &neg_nrc);
    uint32_t ts = millis();

    if (res == DID_OK) {
      found++;
      Serial.printf("%lu;0x%04X;OK;", ts, did);
      for (uint8_t i = 0; i < dlen; i++) Serial.printf("%02X", data[i]);
      Serial.println();
    } else if (res == DID_NEGATIVE) {
      // Negatívne odpovede sa hodia na mapovanie:
      //  0x31 requestOutOfRange   - DID pravdepodobne neexistuje
      //  0x22 conditionsNotCorrect- DID EXISTUJE, len momentálne nedostupný (napr. motor stojí)
      //  0x33 securityAccessDenied- DID EXISTUJE, ale vyžaduje Security Access (0x27)
      //  0x21 busyRepeatRequest   - ECU je zaneprázdnené, treba spomaliť a skúsiť znova
      negatives++;
      if (neg_nrc == 0x22) nrc_cond_cnt++;
      if (neg_nrc == 0x33) nrc_secaccess_cnt++;
      if (neg_nrc == 0x21) nrc_busy_cnt++;
      Serial.printf("%lu;0x%04X;NEG;SID=%02X_NRC=%02X\n", ts, did, neg_sid, neg_nrc);
    }
    // DID_TIMEOUT / DID_INVALID sa nevypisujú - je ich väčšina, len by zahltili výstup

    // Adaptívny delay: pri busyRepeatRequest (0x21) ECU potrebuje viac času.
    // Ak sa NRC21 nakopilo priveľa (escalated_delay), použi ešte pomalší delay natrvalo.
    if (escalated_delay) {
      delay(50);
    } else if (res == DID_NEGATIVE && neg_nrc == 0x21) {
      delay(20);
    } else {
      delay(2);
    }
  }

  Serial.printf("\n[SCAN] HOTOVO. Nájdené %d DID(s) s odpoveďou, %d negatívnych.\n", found, negatives);
  Serial.printf("[SCAN] Z toho NRC 0x22 (conditionsNotCorrect): %d, NRC 0x33 (securityAccessDenied): %d, NRC 0x21 (busy): %d\n",
                nrc_cond_cnt, nrc_secaccess_cnt, nrc_busy_cnt);
  Serial.println(F("[SCAN] DID s NRC 0x22/0x33 pravdepodobne EXISTUJU - skús ich znova pri inom stave motora / po Security Access."));
}

// ═══════════════════════════════════════════════════════════════
// RAW CAN MONITOR
// ═══════════════════════════════════════════════════════════════
void run_raw_monitor() {
  Serial.println(F("\n[RAW] CAN monitor - 30 sekúnd. Odošli 'q' pre zastavenie.\n"));
  Serial.println(F("Čas_ms;CAN_ID;DLC;Dáta(hex)"));

  uint32_t t_end = millis() + 30000UL;
  uint32_t count = 0;
  uint32_t id;
  uint8_t  len;
  uint8_t  data[8];

  while (millis() < t_end) {
    if (Serial.available() && Serial.read() == 'q') break;

    if (mcp_receive(&id, &len, data)) {
      Serial.printf("%lu;0x%03X;%d;", millis(), (unsigned int)id, len);
      for (uint8_t i = 0; i < len; i++) {
        Serial.printf("%02X", data[i]);
        if (i < len - 1) Serial.print(' ');
      }
      Serial.println();
      count++;
    }
    yield();
  }

  Serial.printf("\n[RAW] Koniec. Zachytených %lu rámcov.\n\n", count);
}

// ═══════════════════════════════════════════════════════════════
// VÝPIS AKTUÁLNYCH HODNÔT
// ═══════════════════════════════════════════════════════════════
void print_current_values() {
  Serial.println(F("\n══════════════ AKTUÁLNE HODNOTY ══════════════"));
  Serial.printf("  RPM              : %8.0f  /min\n",  E.rpm);
  Serial.printf("  Rýchlosť         : %8.0f  km/h\n",  E.speed_kmh);
  Serial.println(F("── Teploty ─────────────────────────────────"));
  Serial.printf("  Chladiaca kv.    : %8.1f  °C\n",    E.coolant_c);
  Serial.printf("  Olej motora      : %8.1f  °C\n",    E.oil_c);
  Serial.printf("  Nasávaný vzduch  : %8.1f  °C\n",    E.iat_c);
  Serial.println(F("── Vzduch & Turbo ───────────────────────────"));
  Serial.printf("  MAF skutočný     : %8.2f  g/s\n",   E.maf_actual_gs);
  Serial.printf("  MAP absolútny    : %8.1f  kPa\n",   E.map_abs_kpa);
  Serial.printf("  Atmosf. tlak     : %8.1f  kPa\n",   E.baro_kpa);
  Serial.printf("  Boost (rel.)     : %8.3f  bar\n",   E.boost_bar);
  Serial.printf("  EGR buzenie(raw) : %8.1f  %%  (100%%=zatvorený!)\n", E.egr_buzenie_pct);
  Serial.printf("  EGR otvorenie    : %8.1f  %%  (0%%=zat., 100%%=otv.)\n", E.egr_otvorenie_pct);
  Serial.printf("  EGR odchýlka     : %8.1f  %%\n",    E.egr_err_pct);
  Serial.println(F("── Palivo ───────────────────────────────────"));
  Serial.printf("  Rail tlak        : %8.1f  bar\n",   E.rail_bar);
  Serial.printf("  Spotreba         : %8.2f  l/h\n",   E.fuel_rate_lh);
  Serial.printf("  AFR (calc)       : %8.2f  :1\n",    E.afr_calculated);
  Serial.printf("  Záťaž motora     : %8.1f  %%\n",    E.engine_load_pct);
  if (strlen(last_vin) > 0) {
    Serial.printf("  Posledný VIN     : %s\n", last_vin);
  }
  Serial.println(F("═══════════════════════════════════════════════\n"));
}

// ═══════════════════════════════════════════════════════════════
// ÚVODNÝ BANNER
// ═══════════════════════════════════════════════════════════════
void print_banner() {
  Serial.println();
  Serial.println(F("╔══════════════════════════════════════════════╗"));
  Serial.println(F("║  Touareg 3.0 TDI CASA - CAN Scanner v2.6    ║"));
  Serial.println(F("║  VIN: WVGZZZ7PZBD******                    ║"));
  Serial.println(F("║  ECU: 7P0 907 401 / EDC17 H17 0011         ║"));
  Serial.println(F("╠══════════════════════════════════════════════╣"));
  Serial.println(F("║  HW: ESP32-C5 + MCP2518FD/ATA6563 (20MHz)  ║"));
  Serial.println(F("║  CAN: 500 kbps / OBD2 pin 6+14             ║"));
  Serial.println(F("║  INT: GPIO27 (opravené z GPIO0)            ║"));
  Serial.println(F("╚══════════════════════════════════════════════╝"));
}
