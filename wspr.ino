//***************************************************************************************
// TA1OZV WSPR BEACON - OPTIMIZE WEB SUNUCUSU, KOŞULLU NTP, YERINDE BILDIRIM & SIMETRIK KALIBRASYON
//
// Donanım:
// - Wemos D1 Mini / ESP-07S (ESP8266)
// - Si5351A Sentezör (I2C: D1 SCL [GPIO5], D2 SDA [GPIO4])
// - 16x2 LCD + PCF8574 I2C Backpack (I2C: D1 SCL, D2 SDA)
// - Dahili LED: GPIO2 (TX_LED_PIN - Aktif LOW)
// - RF Katı: TCXO Referanslı Si5351 -> BS170 PA -> LPF
//
// Kütüphaneler:
// - ESP8266WiFi, ESP8266WebServer, EEPROM, Wire, LiquidCrystal_I2C, Etherkit Si5351, Etherkit JTEncode
//***************************************************************************************

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <si5351.h>
#include <JTEncode.h>
#include <time.h>

// ======================================================================================
// 1. DONANIM VE PROTOKOL SABİTLERİ
// ======================================================================================
#define TONE_SPACING        146     // ~1.46 Hz Ton Aralığı
#define WSPR_DELAY          683     // WSPR Sembol Süresi (ms)
#define SYMBOL_COUNT        WSPR_SYMBOL_COUNT
#define TX_LED_PIN          2       // Dahili Mavi LED (Aktif LOW)

#define LCD_ADDR            0x27    // LCD I2C Adresi
#define EEPROM_MAGIC        0x63    // Güncel bellek yapısı kontrol baytı
#define EEPROM_SIZE         512
#define MAX_BANDS           6       // Maksimum bant sayısı

// ======================================================================================
// 2. KALICI BELLEK YAPISI (EEPROM STRUCT)
// ======================================================================================
struct CihazAyarlari {
  uint8_t magic;
  char call[10];
  char loc[7];
  int32_t cal_factor;
  char ssid[33];
  char password[65];
  char ntp_primary[33];
  char ntp_secondary[33];
  float time_zone;
  uint8_t period;          // 4, 6, 10, 30, 60, 120 veya 0
  uint8_t power_dbm;       // 0 - 30 dBm
  uint8_t multi_mode;      // 0: Tek Frekans, 1: Çoklu Frekans
  uint8_t band_count;      // 1 - 6 arası
  uint32_t freqs[MAX_BANDS];
};

CihazAyarlari ayar;

// ======================================================================================
// 3. GLOBAL NESNELER VE DURUM DEĞİŞKENLERİ
// ======================================================================================
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);
ESP8266WebServer server(80);
Si5351 si5351;
JTEncode jtencode;

uint8_t tx_buffer[SYMBOL_COUNT];
bool pre_encoded = false;
bool ap_modu = false;
int sonGosterilenSaniye = -1;
unsigned long toplamIletimSayisi = 0;
char sonIletimSaati[10] = "--:--:--";
uint8_t aktifBantIndeksi = 0;

// RAM Tabanlı TX Durumu (Açılışta ve her resette otomatik aktif başlar)
bool tx_aktif = true;

// Kalibrasyon Modu Değişkenleri
bool cal_rf_active = false;
int32_t cal_current_factor = 0;
uint8_t cal_drive_strength = 0; // Varsayılan: 2mA (Ölçüm güvenliği)

// Dahili zaman değişkenleri (lwIP SNTP)
time_t simdiEpoch;
struct tm zamanDetay;

void zamaniGuncelle() {
  time(&simdiEpoch);
  localtime_r(&simdiEpoch, &zamanDetay);
}

int suAnSaat()   { return zamanDetay.tm_hour; }
int suAnDakika() { return zamanDetay.tm_min; }
int suAnSaniye() { return zamanDetay.tm_sec; }

const char* frekansEtiketi(uint32_t f) {
  if (f == 7040100UL)   return "40m";
  if (f == 10140200UL)  return "30m";
  if (f == 14097100UL)  return "20m";
  if (f == 21096100UL)  return "15m";
  if (f == 28126100UL)  return "10m";
  if (f == 144490500UL) return "2m";
  return "Ozel";
}

void printTimestamp() {
  zamaniGuncelle();
  Serial.printf("[%02d:%02d:%02d] ", suAnSaat(), suAnDakika(), suAnSaniye());
}

// ======================================================================================
// 4. EEPROM YÖNETİMİ
// ======================================================================================
void varsayilanAyarlariYukle() {
  ayar.magic = EEPROM_MAGIC;
  strncpy(ayar.call, "TA1OZV", sizeof(ayar.call));
  strncpy(ayar.loc, "KN41", sizeof(ayar.loc));
  ayar.cal_factor = 134200;
  strncpy(ayar.ssid, "TAYFUN", sizeof(ayar.ssid));
  strncpy(ayar.password, "011105030", sizeof(ayar.password));
  strncpy(ayar.ntp_primary, "pool.ntp.org", sizeof(ayar.ntp_primary));
  strncpy(ayar.ntp_secondary, "time.google.com", sizeof(ayar.ntp_secondary));
  ayar.time_zone = 3.0f;
  ayar.period = 4;
  ayar.power_dbm = 10;
  ayar.multi_mode = 0;
  ayar.band_count = 1;
  ayar.freqs[0] = 14097100UL;
  for (int i = 1; i < MAX_BANDS; i++) ayar.freqs[i] = 0;
}

void ayarlariHafizadanOku() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, ayar);
  if (ayar.magic != EEPROM_MAGIC) {
    Serial.println(F("[EEPROM] Gecersiz veri, varsayilanlar yukleniyor..."));
    varsayilanAyarlariYukle();
    EEPROM.put(0, ayar);
    EEPROM.commit();
  } else {
    Serial.println(F("[EEPROM] Ayarlar basariyla okundu."));
  }
  cal_current_factor = ayar.cal_factor;
}

void ayarlariHafizayaKaydet() {
  ayar.magic = EEPROM_MAGIC;
  EEPROM.put(0, ayar);
  EEPROM.commit();
  Serial.println(F("[EEPROM] Ayarlar kaydedildi."));
}

// ======================================================================================
// 5. KALİBRASYON RF VE DONANIM MOTORU
// ======================================================================================
void applyCalibration(int32_t factor, bool rf_on, uint8_t drive_str) {
  cal_current_factor = factor;
  cal_rf_active = rf_on;
  cal_drive_strength = drive_str;

  si5351_drive drv = SI5351_DRIVE_8MA;
  if (drive_str == 0) drv = SI5351_DRIVE_2MA;
  else if (drive_str == 1) drv = SI5351_DRIVE_4MA;
  else if (drive_str == 2) drv = SI5351_DRIVE_6MA;

  si5351.drive_strength(SI5351_CLK0, drv);
  si5351.set_correction(cal_current_factor, SI5351_PLL_INPUT_XO);
  si5351.set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  si5351.pll_reset(SI5351_PLLA);

  if (cal_rf_active) {
    si5351.set_freq(1400000000ULL, SI5351_CLK0); // Sabit 14.000.000 Hz referans
    si5351.set_clock_pwr(SI5351_CLK0, 1);
    digitalWrite(TX_LED_PIN, LOW);
  } else {
    si5351.set_clock_pwr(SI5351_CLK0, 0);
    digitalWrite(TX_LED_PIN, HIGH);
  }

  printTimestamp();
  Serial.printf("[KALIBRASYON] RF: %s | Katsayi: %d | Drive: %d mA\n", cal_rf_active ? "ACIK (14MHz)" : "KAPALI", cal_current_factor, (drive_str + 1) * 2);
}

// ======================================================================================
// 6. FLASH BELLEKTE STATİK WEB ŞABLONLARI (PROGMEM)
// ======================================================================================
const char PAGE_HEAD[] PROGMEM =
"<!DOCTYPE html><html lang='tr'><head><meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
"<title>TA1OZV WSPR Kontrol Merkezi</title>"
"<style>"
"body{font-family:Segoe UI,Tahoma,sans-serif;background:#09090b;color:#f4f4f5;margin:0;padding:15px;}"
".container{max-width:760px;margin:auto;background:#18181b;padding:22px;border-radius:12px;border:1px solid #27272a;box-shadow:0 10px 30px rgba(0,0,0,0.6);position:relative;}"
"h2{text-align:center;color:#f4f4f5;margin:0 0 16px 0;letter-spacing:0.5px;}"
".nav-bar{display:flex;justify-content:center;gap:12px;margin-bottom:18px;}"
".nav-btn{background:#27272a;color:#a1a1aa;padding:8px 16px;border-radius:6px;text-decoration:none;font-weight:bold;font-size:13px;border:1px solid #3f3f46;transition:0.2s;}"
".nav-btn:hover,.nav-btn.active{background:#3f3f46;color:#ffffff;border-color:#52525b;}"
".dash-cards{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:18px;}"
".card{background:#09090b;padding:14px;border-radius:8px;border:1px solid #27272a;font-size:13px;line-height:1.6;}"
".badge{display:inline-block;padding:2px 8px;border-radius:4px;font-weight:bold;font-size:11px;background:#27272a;color:#d4d4d8;}"
".grid-form{display:grid;grid-template-columns:1fr 1fr;gap:12px;}"
"@media(max-width:600px){.dash-cards,.grid-form{grid-template-columns:1fr;}}"
".form-group{display:flex;flex-direction:column;}"
"label{font-size:12px;color:#a1a1aa;font-weight:600;margin-bottom:4px;display:flex;align-items:center;gap:4px;}"
".tip{cursor:help;color:#71717a;font-size:11px;}"
"input,select{box-sizing:border-box;padding:9px;border:1px solid #27272a;border-radius:6px;background:#09090b;color:#f4f4f5;font-size:13px;outline:none;}"
"input:focus,select:focus{border-color:#52525b;}"
".pw-box{display:flex;gap:6px;} .pw-box input{flex:1;} .btn-toggle{background:#27272a;color:#f4f4f5;border:1px solid #3f3f46;border-radius:6px;padding:0 12px;cursor:pointer;}"
".radio-box{display:flex;gap:15px;background:#09090b;padding:9px;border-radius:6px;border:1px solid #27272a;}"
".radio-box label{color:#f4f4f5;cursor:pointer;margin:0;}"
".band-row{display:flex;gap:8px;align-items:center;margin-top:6px;}"
".btn{width:100%;padding:12px;margin-top:14px;border:none;border-radius:6px;font-weight:bold;cursor:pointer;font-size:14px;transition:0.2s;}"
".btn-save{background:#27272a;color:#ffffff;border:1px solid #3f3f46;} .btn-save:hover{background:#3f3f46;}"
".btn-reboot{background:#18181b;color:#ef4444;border:1px solid #27272a;} .btn-reboot:hover{background:#27272a;}"
".btn-add{background:#18181b;color:#d4d4d8;border:1px solid #27272a;padding:6px 12px;width:auto;margin-top:6px;font-size:12px;}"
".btn-del{background:#18181b;color:#ef4444;border:1px solid #27272a;padding:6px 10px;width:auto;margin-top:0;font-size:12px;}"
".btn-tx-toggle{padding:10px;border:none;border-radius:6px;font-weight:bold;cursor:pointer;font-size:13px;width:100%;margin-top:8px;}"
".btn-tx-on{background:#3f1414;color:#fca5a5;border:1px solid #7f1d1d;} .btn-tx-off{background:#14301c;color:#86efac;border:1px solid #14532d;}"
".full-col{grid-column:1/-1;}"
".footer-sec{border-top:1px solid #27272a;margin-top:16px;padding-top:12px;}"
"#toast{display:none;position:fixed;top:20px;left:50%;transform:translateX(-50%);background:#14532d;color:#86efac;padding:12px 24px;border-radius:8px;border:1px solid #22c55e;font-weight:bold;box-shadow:0 4px 15px rgba(0,0,0,0.5);z-index:9999;}"
"</style></head><body><div id='toast'>✓ Ayarlar Başarıyla Kaydedildi!</div><div class='container'><h2>TA1OZV WSPR KONTROL MERKEZI</h2>";

const char PAGE_CALIBRATION_TEMPLATE[] PROGMEM =
"<div class='nav-bar'><a href='/' class='nav-btn'>📊 Genel Kontrol</a><a href='/kalibrasyon' class='nav-btn active'>🛠️ İleri Ayarlar (Kalibrasyon)</a></div>"
"<div class='card'>"
"<div style='font-size:14px;color:#a1a1aa;'>Referans Çıkış: <b>14.000.000 Hz (CLK0)</b></div>"
"<div class='ctrl-box' style='margin:12px 0;display:flex;gap:8px;justify-content:center;align-items:center;'>"
"<label style='font-size:13px;'>Sürüş Gücü: </label><select id='drive_sel' onchange='updateRf(currentRf)'>"
"<option value='0'>2 mA (En Düşük - SDR Güvenliği)</option>"
"<option value='1'>4 mA</option>"
"<option value='2'>6 mA</option>"
"<option value='3'>8 mA (Maksimum)</option></select>"
"</div>"
"<button id='btn_rf' class='btn-tx-toggle btn-tx-off' style='width:100%;padding:12px;font-size:14px;' onclick='toggleRf()'>📡 14 MHz Taşıyıcıyı Başlat (RF ON)</button>"
"<div class='status-text' id='rf_status' style='font-size:13px;color:#a1a1aa;margin-top:8px;'>Durum: <span style='color:#10b981;font-weight:bold;'>RF KAPALI</span></div>"
"</div>"
"<div class='card' style='margin-top:14px;text-align:center;'>"
"<div style='font-size:13px;color:#a1a1aa;'>Canlı Kalibrasyon Katsayısı (ppb)</div>"
"<div class='val-display' id='disp_val' style='font-size:32px;font-weight:bold;color:#f4f4f5;margin:10px 0;'>0</div>"
"<div style='display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-bottom:8px;'>"
"<button class='btn-save' style='padding:10px;' onclick='stepCal(1)'>+1</button>"
"<button class='btn-save' style='padding:10px;' onclick='stepCal(10)'>+10</button>"
"<button class='btn-save' style='padding:10px;' onclick='stepCal(100)'>+100</button>"
"<button class='btn-save' style='padding:10px;' onclick='stepCal(1000)'>+1000</button>"
"</div>"
"<div style='display:grid;grid-template-columns:repeat(4,1fr);gap:8px;margin-bottom:8px;'>"
"<button class='btn-save' style='padding:10px;' onclick='stepCal(-1)'>-1</button>"
"<button class='btn-save' style='padding:10px;' onclick='stepCal(-10)'>-10</button>"
"<button class='btn-save' style='padding:10px;' onclick='stepCal(-100)'>-100</button>"
"<button class='btn-save' style='padding:10px;' onclick='stepCal(-1000)'>-1000</button>"
"</div>"
"<div style='display:flex;gap:8px;justify-content:center;margin-top:12px;'>"
"<input type='number' id='inp_val' style='width:160px;padding:9px;border:1px solid #27272a;background:#09090b;color:#f4f4f5;border-radius:6px;'>"
"<button class='btn-save' style='width:auto;margin:0;padding:9px 14px;' onclick='setManualCal()'>Değeri Yükle</button>"
"</div>"
"<button class='btn btn-save' onclick='saveCalToEEPROM()'>💾 Bu Katsayıyı EEPROM'a Kalıcı Kaydet</button>"
"<div class='status-text' id='save_status' style='font-size:13px;color:#a1a1aa;margin-top:6px;'>SDR waterfall ekranında tam 14.000.000 Hz'i yakaladığınızda kaydedin.</div>"
"</div>"
"<script>"
"let currentVal = 0; let currentRf = 0;"
"function initPage(){"
"  fetch('/getcal').then(r=>r.json()).then(d=>{"
"    currentVal = d.factor; currentRf = d.rf;"
"    document.getElementById('disp_val').innerText = currentVal;"
"    document.getElementById('inp_val').value = currentVal;"
"    document.getElementById('drive_sel').value = d.drv;"
"    renderRfBtn();"
"  });"
"}"
"function renderRfBtn(){"
"  const btn = document.getElementById('btn_rf'); const stat = document.getElementById('rf_status');"
"  if(currentRf===1){"
"    btn.className='btn-tx-toggle btn-tx-on'; btn.innerText='🛑 14 MHz Taşıyıcıyı Kapat (RF OFF)';"
"    stat.innerHTML='Durum: <span style=\"color:#ef4444;font-weight:bold;\">RF YAYINDA</span>';"
"  }else{"
"    btn.className='btn-tx-toggle btn-tx-off'; btn.innerText='📡 14 MHz Taşıyıcıyı Başlat (RF ON)';"
"    stat.innerHTML='Durum: <span style=\"color:#10b981;font-weight:bold;\">RF KAPALI</span>';"
"  }"
"}"
"function sendUpdate(val, rf, drv){"
"  fetch(`/setcal?val=${val}&rf=${rf}&drv=${drv}`).then(r=>r.json()).then(d=>{"
"    currentVal = d.factor; currentRf = d.rf;"
"    document.getElementById('disp_val').innerText = currentVal;"
"    document.getElementById('inp_val').value = currentVal;"
"    renderRfBtn();"
"  });"
"}"
"function stepCal(s){ sendUpdate(currentVal + s, currentRf, document.getElementById('drive_sel').value); }"
"function setManualCal(){ sendUpdate(parseInt(document.getElementById('inp_val').value), currentRf, document.getElementById('drive_sel').value); }"
"function toggleRf(){ sendUpdate(currentVal, (currentRf===1)?0:1, document.getElementById('drive_sel').value); }"
"function updateRf(rf){ sendUpdate(currentVal, rf, document.getElementById('drive_sel').value); }"
"function saveCalToEEPROM(){"
"  fetch(`/savecal?val=${currentVal}`).then(r=>r.text()).then(m=>{"
"    document.getElementById('save_status').innerHTML='<span style=\"color:#10b981;font-weight:bold;\">✓ ' + m + '</span>';"
"  });"
"}"
"window.onload = initPage;"
"</script></div></body></html>";

// ======================================================================================
// 7. WEB SAYFASI İŞLEYİCİLERİ
// ======================================================================================
void handleRoot() {
  if (cal_rf_active) {
    cal_rf_active = false;
    si5351.set_clock_pwr(SI5351_CLK0, 0);
    digitalWrite(TX_LED_PIN, HIGH);
    Serial.println(F("[GÜVENLİK] Ana sayfaya dönüldü, Kalibrasyon RF susturuldu."));
  }

  zamaniGuncelle();
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent_P(PAGE_HEAD);

  String nav = F("<div class='nav-bar'><a href='/' class='nav-btn active'>📊 Genel Kontrol</a><a href='/kalibrasyon' class='nav-btn'>🛠️ İleri Ayarlar (Kalibrasyon)</a></div>");
  server.sendContent(nav);

  String ipStr = ap_modu ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
  uint32_t suAnkiFrekans = ayar.freqs[aktifBantIndeksi];

  String cards = F("<div class='dash-cards'><div class='card'>");
  cards += F("<b>İstasyon IP:</b> ");
  cards += ipStr;
  cards += (ap_modu ? F(" <span class='badge' style='color:#f59e0b;'>ACIL AP</span>") : F(" <span class='badge'>EV AĞI</span>"));
  cards += F("<br><b>Son TX Saati:</b> ");
  cards += String(sonIletimSaati);
  cards += F("<br><b>Toplam Gönderim:</b> ");
  cards += String(toplamIletimSayisi);
  cards += F(" kez<br><b>TX Durumu:</b> ");

  if (tx_aktif) {
    cards += F("<span class='badge' style='background:#14301c;color:#86efac;'>AKTIF (YAYINDA)</span>");
  } else {
    cards += F("<span class='badge' style='background:#3f1414;color:#fca5a5;'>DURDURULDU</span>");
  }

  cards += F("</div><div class='card'><b>Çalışma Modu:</b> ");
  cards += (ayar.multi_mode ? F("Dönüşümlü Çoklu Bant") : F("Tek Frekans"));
  cards += F("<br><b>Aktif Frekans:</b> ");
  cards += String(frekansEtiketi(suAnkiFrekans));
  cards += F(" (");
  cards += String(suAnkiFrekans);
  cards += F(" Hz)<br><b>Sistem Saati:</b> ");
  cards += (suAnSaat() < 10 ? "0" : "") + String(suAnSaat()) + ":" + (suAnDakika() < 10 ? "0" : "") + String(suAnDakika()) + ":" + (suAnSaniye() < 10 ? "0" : "") + String(suAnSaniye());
  cards += F("<br><form method='POST' action='/toggletx'>");

  if (tx_aktif) {
    cards += F("<button class='btn-tx-toggle btn-tx-on' type='submit'>🛑 Yayını Anında Durdur (TX Pause)</button>");
  } else {
    cards += F("<button class='btn-tx-toggle btn-tx-off' type='submit'>▶️ Yayını Başlat (TX Resume)</button>");
  }
  cards += F("</form></div></div>");
  server.sendContent(cards);

  String form = F("<form id='settings_form' method='POST' action='/save' class='grid-form' onsubmit='submitSettings(event)'>");
  form += F("<div class='form-group'><label>Çağrı İşareti <span class='tip'>ⓘ</span></label><input type='text' name='call' maxlength='9' value='");
  form += String(ayar.call);
  form += F("'></div><div class='form-group'><label>Grid Locator <span class='tip'>ⓘ</span></label><input type='text' name='loc' maxlength='6' value='");
  form += String(ayar.loc);
  form += F("'></div>");

  form += F("<div class='form-group'><label>WSPR Çıkış Gücü <span class='tip'>ⓘ</span></label><select name='power'>");
  struct GucSecenek { uint8_t dbm; const char* etiket; };
  GucSecenek gucler[] = {
    {0,  "0 dBm (1 mW)"},
    {3,  "3 dBm (2 mW)"},
    {7,  "7 dBm (5 mW)"},
    {10, "10 dBm (10 mW)"},
    {13, "13 dBm (20 mW)"},
    {17, "17 dBm (50 mW)"},
    {20, "20 dBm (100 mW)"},
    {23, "23 dBm (200 mW)"},
    {27, "27 dBm (500 mW)"},
    {30, "30 dBm (1 W)"}
  };
  for (const auto& g : gucler) {
    form += F("<option value='");
    form += String(g.dbm);
    form += F("'");
    if (ayar.power_dbm == g.dbm) form += F(" selected");
    form += F(">");
    form += g.etiket;
    form += F("</option>");
  }
  form += F("</select></div>");

  form += F("<div class='form-group'><label>Yayın Periyodu <span class='tip'>ⓘ</span></label><select name='period'>");
  form += F("<option value='4'");   if (ayar.period == 4)   form += F(" selected"); form += F(">Her 4 Dakikada Bir (Önerilen)</option>");
  form += F("<option value='6'");   if (ayar.period == 6)   form += F(" selected"); form += F(">Her 6 Dakikada Bir</option>");
  form += F("<option value='10'");  if (ayar.period == 10)  form += F(" selected"); form += F(">Her 10 Dakikada Bir</option>");
  form += F("<option value='30'");  if (ayar.period == 30)  form += F(" selected"); form += F(">Her 30 Dakikada Bir</option>");
  form += F("<option value='60'");  if (ayar.period == 60)  form += F(" selected"); form += F(">Saatte 1 Kez</option>");
  form += F("<option value='120'"); if (ayar.period == 120) form += F(" selected"); form += F(">2 Saatte 1 Kez</option>");
  form += F("<option value='0'");   if (ayar.period == 0)   form += F(" selected"); form += F(">Yayını Durdur (Beklemede Kal)</option>");
  form += F("</select></div>");

  form += F("<div class='form-group'><label>Birincil NTP <span class='tip'>ⓘ</span></label><input type='text' name='ntp1' maxlength='32' value='");
  form += String(ayar.ntp_primary);
  form += F("'></div><div class='form-group'><label>İkincil NTP <span class='tip'>ⓘ</span></label><input type='text' name='ntp2' maxlength='32' value='");
  form += String(ayar.ntp_secondary);
  form += F("'></div>");

  form += F("<div class='form-group'><label>Wi-Fi SSID <span class='tip'>ⓘ</span></label><input type='text' name='ssid' maxlength='32' value='");
  form += String(ayar.ssid);
  form += F("'></div><div class='form-group'><label>Wi-Fi Şifresi <span class='tip'>ⓘ</span></label><div class='pw-box'><input type='password' id='wifi_pass' name='pass' maxlength='64' value='");
  form += String(ayar.password);
  form += F("'><button type='button' class='btn-toggle' onclick='togglePw()'>👁</button></div></div>");

  form += F("<div class='form-group full-col'><label>UTC Saat Dilimi <span class='tip'>ⓘ</span></label><input type='text' name='tz' value='");
  form += String(ayar.time_zone, 1);
  form += F("'></div>");
  server.sendContent(form);

  String modes = F("<div class='full-col'><label>Frekans Çalışma Modu</label><div class='radio-box'>");
  modes += F("<label><input type='radio' name='mode' value='0' id='r_single'");
  if (ayar.multi_mode == 0) modes += F(" checked");
  modes += F(" onchange='updateMode()'> Tek Frekans</label>");
  modes += F("<label><input type='radio' name='mode' value='1' id='r_multi'");
  if (ayar.multi_mode == 1) modes += F(" checked");
  modes += F(" onchange='updateMode()'> Çoklu Frekans (Dönüşümlü)</label></div></div>");

  modes += F("<div class='full-col' id='sec_single'><label>Frekans Seçimi</label><select name='single_freq' id='s_freq' onchange='checkCustomSingle()'>");
  modes += F("<option value='14097100'");  if (ayar.freqs[0] == 14097100UL)  modes += F(" selected"); modes += F(">20m (14.097.100 Hz)</option>");
  modes += F("<option value='7040100'");   if (ayar.freqs[0] == 7040100UL)   modes += F(" selected"); modes += F(">40m (7.040.100 Hz)</option>");
  modes += F("<option value='10140200'");  if (ayar.freqs[0] == 10140200UL)  modes += F(" selected"); modes += F(">30m (10.140.200 Hz)</option>");
  modes += F("<option value='21096100'");  if (ayar.freqs[0] == 21096100UL)  modes += F(" selected"); modes += F(">15m (21.096.100 Hz)</option>");
  modes += F("<option value='28126100'");  if (ayar.freqs[0] == 28126100UL)  modes += F(" selected"); modes += F(">10m (28.126.100 Hz)</option>");
  modes += F("<option value='144490500'"); if (ayar.freqs[0] == 144490500UL) modes += F(" selected"); modes += F(">2m VHF (144.490.500 Hz)</option>");

  bool singleCustom = (ayar.freqs[0] != 14097100UL && ayar.freqs[0] != 7040100UL && ayar.freqs[0] != 10140200UL && ayar.freqs[0] != 21096100UL && ayar.freqs[0] != 28126100UL && ayar.freqs[0] != 144490500UL);
  modes += F("<option value='custom'"); if (singleCustom) modes += F(" selected"); modes += F(">Özel Frekans (Elle Giriş)</option></select>");
  modes += F("<input type='number' name='single_custom_val' id='s_custom_val' style='");
  modes += (singleCustom ? F("display:block;") : F("display:none;"));
  modes += F("margin-top:6px;' placeholder='Hz' value='");
  modes += String(ayar.freqs[0]);
  modes += F("'></div>");

  modes += F("<div class='full-col' id='sec_multi' style='display:none;'><label>Dönüşümlü Yayın Bantları (Maks. 6 Adet)</label><div id='multi_container'></div>");
  modes += F("<button type='button' class='btn btn-add' id='btn_add' onclick='addBandRow()'>+ Başka Bant Ekle</button></div>");
  modes += F("<div class='full-col'><button class='btn btn-save' type='submit'>Ayarları Kaydet</button></div></form>");

  modes += F("<div class='footer-sec'><form method='POST' action='/reboot'><button class='btn btn-reboot' onclick=\"return confirm('Cihaz yeniden başlatılsın mı?');\">Cihazı Yeniden Başlat (Reboot)</button></form></div>");
  server.sendContent(modes);

  String script = F("<script>const savedFreqs=[");
  for (int i = 0; i < ayar.band_count; i++) {
    script += String(ayar.freqs[i]) + (i < ayar.band_count - 1 ? "," : "");
  }
  script += F("];"
    "function togglePw(){const el=document.getElementById('wifi_pass');el.type=(el.type==='password')?'text':'password';}"
    "function updateMode(){const isM=document.getElementById('r_multi').checked;document.getElementById('sec_single').style.display=isM?'none':'block';document.getElementById('sec_multi').style.display=isM?'block':'none';}"
    "function checkCustomSingle(){const s=document.getElementById('s_freq');document.getElementById('s_custom_val').style.display=(s.value==='custom')?'block':'none';}"
    "function getBandOptions(val){const opts=[{v:'14097100',t:'20m (14.097.100 Hz)'},{v:'7040100',t:'40m (7.040.100 Hz)'},{v:'10140200',t:'30m (10.140.200 Hz)'},{v:'21096100',t:'15m (21.096.100 Hz)'},{v:'28126100',t:'10m (28.126.100 Hz)'},{v:'144490500',t:'2m VHF (144.490.500 Hz)'},{v:'custom',t:'Özel Frekans'}];"
    "let h='';let match=false;opts.forEach(o=>{if(o.v==val)match=true;h+=`<option value='${o.v}' ${o.v==val?'selected':''}>${o.t}</option>`;});if(!match&&val)h=h.replace(\"value='custom'\",\"value='custom' selected\");return h;}"
    "function addBandRow(val='14097100'){const c=document.getElementById('multi_container');if(c.children.length>=6)return;const idx=c.children.length;const div=document.createElement('div');div.className='band-row';div.id='row_'+idx;"
    "const isC=(val&&!['14097100','7040100','10140200','21096100','28126100','144490500'].includes(String(val)));"
    "div.innerHTML=`<select name='m_freq_${idx}' onchange='toggleCustom(this,${idx})'>${getBandOptions(val)}</select>`+`<input type='number' name='m_custom_${idx}' id='m_c_${idx}' placeholder='Hz' value='${val}' style='${isC?\"display:block\":\"display:none\"};width:120px;'>`+(idx>0?`<button type='button' class='btn btn-del' onclick='removeRow(${idx})'>Kaldır</button>`:'');"
    "c.appendChild(div);if(c.children.length>=6)document.getElementById('btn_add').style.display='none';}"
    "function removeRow(idx){const r=document.getElementById('row_'+idx);if(r)r.remove();document.getElementById('btn_add').style.display='inline-block';}"
    "function toggleCustom(sel,idx){document.getElementById('m_c_'+idx).style.display=(sel.value==='custom')?'block':'none';}"
    "function submitSettings(e){"
    "  e.preventDefault();"
    "  const form = document.getElementById('settings_form');"
    "  const data = new URLSearchParams(new FormData(form));"
    "  fetch('/save', {method:'POST', body:data})"
    "  .then(res=>res.text())"
    "  .then(msg=>{"
    "    const t = document.getElementById('toast');"
    "    t.style.display = 'block';"
    "    setTimeout(()=>{ t.style.display = 'none'; }, 2200);"
    "  });"
    "}"
    "window.onload=function(){updateMode();if(savedFreqs.length>0){savedFreqs.forEach(f=>addBandRow(f));}else{addBandRow('14097100');}};"
    "</script></div></body></html>");
  server.sendContent(script);
}

void handleCalibrationPage() {
  tx_aktif = false;
  pre_encoded = false;

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");
  server.sendContent_P(PAGE_HEAD);
  server.sendContent_P(PAGE_CALIBRATION_TEMPLATE);
}

void handleGetCal() {
  String json = "{\"factor\":" + String(cal_current_factor) + ",\"rf\":" + String(cal_rf_active ? 1 : 0) + ",\"drv\":" + String(cal_drive_strength) + "}";
  server.send(200, "application/json", json);
}

void handleSetCal() {
  if (server.hasArg("val")) cal_current_factor = server.arg("val").toInt();
  if (server.hasArg("rf"))  cal_rf_active = (server.arg("rf").toInt() == 1);
  if (server.hasArg("drv")) cal_drive_strength = server.arg("drv").toInt();

  applyCalibration(cal_current_factor, cal_rf_active, cal_drive_strength);

  String json = "{\"factor\":" + String(cal_current_factor) + ",\"rf\":" + String(cal_rf_active ? 1 : 0) + "}";
  server.send(200, "application/json", json);
}

void handleSaveCal() {
  if (server.hasArg("val")) {
    ayar.cal_factor = server.arg("val").toInt();
    ayarlariHafizayaKaydet();
    server.send(200, "text/plain", "Katsayi EEPROM'a Basariyla Kaydedildi!");
  } else {
    server.send(400, "text/plain", "Hata: Gecersiz Deger");
  }
}

void handleToggleTx() {
  if (cal_rf_active) {
    cal_rf_active = false;
    si5351.set_clock_pwr(SI5351_CLK0, 0);
    digitalWrite(TX_LED_PIN, HIGH);
  }

  tx_aktif = !tx_aktif;
  printTimestamp();
  Serial.printf("[KONTROL] TX Durumu Degistirildi: %s (Yalnizca RAM)\n", tx_aktif ? "AKTIF" : "DURDURULDU");

  server.sendHeader("Location", "/");
  server.send(303, "text/plain", "");
}

// KOŞULLU NTP VE SAYFA YENİLEMESİZ HAFİF KAYIT YÖNETİMİ
void handleSave() {
  if (server.hasArg("call")) strncpy(ayar.call, server.arg("call").c_str(), sizeof(ayar.call));
  if (server.hasArg("loc")) strncpy(ayar.loc, server.arg("loc").c_str(), sizeof(ayar.loc));
  if (server.hasArg("power")) ayar.power_dbm = server.arg("power").toInt();
  if (server.hasArg("period")) ayar.period = server.arg("period").toInt();
  if (server.hasArg("ssid")) strncpy(ayar.ssid, server.arg("ssid").c_str(), sizeof(ayar.ssid));
  if (server.hasArg("pass")) strncpy(ayar.password, server.arg("pass").c_str(), sizeof(ayar.password));

  // Koşullu NTP Kontrolü
  bool ntpDegisti = false;
  if (server.hasArg("ntp1") && strcmp(ayar.ntp_primary, server.arg("ntp1").c_str()) != 0) {
    strncpy(ayar.ntp_primary, server.arg("ntp1").c_str(), sizeof(ayar.ntp_primary));
    ntpDegisti = true;
  }
  if (server.hasArg("ntp2") && strcmp(ayar.ntp_secondary, server.arg("ntp2").c_str()) != 0) {
    strncpy(ayar.ntp_secondary, server.arg("ntp2").c_str(), sizeof(ayar.ntp_secondary));
    ntpDegisti = true;
  }
  if (server.hasArg("tz")) {
    float yeniTz = server.arg("tz").toFloat();
    if (ayar.time_zone != yeniTz) {
      ayar.time_zone = yeniTz;
      ntpDegisti = true;
    }
  }

  if (server.hasArg("mode")) ayar.multi_mode = server.arg("mode").toInt();

  if (ayar.multi_mode == 0) {
    ayar.band_count = 1;
    String sf = server.arg("single_freq");
    ayar.freqs[0] = (sf == "custom") ? server.arg("single_custom_val").toInt() : sf.toInt();
  } else {
    int sayac = 0;
    for (int i = 0; i < MAX_BANDS; i++) {
      String key = "m_freq_" + String(i);
      if (server.hasArg(key)) {
        String secim = server.arg(key);
        ayar.freqs[sayac] = (secim == "custom") ? server.arg("m_custom_" + String(i)).toInt() : secim.toInt();
        sayac++;
      }
    }
    ayar.band_count = (sayac > 0) ? sayac : 1;
    if (sayac == 0) ayar.freqs[0] = 14097100UL;
  }

  aktifBantIndeksi = 0;
  pre_encoded = false;
  ayarlariHafizayaKaydet();

  // Yalnızca NTP sunucusu veya saat dilimi değiştiyse saat motorunu güncelle
  if (ntpDegisti) {
    configTime((long)(ayar.time_zone * 3600), 0, ayar.ntp_primary, ayar.ntp_secondary);
    Serial.println(F("[SNTP] Zaman ayarları değiştiği için senkronizasyon yenilendi."));
  } else {
    Serial.println(F("[SNTP] Zaman ayarları değişmedi, gereksiz senkronizasyon atlandı."));
  }

  // Tarayıcıya HTML yönlendirmesi yerine hafif onay metni döndürülür
  server.send(200, "text/plain", "OK");
}

void handleReboot() {
  String html = F("<html><body style='background:#18181b;color:#ef4444;text-align:center;padding-top:50px;'><h2>Cihaz Yeniden Baslatiliyor...</h2><script>setTimeout(()=>{window.location.href='/';},12000);</script></body></html>");
  server.send(200, "text/html", html);
  delay(1000);
  ESP.restart();
}

// ======================================================================================
// 8. WI-FI VE KURTARMA AĞI
// ======================================================================================
void wifiKurulumu() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ayar.ssid, ayar.password);

  Serial.println(F("[WiFi] Agina baglanilmaya calisiliyor..."));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi Baglaniyor ");

  unsigned long baslangicZamani = millis();
  const unsigned long zamanAsimi = 150000UL;

  while (WiFi.status() != WL_CONNECTED && (millis() - baslangicZamani < zamanAsimi)) {
    digitalWrite(TX_LED_PIN, LOW);
    delay(250);
    digitalWrite(TX_LED_PIN, HIGH);
    delay(250);

    unsigned long gecenSn = (millis() - baslangicZamani) / 1000;
    lcd.setCursor(0, 1);
    char buf[17];
    snprintf(buf, sizeof(buf), "%-9s %3lus", ayar.ssid, 150 - gecenSn);
    lcd.print(buf);
    yield();
  }

  if (WiFi.status() == WL_CONNECTED) {
    ap_modu = false;
    Serial.println(F("[WiFi] Baglanti Basarili!"));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WiFi Baglandi!  ");
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP().toString());

    configTime((long)(ayar.time_zone * 3600), 0, ayar.ntp_primary, ayar.ntp_secondary);
    Serial.println(F("[SNTP] Dahili lwIP zaman senkronizasyonu devrede."));
    delay(1200);
  } else {
    ap_modu = true;
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    WiFi.softAP("TA1OZV_BEACON");

    Serial.println(F("[WiFi] Zaman asimi! Kurtarma AP Modu: TA1OZV_BEACON"));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ACIL DURUM AP   ");
    lcd.setCursor(0, 1);
    lcd.print("192.168.4.1     ");
    delay(2000);
  }
}

// ======================================================================================
// 9. WSPR İLETİM ÇEKİRDEĞİ (OTOMATIK 8 mA TAM GUC KORUMALI)
// ======================================================================================
void encodeAndTransmit() {
  if (cal_rf_active) return;

  uint32_t suAnkiFrekans = ayar.freqs[aktifBantIndeksi];
  si5351_clock aktifKanal = (suAnkiFrekans > 100000000UL) ? SI5351_CLK1 : SI5351_CLK0;

  // Kalibrasyonda 2mA kalsa dahi normal yayında 8mA'e zorlanır
  si5351.drive_strength(aktifKanal, SI5351_DRIVE_8MA);

  digitalWrite(TX_LED_PIN, LOW);
  printTimestamp();
  Serial.printf("[WSPR TX] BASLADI -> Frekans: %u Hz | Guc: %d dBm | Drive: 8mA | Kanal: CLK%d\n", suAnkiFrekans, ayar.power_dbm, (aktifKanal == SI5351_CLK1 ? 1 : 0));

  lcd.clear();
  lcd.setCursor(0, 0);
  char lBuf[17];
  snprintf(lBuf, sizeof(lBuf), "TX:%-5s %3ddBm", frekansEtiketi(suAnkiFrekans), ayar.power_dbm);
  lcd.print(lBuf);
  lcd.setCursor(0, 1);
  lcd.print("WSPR ILETIMI....");

  si5351.set_clock_pwr(aktifKanal, 1);

  for (uint8_t i = 0; i < SYMBOL_COUNT; i++) {
    si5351.set_freq((suAnkiFrekans * 100ULL) + (tx_buffer[i] * TONE_SPACING), aktifKanal);
    delay(WSPR_DELAY);
    yield();
  }

  si5351.set_clock_pwr(aktifKanal, 0);
  digitalWrite(TX_LED_PIN, HIGH);

  toplamIletimSayisi++;
  zamaniGuncelle();
  snprintf(sonIletimSaati, sizeof(sonIletimSaati), "%02d:%02d:%02d", suAnSaat(), suAnDakika(), suAnSaniye());

  printTimestamp();
  Serial.println(F("[WSPR TX] BASARIYLA TAMAMLANDI."));

  if (ayar.multi_mode == 1 && ayar.band_count > 1) {
    aktifBantIndeksi = (aktifBantIndeksi + 1) % ayar.band_count;
    printTimestamp();
    Serial.printf("[BAND] Siradaki Frekans: %s (%u Hz)\n", frekansEtiketi(ayar.freqs[aktifBantIndeksi]), ayar.freqs[aktifBantIndeksi]);
  }

  pre_encoded = false;
}

// ======================================================================================
// 10. SETUP
// ======================================================================================
void setup() {
  pinMode(TX_LED_PIN, OUTPUT);
  digitalWrite(TX_LED_PIN, LOW);

  // Açılışta veya her reset sonrası TX durumu otomatik olarak aktif başlar
  tx_aktif = true;

  Serial.begin(115200);
  delay(20);
  Serial.println(F("\n\n================================================"));
  Serial.println(F(" TA1OZV WSPR AKILLI BEACON - OPTIMIZE SON SURUM"));
  Serial.println(F("================================================"));

  Wire.begin(4, 5); // SDA: GPIO4 (D2), SCL: GPIO5 (D1)

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("TA1OZV WSPR IOT ");
  lcd.setCursor(0, 1);
  lcd.print("Baslatiliyor... ");

  ayarlariHafizadanOku();
  wifiKurulumu();

  // Web Rotaları
  server.on("/", HTTP_GET, handleRoot);
  server.on("/kalibrasyon", HTTP_GET, handleCalibrationPage);
  server.on("/getcal", HTTP_GET, handleGetCal);
  server.on("/setcal", HTTP_GET, handleSetCal);
  server.on("/savecal", HTTP_GET, handleSaveCal);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/toggletx", HTTP_POST, handleToggleTx);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.begin();
  Serial.println(F("[HTTP] Web Sunucusu 80 Portunda Baslatildi."));

  // Si5351 Başlatma
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  si5351.set_correction(ayar.cal_factor, SI5351_PLL_INPUT_XO);

  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_8MA);
  si5351.set_clock_pwr(SI5351_CLK0, 0);

  si5351.drive_strength(SI5351_CLK1, SI5351_DRIVE_8MA);
  si5351.set_clock_pwr(SI5351_CLK1, 0);

  Serial.println(F("[Si5351] Sentezor Hazir, PLL Kuruldu."));
  digitalWrite(TX_LED_PIN, HIGH);
}

// ======================================================================================
// 11. LOOP (ANA DÖNGÜ)
// ======================================================================================
void loop() {
  yield();
  server.handleClient();
  zamaniGuncelle();

  uint32_t suAnkiFrekans = ayar.freqs[aktifBantIndeksi];
  si5351_clock aktifKanal = (suAnkiFrekans > 100000000UL) ? SI5351_CLK1 : SI5351_CLK0;

  if (!cal_rf_active) {
    // 58. Saniyede Ön Kodlama
    if (ayar.period > 0 && tx_aktif && !pre_encoded) {
      int hedefDakika = suAnDakika() + 1;
      if ((hedefDakika % ayar.period == 0) && suAnSaniye() == 58) {
        jtencode.wspr_encode(ayar.call, ayar.loc, ayar.power_dbm, tx_buffer);
        si5351.set_freq(suAnkiFrekans * 100ULL, aktifKanal);
        si5351.set_clock_pwr(aktifKanal, 0);
        pre_encoded = true;

        printTimestamp();
        Serial.printf("[HAZIRLIK] Semboller hesaplandi. Frekans: %s\n", frekansEtiketi(suAnkiFrekans));
      }
    }

    // 01. Saniyede WSPR İletimi
    if (ayar.period > 0 && tx_aktif && pre_encoded) {
      if ((suAnDakika() % ayar.period == 0) && suAnSaniye() == 1) {
        encodeAndTransmit();
      }
    }

    if (!tx_aktif && pre_encoded) {
      pre_encoded = false;
    }
  }

  // 16x2 LCD Ekran Güncellemesi
  if (suAnSaniye() != sonGosterilenSaniye) {
    sonGosterilenSaniye = suAnSaniye();

    if (cal_rf_active) {
      lcd.setCursor(0, 0);
      lcd.print(">> KALIBRASYON <<");
      lcd.setCursor(0, 1);
      char cBuf[17];
      snprintf(cBuf, sizeof(cBuf), "14M CW: %+7d", cal_current_factor);
      lcd.print(cBuf);
    } else {
      char satir1[17];
      snprintf(satir1, sizeof(satir1), "%-6s  %02d:%02d:%02d", ayar.call, suAnSaat(), suAnDakika(), suAnSaniye());
      lcd.setCursor(0, 0);
      lcd.print(satir1);

      char satir2[17];
      if (!tx_aktif) {
        snprintf(satir2, sizeof(satir2), "TX: DURDURULDU  ");
      } else if (ayar.period == 0) {
        snprintf(satir2, sizeof(satir2), "PERIYOT: KAPALI ");
      } else {
        if ((suAnSaniye() % 6) < 3) {
          int gecenSn = (suAnDakika() % ayar.period) * 60 + suAnSaniye();
          int toplamPeriyotSn = ayar.period * 60;
          int kalanSn = toplamPeriyotSn - gecenSn;
          if (kalanSn < 0) kalanSn += toplamPeriyotSn;
          snprintf(satir2, sizeof(satir2), "%-3s:%-4s     %3ds", frekansEtiketi(suAnkiFrekans), ayar.loc, kalanSn);
        } else {
          snprintf(satir2, sizeof(satir2), "IP:%-13s", WiFi.localIP().toString().c_str());
        }
      }

      lcd.setCursor(0, 1);
      lcd.print(satir2);
    }
  }
}