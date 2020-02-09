#ifndef LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED
#define LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED

// UPDATE WITH YOUR TTN KEYS AND ADDR.
// ABP
// t-beam-sf11
static PROGMEM u1_t NWKSKEY[16] = {0x1F, 0x81, 0xDF, 0x84, 0x79, 0x96, 0x04, 0x1C, 0x57, 0x96, 0xBC, 0x85, 0x02, 0x9F, 0xEA, 0xB6}; // LoRaWAN NwkSKey, network session key
static u1_t PROGMEM APPSKEY[16] = {0xD9, 0xDE, 0xC2, 0x7B, 0xEA, 0x19, 0x31, 0x1D, 0xD8, 0x7F, 0xC6, 0x0F, 0x34, 0xBC, 0x6F, 0x7E}; // LoRaWAN AppSKey, application session key
static const u4_t DEVADDR = 0x26021C90;                                                                                             // LoRaWAN end-device address (DevAddr)

#endif //LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED