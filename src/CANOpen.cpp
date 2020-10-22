//
// Created by Misha on 6/28/2020.
//

// Guide on using due_can: https://github.com/collin80/due_can/blob/master/howtouse.txt

#include "CANOpen.h"
#include <due_can.h>

CANOpenDevice::CANOpenDevice(CANRaw *can_line, uint16_t node_id) {
    this->can_line = can_line;
    this->node_id = node_id;

    rx_pdo_table = new PDOMap[PDO_RX_NUM];
    tx_pdo_table = new PDOMap[PDO_TX_NUM];

    for (int i = 0; i < PDO_RX_NUM; i++)
        rx_pdo_table[i].mappings = new PDOMapping[8];
    for (int i = 0; i < PDO_TX_NUM; i++)
        tx_pdo_table[i].mappings = new PDOMapping[8];

    tx_pdo_buffer = new BytesUnion[PDO_TX_NUM];
}

void CANOpenDevice::networkCommand(uint8_t cmd) {
    outgoing.id = COB_NMT;
    outgoing.extended = false;
    outgoing.length = 2;
    outgoing.data.byte[0] = cmd;
    outgoing.data.byte[1] = 0;

    can_line->sendFrame(outgoing);
}

void CANOpenDevice::sync() {
    outgoing.id = COB_SYNC + node_id;
    outgoing.extended = false;
    outgoing.length = 0;

    can_line->sendFrame(outgoing);
}

void CANOpenDevice::writeSDO(uint16_t index, uint8_t sub_index, uint8_t data_len, uint32_t data) {
    outgoing.id = COB_SDO_WRITE + node_id;
    outgoing.extended = false;
    outgoing.length = 8;
    outgoing.data.byte[0] = data_len;
    outgoing.data.byte[1] = (uint8_t) (index & 0x00FFU);
    outgoing.data.byte[2] = (uint8_t) ((index & 0xFF00U) >> 8U);
    outgoing.data.byte[3] = sub_index;
    outgoing.data.byte[4] = (uint8_t) ((data & 0x000000FFU) >> 0U);
    outgoing.data.byte[5] = (uint8_t) ((data & 0x0000FF00U) >> 8U);
    outgoing.data.byte[6] = (uint8_t) ((data & 0x00FF0000U) >> 16U);
    outgoing.data.byte[7] = (uint8_t) ((data & 0xFF000000U) >> 24U);

    can_line->sendFrame(outgoing);

    while (!can_line->available());

    can_line->read(incoming);

    uint32_t error;
    if (incoming.data.byte[0] != 0x80U)
        error = incoming.data.byte[4] | (incoming.data.byte[5] << 8U) | (incoming.data.byte[6] << 16U) |
                (incoming.data.byte[7] << 24U);
    else {
        Serial.print("Error writing SDO ");
        Serial.print(incoming.data.byte[2]<<8U | incoming.data.byte[1], HEX);
        Serial.print(":");
        Serial.print(incoming.data.byte[3]);
        Serial.print(", error code: ");
        Serial.println(incoming.data.high, HEX);
    }
}

void CANOpenDevice::readSDO(uint16_t index, uint8_t sub_index, uint32_t &data) {
    outgoing.id = COB_SDO_READ + node_id;
    outgoing.extended = false;
    outgoing.length = 8;
    outgoing.data.byte[0] = 0x40U;
    outgoing.data.byte[1] = (uint8_t) (index & 0x00FFU);
    outgoing.data.byte[2] = (uint8_t) ((index & 0xFF00U) >> 8U);
    outgoing.data.byte[3] = sub_index;
    outgoing.data.byte[4] = 0;
    outgoing.data.byte[5] = 0;
    outgoing.data.byte[6] = 0;
    outgoing.data.byte[7] = 0;

    can_line->sendFrame(outgoing);

    while (!can_line->available());

    can_line->read(incoming);

    if (incoming.data.byte[0] != 0x80U)
        data = incoming.data.byte[4] | (incoming.data.byte[5] << 8U) | (incoming.data.byte[6] << 16U) |
               (incoming.data.byte[7] << 24U);
    else {

        Serial.print("Error reading SDO ");
        Serial.print(incoming.data.byte[2]<<8U | incoming.data.byte[1], HEX);
        Serial.print(":");
        Serial.print(incoming.data.byte[3]);
        Serial.print(", error code: ");
        Serial.println(incoming.data.high, HEX);
    }
}

void
CANOpenDevice::configureRxPDO(uint8_t pdo_map_num, uint8_t trans_type, uint8_t map_count, PDOMapping const *mappings) {
    uint16_t cob_base;
    switch (pdo_map_num) {
        case 0:
            cob_base = PDO_RX_COB0;
            break;
        case 1:
            cob_base = PDO_RX_COB1;
            break;
        case 2:
            cob_base = PDO_RX_COB2;
            break;
        case 3:
            cob_base = PDO_RX_COB3;
            break;
        default:
            cob_base = 0x000U;

    }

    // Save PDO configuration locally
    rx_pdo_table[pdo_map_num].map_count = map_count;
    rx_pdo_table[pdo_map_num].cob_id = cob_base + node_id;
    unsigned int l = 0;
    for (int i = 0; i < map_count; i++) {
        rx_pdo_table[pdo_map_num].mappings[i] = mappings[i];
        l += mappings[i].bit_length;
    }
    rx_pdo_table[pdo_map_num].len = l / 8;

    // Deactivate mapping
    writeSDO(PDO_RX_CONFIG_COMM + pdo_map_num, 1, SDO_WRITE_4B, (1U<<31U) | (cob_base + node_id));
    delay(100);
    writeSDO(PDO_RX_CONFIG_MAP + pdo_map_num, 0, SDO_WRITE_1B, 0);
    delay(100);


    // Configure communication parameters for PDO on controller
    writeSDO(PDO_RX_CONFIG_COMM + pdo_map_num, 2, SDO_WRITE_1B, trans_type);


    // Configure content of PDO and map to object dictionary on controller
    for (int i = 0; i < map_count; i++) {
        uint32_t map = (((uint32_t) mappings[i].index) << 16U) |
                       (((uint32_t) mappings[i].sub_index) << 8U) |
                       ((uint32_t) mappings[i].bit_length);
        Serial.print("Configuring 0x");
        Serial.print(PDO_RX_CONFIG_MAP + pdo_map_num, HEX);
        Serial.print(":");
        Serial.print(i + 1);
        Serial.print(" with ");
        Serial.println(map, HEX);
        writeSDO(PDO_RX_CONFIG_MAP + pdo_map_num, i + 1, SDO_WRITE_4B, map);
    }

    // Re-enable mappings
    writeSDO(PDO_RX_CONFIG_MAP + pdo_map_num, 0, SDO_WRITE_1B, map_count);
    writeSDO(PDO_RX_CONFIG_COMM + pdo_map_num, 1, SDO_WRITE_4B, cob_base + node_id);
}

void CANOpenDevice::configureTxPDO(uint8_t pdo_map_num, uint8_t trans_type, uint16_t inhibit_time, uint16_t event_timer,
                                   uint8_t map_count, PDOMapping const *mappings) {
    uint16_t cob_base;
    switch (pdo_map_num) {
        case 0:
            cob_base = PDO_TX_COB0;
            break;
        case 1:
            cob_base = PDO_TX_COB1;
            break;
        case 2:
            cob_base = PDO_TX_COB2;
            break;
        case 3:
            cob_base = PDO_TX_COB3;
            break;
        default:
            cob_base = 0x000U;

    }

    // Save PDO configuration locally
    tx_pdo_table[pdo_map_num].map_count = map_count;
    tx_pdo_table[pdo_map_num].cob_id = cob_base + node_id;
    unsigned int l = 0;
    for (int i = 0; i < map_count; i++) {
        tx_pdo_table[pdo_map_num].mappings[i] = mappings[i];
        l += mappings[i].bit_length;
    }
    tx_pdo_table[pdo_map_num].len = l / 8;

    // Deactivate mapping
    writeSDO(PDO_TX_CONFIG_COMM + pdo_map_num, 1, SDO_WRITE_4B, (1U<<31U) | (cob_base + node_id));
    delay(100);
    writeSDO(PDO_TX_CONFIG_MAP + pdo_map_num, 0, SDO_WRITE_1B, 0);
    delay(100);


    // Configure communication parameters for PDO
    writeSDO(PDO_TX_CONFIG_COMM + pdo_map_num, 2, SDO_WRITE_1B, trans_type);
    writeSDO(PDO_TX_CONFIG_COMM + pdo_map_num, 3, SDO_WRITE_2B, inhibit_time);
    writeSDO(PDO_TX_CONFIG_COMM + pdo_map_num, 5, SDO_WRITE_2B, event_timer);

    // Configure content of PDO and map to object dictionary
    for (int i = 0; i < map_count; i++) {
        uint32_t map = (((uint32_t) mappings[i].index) << 16U) |
                       (((uint32_t) mappings[i].sub_index) << 8U) |
                       ((uint32_t) mappings[i].bit_length);

        Serial.print("Configuring 0x");
        Serial.print(PDO_TX_CONFIG_MAP + pdo_map_num, HEX);
        Serial.print(":");
        Serial.print(i + 1);
        Serial.print(" with ");
        Serial.println(map, HEX);

        writeSDO(PDO_TX_CONFIG_MAP + pdo_map_num, i + 1, SDO_WRITE_4B, map);
    }

    // Re-enable mappings
    writeSDO(PDO_TX_CONFIG_MAP + pdo_map_num, 0, SDO_WRITE_1B, map_count);
    writeSDO(PDO_TX_CONFIG_COMM + pdo_map_num, 1, SDO_WRITE_4B, cob_base + node_id);
}

void CANOpenDevice::update() {
    int c = 0;
    while (can_line->available() > 0 && c < 16) {

        can_line->read(incoming);
        for (int i = 0; i < PDO_TX_NUM; i++)
            if (incoming.id == tx_pdo_table[i].cob_id) {
                tx_pdo_buffer[i].value = incoming.data.value;
                break;
            }

        c++;
    }
}

void CANOpenDevice::readPDO(uint8_t pdo_map_num, BytesUnion &data) {
    data.value = tx_pdo_buffer[pdo_map_num].value;
}

void CANOpenDevice::writePDO(uint8_t pdo_map_num, const BytesUnion &data) {
    outgoing.id = rx_pdo_table[pdo_map_num].cob_id;
    outgoing.extended = false;
    outgoing.length = rx_pdo_table[pdo_map_num].len;
    outgoing.data.value = data.value;

    can_line->sendFrame(outgoing);
}

void CANOpenDevice::waitForBoot() {
    while (true) {
        while (!can_line->available());

        can_line->read(incoming);
        if(incoming.id == (0x700U + node_id)) {
            Serial.println("Controller has booted.");
            break;
        }
    }
}



