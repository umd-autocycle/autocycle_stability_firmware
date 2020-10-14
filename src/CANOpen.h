//
// Created by Misha on 6/28/2020.
// Library for communicating as master on a CANOpen network
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_CANOPEN_H
#define AUTOCYCLE_STABILITY_FIRMWARE_CANOPEN_H

#include <Arduino.h>
#include <due_can.h>

// CANOpen CAN-ID prefixes
#define COB_NMT         0x000U
#define COB_SYNC        0x080U
#define COB_PDO_WRITE   0x200U
#define COB_PDO_READ    0x180U
#define COB_SDO_WRITE   0x600U
#define COB_SDO_READ    0x580U
#define COB_BOOTUP      0x700U
#define COB_HEART       0x700U


// CANOpen Network Management (NMT) Command Codes
#define NMT_STATE_OPERATIONAL       0x01U
#define NMT_STATE_STOP              0x02U
#define NMT_STATE_PREOPERATIONAL    0x80U
#define NMT_RESET_NODE              0x81U
#define NMT_REST_COMM               0x82U


// CANOpen SDO read/write data length Command Codes
#define SDO_WRITE_1B        0x2FU
#define SDO_WRITE_2B        0x2BU
#define SDO_WRITE_3B        0x27U
#define SDO_WRITE_4B        0x23U
#define SDO_READ_1B         0x4FU
#define SDO_READ_2B         0x4BU
#define SDO_READ_3B         0x47U
#define SDO_READ_4B         0x43U

// CANOpen PDO Configuration object roots - (note: RX means read by CAN slave, TX means sent by CAN slave)
#define PDO_RX_CONFIG_COMM     0x1400U
#define PDO_RX_CONFIG_MAP      0x1600U
#define PDO_TX_CONFIG_COMM     0x1800U
#define PDO_TX_CONFIG_MAP      0x1A00U


// CANOpen PDO COB-ID bases
#define PDO_RX_COB0     0x200U
#define PDO_RX_COB1     0x300U
#define PDO_RX_COB2     0x400U
#define PDO_RX_COB3     0x500U
#define PDO_TX_COB0     0x180U
#define PDO_TX_COB1     0x280U
#define PDO_TX_COB2     0x380U
#define PDO_TX_COB3     0x480U


// CANOpen PDO Transmission types
#define PDO_RX_TRANS_SYNC       0x00U
#define PDO_RX_TRANS_ASYNC      0xFEU
#define PDO_TX_TRANS_SYNC_ACYC  0x00U
#define PDO_TX_TRANS_SYNC_CYC   0x01U
#define PDO_TX_TRANS_SYNC_RTR   0xFCU
#define PDO_TX_TRANS_ASYNC_RTR  0xFDU
#define PDO_TX_TRANS_ASYNC      0xFEU


// CANOpen RX/TX PDO counts
#define PDO_RX_NUM      8
#define PDO_TX_NUM      8

typedef struct {
    uint16_t index;
    uint8_t sub_index;
    uint8_t bit_length;
} PDOMapping;

typedef struct {
    uint8_t map_count;
    uint16_t cob_id;
    uint8_t len;
    PDOMapping *mappings;
} PDOMap;

class CANOpenDevice {
public:
    CANOpenDevice(CANRaw *can_line, uint16_t node_id);

    void networkCommand(uint8_t cmd);

    void sync();

    // void emergency();
    void writeSDO(uint16_t index, uint8_t sub_index, uint8_t data_len, uint32_t data);

    void readSDO(uint16_t index, uint8_t sub_index, uint32_t &data);

    void configureRxPDO(uint8_t pdo_map_num, uint8_t trans_type, uint8_t map_count, PDOMapping const mappings[]);

    void configureTxPDO(uint8_t pdo_map_num, uint8_t trans_type, uint16_t inhibit_time, uint16_t event_timer,
                        uint8_t map_count, PDOMapping const mappings[]);

    void update();

    void readPDO(uint8_t pdo_map_num, BytesUnion &data);

    void writePDO(uint8_t pdo_map_num, const BytesUnion &data);

    BytesUnion *tx_pdo_buffer;

private:
    CANRaw *can_line;
    CAN_FRAME incoming, outgoing;
    uint16_t node_id;
    PDOMap *rx_pdo_table, *tx_pdo_table;

};


#endif //AUTOCYCLE_STABILITY_FIRMWARE_CANOPEN_H
