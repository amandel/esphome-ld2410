#include "esphome.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/log.h"
#include "esphome/components/number/number.h"
#include "esphome/components/template/number/template_number.h"

// todo NAMING!!!

/*
 * Alias:
 *
 * static also known as stillness, resting, stationary, rest
 * motion also known as movement, exercise
 * unmanned also known as "no time" == unoccupied delay time
 *
 * 100% sensitivity means disabled - might need to be changed!
 *
 * Pull down useful for OUT GPIO to avoid false readings?
 *
 * gate
 */


class VersionSensor : public TextSensor { // , public Sensor {
public:
    VersionSensor() : TextSensor() {
        // https://esphome.io/api/classesphome_1_1_entity_base.html#a3a714cc57822a7d5fd1df3f045ffa9f4
        set_disabled_by_default(true);
        set_internal(false);
        set_name("Version Number");
        publish_state("NO INFO");
    };
};

class TextInfoSensor : public TextSensor { // , public Sensor {
public:
    TextInfoSensor() : TextSensor() { };
};

class ConfigNumber : public number::Number, public NumberTraits, public Component {
public:
    ConfigNumber(std::function<void(float num)> &&f) : change(f) { }
    std::function<void(float num)> change;
protected:
    void control(float value) override { change(value); }

};

class ConfigSwitch : public Switch, public Component {
public:
    ConfigSwitch(std::function<void(bool state)> &&f) : stateChange(f) {
    }
    std::function<void(bool state)> stateChange;

    void setup() override {
        this->publish_state(1);
    }

protected:
    void write_state(bool state) override { stateChange(state); }
};


class LD2410 : public Component, public UARTDevice {
public:

//    ConfigNumber *maxConfigDistance
//            = new ConfigNumber([this](float n) { this->setMaxDistancesAndNoneDuration(n, n, id(noneDuration).state); });
//    ConfigNumber *noneDuration = new Sensor();
    static const int DATA_BUFFER_MAX_SIZE = 64;
    static const int NUMBER_OF_GATES = 9; // 0 - 8
    static const int INDEX_OF_LAST_GATE = 8; // 0 - 8
    static const int GATE_SIZE_CM = 75;

    static const int FRAME_HEADER_LENGTH = 4;
    static const int FRAME_END_LENGTH = 4;
    static const uint32_t CONFIG_FRAME_HEADER = 0xFAFBFCFD;
    static const uint32_t CONFIG_FRAME_END = 0x01020304;
    static const uint32_t TARGET_FRAME_HEADER = 0xF1F2F3F4;
    static const uint32_t TARGET_FRAME_END = 0xF5F6F7F8;


    static constexpr char *const TAG = (char*) "ld2410";

    ConfigSwitch *show_stats = new ConfigSwitch( [this](bool b){this->setShowStats(b);});
    ConfigSwitch *show_engineering_stats = new ConfigSwitch([this](bool b){this->setEngineeringMode(b);});

    Sensor *motionTargetDistance = new Sensor();
    Sensor *motionTargetEnergy = new Sensor();
    Sensor *staticTargetDistance = new Sensor();
    Sensor *staticTargetEnergy = new Sensor();
    Sensor *detectDistance = new Sensor();

    esphome::template_::TemplateNumber *staticGateSensitivity[NUMBER_OF_GATES];
    esphome::template_::TemplateNumber *motionGateSensitivity[NUMBER_OF_GATES];

    // Having a sensor per gate energy is too much so use one text sensor each
    // to publish all as string representation.
    TextInfoSensor *staticGateEnergy = new TextInfoSensor();
    TextInfoSensor *motionGateEnergy = new TextInfoSensor();

    Sensor *allSensitivity = new Sensor();
    TextSensor *version_sensor = new VersionSensor();

    enum COMMAND : uint16_t {
        CONFIG_START = 0x00FF,
        CONFIG_END = 0x00FE,
        SET_DISTANCE_AND_DURATION = 0x0060,
        READ_PARAMETER = 0x0061,
        ENGINEERING_START = 0x0062,
        ENGINEERING_END = 0x0063,
        SET_GATE_SENSITIVITY = 0x0064,
        READ_FIRMWARE_VERSION = 0x00A0,
        SET_BAUD_RATE = 0x00A1,
        FACTORY_RESET = 0x00A2,
        RESTART = 0x00A3,
        SET_BLUETOOTH = 0x00A4,
        READ_MAC_ADDRESS = 0x00A5,

        RESPONSE = 0x0100
    };

    enum FRAME_STATE {
        HEADER,
        CONFIG_FRAME_DATA,
        TARGET_FRAME_DATA
    } frameState = HEADER;

    const std::vector <uint8_t> ld2410_end_conf = {0x04, 0x03, 0x02, 0x01};

    int dataBufferPos = 0;
    uint8_t dataBuffer[DATA_BUFFER_MAX_SIZE];
    typedef struct __attribute__((packed)) {
        uint32_t header;
        uint16_t length;
    } FrameStart;
    static const int FRAME_START_LENGTH = sizeof(FrameStart);
    int expectedDataAmount = FRAME_START_LENGTH;

    typedef struct __attribute__((packed)) {
        uint16_t command;  // 0xA0 0x01
        uint16_t ackSate;  // 0 = success; 1 = failure
        uint16_t firmwareType;  // 0 = always?
        uint8_t majorVersion2;  // 2nd digit of the mayor version
        uint8_t majorVersion1;  // 1st digit of the mayor version
        uint32_t minorVersion;  // minor version number
    } FirmwareVersionFrame;
    typedef struct __attribute__((packed)) {
        uint16_t command;  // 0x61   0x01
        uint16_t ackSate;  // 0 = success; 1 = failure
        uint8_t head;        // 0xAA Always?
        uint8_t maxGate; // counting from 0 with 9 gate we end at 8
        uint8_t maxMotionGate; // counting from 0 with 9 gate we end at 8 / used gates!?
        uint8_t maxStaticGate; // counting from 0 with 9 gate we end at 8 / used gates!?
        uint8_t motionGateSensitivity[NUMBER_OF_GATES]; // actually NUMBER_OF_GATES
        uint8_t staticGateSensitivity[NUMBER_OF_GATES]; // actually NUMBER_OF_GATES
        uint16_t timeDuration; // unmanned duration!??
    } ConfigurationParameterFrame;

    typedef enum : uint8_t { // bitfield?
        NO_TARGET = 0,
        MOTION = 1,
        STATIC = 2,
        MOTION_AND_STATIC = 3
    } TargetState;

    typedef struct __attribute__((packed)) {
        uint8_t type;                 // 0x01 == engineering / 0x02 == target
        uint8_t head;                // 0xAA = fixed head
        TargetState state;
        uint16_t motionTargetDistanceCm;
        uint8_t motionTargetEnergy;
        uint16_t staticTargetDistanceCm;
        uint8_t staticTargetEnergy;
        uint16_t detectionDistance;

        uint8_t maximumMotionGate;  // 8 == INDEX_OF_LAST_GATE always!?
        uint8_t maximumStaticGate;  // 8 == INDEX_OF_LAST_GATE always!?
        uint8_t motionGateEnergy[NUMBER_OF_GATES];
        uint8_t staticGateEnergy[NUMBER_OF_GATES];
        // uint8_t additionalInformation[16]; // M bytes "Retain data, store additional information"
        // ???
        // uint8_t end;                // 0x55 always
        // uint8_t check;              // 0x00 always
    } TargetDataEngineeringFrame;

    LD2410(UARTComponent *parent) : UARTDevice(parent) {
        show_stats->set_disabled_by_default(true);

        motionTargetDistance->set_unit_of_measurement("cm");
        motionTargetDistance->set_accuracy_decimals(0);
        motionTargetEnergy->set_unit_of_measurement("%");
        motionTargetEnergy->set_accuracy_decimals(0);

        staticTargetDistance->set_unit_of_measurement("cm");
        staticTargetDistance->set_accuracy_decimals(0);
        staticTargetEnergy->set_unit_of_measurement("%");
        staticTargetEnergy->set_accuracy_decimals(0);

        detectDistance->set_unit_of_measurement("cm");
        detectDistance->set_accuracy_decimals(0);

        motionGateEnergy->set_internal(true);
        staticGateEnergy->set_internal(true);

//        maxConfigDistance->set_unit_of_measurement("cm");
//        maxConfigDistance->set_min_value(0);
//        maxConfigDistance->set_max_value(600);
//        maxConfigDistance->set_step(75);
    }

    void setup() override {
        Component::setup();
        // not sure when external sensors are initialized, here they are valid!
        motionGateSensitivity[0] = motionGate0Sensitivity;
        motionGateSensitivity[1] = motionGate1Sensitivity;
        motionGateSensitivity[2] = motionGate2Sensitivity;
        motionGateSensitivity[3] = motionGate3Sensitivity;
        motionGateSensitivity[4] = motionGate4Sensitivity;
        motionGateSensitivity[5] = motionGate5Sensitivity;
        motionGateSensitivity[6] = motionGate6Sensitivity;
        motionGateSensitivity[7] = motionGate7Sensitivity;
        motionGateSensitivity[8] = motionGate8Sensitivity;
        staticGateSensitivity[0] = nullptr; //  staticGate0Sensitivity, // can not be used
        staticGateSensitivity[1] = nullptr; //  staticGate0Sensitivity, // can not be used
        staticGateSensitivity[2] = staticGate2Sensitivity;
        staticGateSensitivity[3] = staticGate3Sensitivity;
        staticGateSensitivity[4] = staticGate4Sensitivity;
        staticGateSensitivity[5] = staticGate5Sensitivity;
        staticGateSensitivity[6] = staticGate6Sensitivity;
        staticGateSensitivity[7] = staticGate7Sensitivity;
        staticGateSensitivity[8] = staticGate8Sensitivity;
        set_timeout("setup-query", 1000, [this](){this->queryAll();});
    }

    void dump_config() override {
        ESP_LOGCONFIG(TAG, "ld2410:");
        check_uart_settings(256000, 1, UARTParityOptions::UART_CONFIG_PARITY_NONE, 8);
        if (version_sensor != nullptr) {
            ESP_LOGCONFIG(TAG, "  firmware: %s", version_sensor->state.c_str());
        }
        ESP_LOGCONFIG(TAG, "  Show Stats: %s", YESNO(this->show_stats->state));
        ESP_LOGCONFIG(TAG, "  Show Engineering Stats: %s", YESNO(this->show_engineering_stats->state));
    }

    void loop() override {
        auto start = millis();
        handle();
        auto end = millis();
        auto msNeeded = end - start;
        if (msNeeded > 50) {
            ESP_LOGW(TAG, "Loop took %dms! :(", msNeeded);
        }
    }

    void ESP_LOGD_HEX() {
#ifdef USE_ESP32
        std::string res;
        char buf[256];
        sprintf(buf, "Size: %d Expected %d ", dataBufferPos, expectedDataAmount);
        res += buf;
        for (size_t i = 0; i < dataBufferPos; i++) {
            if (i > 0) {
                res += ":";
            }
            sprintf(buf, "%02X", dataBuffer[i]);
            res += buf;
        }
        ESP_LOGD(TAG, res.c_str());
#endif
    }

    void ESP_LOGD_HEX(std::vector <uint8_t> bytes, uint8_t separator) {
#ifdef USE_ESP32
        std::string res;
        size_t len = bytes.size();
        char buf[5];
        for (size_t i = 0; i < len; i++) {
            if (i > 0) {
                res += separator;
            }
            sprintf(buf, "%02X", bytes[i]);
            res += buf;
        }
        ESP_LOGD(TAG, res.c_str());
#endif
    }

    bool sendCommand(uint16_t cmd, char *commandValue = nullptr, int commandValueLen = 0) {
        ESP_LOGI(TAG, ">>> %s (0x%04X)", commandWordAsString(cmd).c_str(), cmd);

        uint16_t len = 2;
        if (commandValue != nullptr) {
            len += commandValueLen;
        }
        std::vector <uint8_t> ld2410_conf
                = {0xFD, 0xFC, 0xFB, 0xFA, lowByte(len), highByte(len), lowByte(cmd), highByte(cmd)};
        if (commandValue != nullptr) {
            for (int i = 0; i < commandValueLen; i++) {
                ld2410_conf.push_back(commandValue[i]);
            }
        }
        for (int i = 0; i < FRAME_HEADER_LENGTH; i++) {
            ld2410_conf.push_back(ld2410_end_conf[i]);
        }
        // ESP_LOGD_HEX(ld2410_conf, '>');
        write_array(std::vector<uint8_t>(ld2410_conf.begin(), ld2410_conf.end()));

        return handle(100, cmd | RESPONSE);
    }

    static int twoByteToInt(char firstByte, char secondByte) {
        return (int16_t)(secondByte << 8) + firstByte;
    }

    static std::string targetStateAsString(TargetState state) {
        std::string result;
        switch (state) {
            case TargetState::NO_TARGET:         result += "--";  break;
            case TargetState::MOTION:            result += "M-";  break;
            case TargetState::STATIC:            result += "-S";  break;
            case TargetState::MOTION_AND_STATIC: result += "MS";  break;
            default:
                char buf[5];
                sprintf(buf, "%02X", state);
                result += "Unknown 0x";
                result += buf;
        }
        return result;
    }

    static std::string commandWordAsString(uint16_t commandWord) {
        std::string result;
        if ((commandWord & COMMAND::RESPONSE) == COMMAND::RESPONSE) {
            result += "ACK ";
            commandWord &= ~COMMAND::RESPONSE;
        }
        switch (commandWord) {
            case CONFIG_START:              result += "start Config";               break;
            case CONFIG_END:                result += "end Config";                 break;
            case SET_DISTANCE_AND_DURATION: result += "Set Range and Duration";     break;
            case READ_PARAMETER:            result += "Read Parameter";             break;
            case ENGINEERING_START:         result += "start engineering mode";     break;
            case ENGINEERING_END:           result += "end engineering mode";       break;
            case SET_GATE_SENSITIVITY:      result += "Rage Gate Config";           break;
            case READ_FIRMWARE_VERSION:     result += "Read Firmware Version";      break;
            case SET_BAUD_RATE:             result += "Set serial port baud rate";  break;
            case FACTORY_RESET:             result += "factory config reset";       break;
            case RESTART:                   result += "Restart module";             break;
            case SET_BLUETOOTH:             result += "Bluetooth setting";          break;
            case READ_MAC_ADDRESS:          result += "read mac address";           break;
            default:
                char buf[6];
                sprintf(buf, "%04X", commandWord);
                result += "Unknown cmd 0X";
                result += buf;
        }
        return result;
    }

    int statusMessageCount = 0;
    LD2410::TargetState lastReportedTargetState = LD2410::TargetState::NO_TARGET;

    void handleTargetData() {
        TargetDataEngineeringFrame *msg = (TargetDataEngineeringFrame *) dataBuffer;

        if (msg->type == 0x01) {
            publishIfChanged(show_engineering_stats, true);
        } else if (msg->type == 0x02) {
            publishIfChanged(show_engineering_stats, false);
        }

        statusMessageCount++;
        if (statusMessageCount % 50 == 0 || lastReportedTargetState != msg->state) {
            lastReportedTargetState = msg->state;
            statusMessageCount = 0;
            char buf[1024];
            if (msg->type == 0x02 || msg->type == 0x01) {
                std::string gateEnergy;
                sprintf(buf, "State: %s %3dcm  Motion: %3dcm (%3d%%), Static: %3dcm (%3d%%)",
                        targetStateAsString(msg->state).c_str(),
                        msg->detectionDistance,
                        msg->motionTargetDistanceCm,
                        msg->motionTargetEnergy,
                        msg->staticTargetDistanceCm,
                        msg->staticTargetEnergy);
                gateEnergy += buf;
                if (msg->type == 0x01) {
                    sprintf(buf, " ENG max (motion %d/ static %d) ", msg->maximumMotionGate, msg->maximumStaticGate);
                    gateEnergy += buf;
                    for (int gateIndex = 0; gateIndex < NUMBER_OF_GATES; gateIndex++) {
                        sprintf(buf, "%dcm:%3d/%3d ",
                                GATE_SIZE_CM * gateIndex,
                                msg->motionGateEnergy[gateIndex], msg->staticGateEnergy[gateIndex]);
                        gateEnergy += buf;
                    }
                    // bytes after the known data
                    if (dataBufferPos != 39 /* 39 is expected length 00 55 + END*/) {
                        for (int pos = 33; pos < dataBufferPos; pos++) {
                            sprintf(buf, "%02X:", dataBuffer[pos]);
                            gateEnergy += buf;
                        }
                    }
                }
                ESP_LOGD(TAG, "%s", gateEnergy.c_str());
            } else {
                ESP_LOGW(TAG, "Unknown target data type: %02x", msg->type);
            }
        }

        if (msg->type == 0x01) {
            std::string motionGate;
            std::string staticGate;
            char buf[128];
            for (int i = 0; i < NUMBER_OF_GATES; i++) {
                if (i > 0) {
                    motionGate += "|";
                    staticGate += "|";
                }
                sprintf(buf, "%3d", msg->motionGateEnergy[i]);
                motionGate += buf;
                sprintf(buf, "%3d", msg->staticGateEnergy[i]);
                staticGate += buf;
            }
            publishIfChanged(motionGateEnergy, motionGate);
            publishIfChanged(staticGateEnergy, staticGate);
        }

        if (show_stats->state && (msg->type == 0x02 || msg->type == 0x01)) {
            publishIfChanged(motionTargetDistance, msg->motionTargetDistanceCm);
            publishIfChanged(motionTargetEnergy, msg->motionTargetEnergy);
            publishIfChanged(staticTargetDistance, msg->staticTargetDistanceCm);
            publishIfChanged(staticTargetEnergy, msg->staticTargetEnergy);
            publishIfChanged(detectDistance, msg->detectionDistance);
        }
    }

    uint16_t lastCommandReceived = 0;

    void handleConfData() {
        int cmdWord = twoByteToInt(dataBuffer[0], dataBuffer[1]);
        lastCommandReceived = cmdWord;
        ESP_LOGD(TAG, "<<< %s (0x%04X)", commandWordAsString(cmdWord).c_str(), cmdWord);
//        ESP_LOGD_HEX();
        switch (cmdWord) {
            case ENGINEERING_START | RESPONSE: { // Engineering mode start ACK
                id(show_engineering_stats).publish_state(true);
                break;
            }
            case ENGINEERING_END | RESPONSE: { // Engineering mode end ACK
                id(show_engineering_stats).publish_state(false);
                id(staticGateEnergy).publish_state("-");
                id(motionGateEnergy).publish_state("-");
                break;
            }
            case READ_FIRMWARE_VERSION | RESPONSE: { // Firmware Version ACK
                FirmwareVersionFrame *msg = (FirmwareVersionFrame *) dataBuffer;
                char buf[16];
                sprintf(buf, "V%d.%02x.%08x", msg->majorVersion1, msg->majorVersion2, msg->minorVersion);
                std::string res;
                res += buf;
                ESP_LOGI("conf", "Firmware Version is %s", buf);
                id(version_sensor).publish_state(res);
                break;
            }
            case READ_PARAMETER | RESPONSE: { // Read Parameter ACK
                ConfigurationParameterFrame *msg = (ConfigurationParameterFrame *) dataBuffer;
                publishIfChanged(maxConfigDistance, GATE_SIZE_CM * msg->maxStaticGate);
                publishIfChanged(allSensitivity, msg->motionGateSensitivity[0]);
                publishIfChanged(noneDuration, msg->timeDuration);
                char buf[256];
                sprintf(buf, " Configuration gates %d/%d/%d ", msg->maxGate, msg->maxMotionGate, msg->maxStaticGate);
                std::string parameters;
                parameters += buf;
                for (int gateIndex = 0; gateIndex < NUMBER_OF_GATES; gateIndex++) {
                    sprintf(buf, "%dcm:%3d/%3d ",
                            GATE_SIZE_CM * gateIndex,
                            msg->motionGateSensitivity[gateIndex], msg->staticGateSensitivity[gateIndex]);
                    parameters += buf;

                    publishIfChanged(motionGateSensitivity[gateIndex], msg->motionGateSensitivity[gateIndex]);
                    publishIfChanged(staticGateSensitivity[gateIndex], msg->staticGateSensitivity[gateIndex]);
                }
                sprintf(buf, "no time %ds", msg->timeDuration);
                parameters += buf;
                ESP_LOGI(TAG, "Parameters: %s", parameters.c_str());
                break;
            }
            default:; // ESP_LOGI("conf", "Unhandled command 0x%04X %s", cmdWord, commandWordAsString(cmdWord).c_str());
        }
    }

    static void publishIfChanged(Sensor *s, int value) {
        if (s && id(s).state != value) {
            id(s).publish_state(value);
        }
    }

    static void publishIfChanged(esphome::template_::TemplateNumber *s, int value) {
        if (s && id(s).state != value) {
            id(s).publish_state(value);
        }
    }

    static void publishIfChanged(ConfigSwitch *s, bool value) {
        if (s && id(s).state != value) {
            id(s).publish_state(value);
        }
    }

    static void publishIfChanged(TextInfoSensor *s, std::string value) {
        if (s && id(s).state != value) {
            id(s).publish_state(value);
        }
    }

    void setMotionSensitivity(int gate, int motionSensitivity) {
        ESP_LOGV(TAG, "setMotionSensitivity %d %d", gate, motionSensitivity);
        setConfigMode(true);
        int staticSensitivity = motionSensitivity;
        if (staticGateSensitivity[gate]) {
            staticSensitivity = staticGateSensitivity[gate]->state;
        } else {
            ESP_LOGD(TAG, "No Static Gate Value found - using motion gate");
        }
        doSetGateSensitivity(gate, motionSensitivity, staticSensitivity);
        queryParameters();
        setConfigMode(false);
    }

    void setStaticSensitivity(int gate, int sensitivity) {
        ESP_LOGV(TAG, "setStaticSensitivity %d %d", gate, sensitivity);
        setConfigMode(true);
        doSetGateSensitivity(gate, motionGateSensitivity[gate]->state, sensitivity);
        queryParameters();
        setConfigMode(false);
    }

    void doSetGateSensitivity(int gate, int motionSensitivity, int staticSensitivity) {
        // setting not all values does not work, so we always set motion and static
        ESP_LOGI(TAG, "doSetGateSensitivity Gate: %d Motion: %d Static: %d",
                 gate, motionSensitivity, staticSensitivity);
        char data[18] = {
                /* Select Gate */0x00, 0x00,
                                 (char) gate, 0x00, 0x00, 0x00,
                /* Select Motion*/ 0x01, 0x00,
                                 (char) motionSensitivity, 0x00, 0x00,0x00,
                /* Select Static*/ 0x02, 0x00,
                                 (char) staticSensitivity, 0x00, 0x00,0x00};
        sendCommand(COMMAND::SET_GATE_SENSITIVITY, data, sizeof(data));
    }

    void queryAll() {
        setConfigMode(true);
        queryParameters();
        queryFirmwareVersion();
        setConfigMode(false);
    }

    void setShowStats(bool stats) {
        id(show_stats).publish_state(stats);
        id(motionTargetDistance).publish_state(NAN);
        id(motionTargetEnergy).publish_state(NAN);
        id(staticTargetDistance).publish_state(NAN);
        id(staticTargetEnergy).publish_state(NAN);
        id(detectDistance).publish_state(NAN);
    }

    void setConfigMode(bool confenable) {
        char value[2] = {0x01, 0x00};
        sendCommand(
                confenable ? COMMAND::CONFIG_START : COMMAND::CONFIG_END,
                confenable ? value : nullptr, 2);
    }

    void queryParameters() {
        sendCommand(COMMAND::READ_PARAMETER);
    }

    void queryFirmwareVersion() {
        sendCommand(COMMAND::READ_FIRMWARE_VERSION);
    }

    void consumeSerial() {
        switch (frameState) {
            case HEADER: { // waiting for the header
                FrameStart *start = (FrameStart *) dataBuffer;
                if (TARGET_FRAME_HEADER == start->header) {
                    frameState = FRAME_STATE::TARGET_FRAME_DATA;
                    expectedDataAmount = start->length + FRAME_END_LENGTH;
                    dataBufferPos = 0;
                } else if (CONFIG_FRAME_HEADER == start->header) {
                    frameState = FRAME_STATE::CONFIG_FRAME_DATA;
                    expectedDataAmount = start->length + FRAME_END_LENGTH;
                    dataBufferPos = 0;
                } else {
                    // FIXME: We should not log this so better count it!
                    // ESP_LOGW(TAG, "FAILED HEADER %08X - %d bytes in buffer", start->header, dataBufferPos);
                    // we need to search for the header :( byte by byte
                    std::memmove(dataBuffer, dataBuffer + 1, dataBufferPos - 1);
                    expectedDataAmount = 1; // need ro read 1 byte after the other
                    dataBufferPos -= 1;
                }
                break;
            }
            case CONFIG_FRAME_DATA: {
                handleConfData();
                frameState = FRAME_STATE::HEADER;
                dataBufferPos = 0;
                expectedDataAmount = FRAME_START_LENGTH;
                break;
            }
            case TARGET_FRAME_DATA: {
                handleTargetData();
                frameState = FRAME_STATE::HEADER;
                dataBufferPos = 0;
                expectedDataAmount = FRAME_START_LENGTH;
                break;
            }
            default:
                // SOME ERROR HANDLING
                break;
        }
        if (expectedDataAmount + dataBufferPos > DATA_BUFFER_MAX_SIZE) {
            ESP_LOGW(TAG, "Requested data amount %d larger than buffer %d, will reset buffer",
                     expectedDataAmount, DATA_BUFFER_MAX_SIZE);
            frameState = FRAME_STATE::HEADER;
            dataBufferPos = 0;
            expectedDataAmount = FRAME_START_LENGTH;
        }
    }

    void handle() {
        int av = available();
        if (av > 128) { // default buffer on ESP32 is 256 bytes
            ESP_LOGW(TAG, "More than expected bytes in buffer. %d - we are to slow! expecting %d",
                     av, expectedDataAmount);
        }
        while (expectedDataAmount && av >= expectedDataAmount
               && read_array(&(dataBuffer[dataBufferPos]), expectedDataAmount)) {
            dataBufferPos += expectedDataAmount;
            consumeSerial();
            av = available();
        }
    }

    /* Will wait for the given number of ms and handle serial data if needed. */
    bool handle(uint32_t milliSeconds, uint16_t cmd) {
        lastCommandReceived = 0;
        const auto start = millis();
        const auto end = start + milliSeconds;
        while (end > millis() && lastCommandReceived != cmd) {
            handle();
            if (lastCommandReceived != cmd) {
                delay(1);
            }
        }
        bool done = lastCommandReceived == cmd;
        if (done) {
            ESP_LOGD(TAG, "Received %s (0x%04X) after %dms",
                     commandWordAsString(cmd).c_str(), cmd, millis() - start);
        } else {
            ESP_LOGE(TAG, "NOT received %04X != %04X after %dms", cmd, lastCommandReceived, millis() - start);
        }
        return done;
    }

    void setEngineeringMode(bool engenable) {
        setConfigMode(true);
        sendCommand(engenable ? COMMAND::ENGINEERING_START : COMMAND::ENGINEERING_END);
        setConfigMode(false);
    }

    void setMaxDistancesAndNoneDuration(int maxMotionDistanceRange, int maxStaticDistanceRange, int noneDuration) {
        setConfigMode(true);
        doSetMaxDistancesAndNoneDuration(maxMotionDistanceRange, maxStaticDistanceRange, noneDuration);
        queryParameters();
        setConfigMode(false);
    }

    void doSetMaxDistancesAndNoneDuration(int maxMotionDistanceRange, int maxStaticDistanceRange, int noneDuration) {
        // we have to set the gate - derived from the samples in the spec so:
        uint8_t maxMotionGate = maxMotionDistanceRange / GATE_SIZE_CM;
        uint8_t maxStaticGate = maxStaticDistanceRange / GATE_SIZE_CM;
        char value[18] = {0x00, 0x00, maxMotionGate, 0x00, 0x00, 0x00,
                          0x01, 0x00, maxStaticGate, 0x00, 0x00, 0x00,
                          0x02, 0x00, lowByte(noneDuration), highByte(noneDuration), 0x00, 0x00};
        sendCommand(COMMAND::SET_DISTANCE_AND_DURATION, value, sizeof(value));
    }

    void factoryReset() {
        sendCommand(COMMAND::FACTORY_RESET);
    }

    void reboot() {
        sendCommand(COMMAND::RESTART);
        // not need to exit config mode because the ld2410 will reboot automatically
    }

    void setBaudrate(uint8_t index) {
        // this is not the baud rate to set but
        // Baud rate selection index value
        // 0x0001 9600
        // 0x0002 19200
        // 0x0003 38400
        // 0x0004 57600
        // 0x0005 115200
        // 0x0006 230400
        // 0x0007 256000 (default, what we use)
        // 0x0008 460800
        char value[2] = {index, 0x00};
        sendCommand(COMMAND::SET_BAUD_RATE, value, sizeof(value));
    }
};
