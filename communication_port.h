#pragma once

#include <QSerialPort>

namespace robotiq {

class CommunicationPort
{
public:
    enum class CommandTopic {
        Activation,
        Move,
        RequestedPosition,
        Speed,
        Force
    };

    enum class QueryTopic {
        Activation
    };

    enum class WriteRegisterAddress {
        ActionRequest   = 0x00,
        PositionRequest = 0x03,
        Speed           = 0x04,
        Force           = 0x05
    };

    enum class ActionRequestBitField {
        Activation                  = 0x00,
        GoTo                        = 0x03,
        EmergencyRelease            = 0x04,
        EmergencyReleaseDirection   = 0x05
    };

    enum class ReadRegisterAddress {
        GripperStatus       = 0x00,
        FaultStatus         = 0x02,
        RequestedPosition   = 0x03,
        Position            = 0x04,
        Current             = 0x05
    };

    enum class GripperStatusBitField {
        Activation              = 0x01,
        GoTo                    = 0x03,
        GripperStatusLowBit     = 0x04,
        GripperStatusHighBit    = 0x05,
        ObjectDetectionLowBit   = 0x06,
        ObjectDetectionHighBit  = 0x07
    };

    CommunicationPort(QString portName);

    void sendCommand(CommandTopic property, quint8 value);
    quint8 query(QueryTopic topic);

private:
    static constexpr int numberOfWriteRegisters {8};
    static constexpr quint8 slaveId {0x09};
    static constexpr quint8 writeRegisterCode {0x10};
    static constexpr quint8 readRegisterCode {0x03};
    static constexpr quint16 writeRegistersStart {0x03 << 8 | 0xE8};
    static constexpr quint16 readRegistersStart {0x07 << 8 | 0xD0};

    void sendWriteCommand(quint8 index, quint16 numberOfRegisters);
    QByteArray sendReadCommand(quint8 index, quint16 numberOfRegisters);

    QSerialPort m_port;
    QByteArray m_writeRegisters;
};

}


