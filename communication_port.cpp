#include "communication_port.h"

#include "modbus_crc.h"

#include <QDataStream>

using namespace robotiq;

inline void setBit(QByteRef byte, int pos, bool value)
{
    byte = static_cast<char>(byte | (value << pos));
}

CommunicationPort::CommunicationPort(QString portName) :
    m_port(portName)
{
    m_port.setBaudRate(QSerialPort::Baud115200);

    // Each register is sent as uint16,
    // although marked as uint8 in the datasheet
    m_writeRegisters.resize(numberOfWriteRegisters * 2);
}

void CommunicationPort::sendCommand(CommunicationPort::CommandTopic property, quint8 value)
{
    quint8 numberOfRegisters = 0;
    quint8 address = 0;
    int bitField = -1;

    switch (property) {
    case CommandTopic::Activation:
        numberOfRegisters = 3;
        address = static_cast<int>(WriteRegisterAddress::ActionRequest);
        bitField = static_cast<int>(ActionRequestBitField::Activation);
        break;

    case CommandTopic::Move:
        numberOfRegisters = 1;
        address = static_cast<int>(WriteRegisterAddress::ActionRequest);
        bitField = static_cast<int>(ActionRequestBitField::GoTo);
        break;

    case CommandTopic::RequestedPosition:
        numberOfRegisters = 1;
        address = static_cast<int>(WriteRegisterAddress::PositionRequest);
        break;

    case CommandTopic::Speed:
        numberOfRegisters = 1;
        address = static_cast<int>(WriteRegisterAddress::Speed);
        break;
    case CommandTopic::Force:
        numberOfRegisters = 1;
        address = static_cast<int>(WriteRegisterAddress::Force);
        break;

    }

    if (bitField != -1) {
        setBit(m_writeRegisters[address], bitField, value);
    } else {
        m_writeRegisters[address] = static_cast<char>(value);
    }

    sendWriteCommand(address, numberOfRegisters);
}

quint8 CommunicationPort::query(CommunicationPort::QueryTopic topic)
{
    quint8 address= 0;
    quint8 numberOfRegisters = 0;
    quint8 mask = std::numeric_limits<quint8>::max();

    switch (topic) {
    case QueryTopic::Activation:
        address = 0;
        numberOfRegisters = 1;
        mask = static_cast<quint8>(GripperStatusBitField::Activation) |
                static_cast<quint8>(GripperStatusBitField::GripperStatusLowBit) |
                static_cast<quint8>(GripperStatusBitField::GripperStatusHighBit);
        break;
    }

    QByteArray response = sendReadCommand(address, numberOfRegisters);
    return response.front() & mask;
}

void CommunicationPort::sendWriteCommand(quint8 index, quint16 numberOfRegisters)
{
    QByteArray arr;
    QByteArray data = m_writeRegisters.mid(index, numberOfRegisters * sizeof(quint16));
    QDataStream out(&arr, QIODevice::WriteOnly);

    out << slaveId
        << writeRegisterCode
        << static_cast<quint16>(writeRegistersStart + index)
        << numberOfRegisters
        << static_cast<quint8>(data.size());

    out.writeRawData(data.data(), data.size());

    out << сhecksum(arr.data(), static_cast<unsigned short>(arr.size()));

    m_port.write(arr);

    // Expected response:
    // slaveId + functionCode + addressOfFirstRegister + numberOfWrittenRegisters + CRC16
    qint64 bytesExpected = 8;
    int timeout = 5000;

    QByteArray response;

    while(m_port.waitForReadyRead(timeout)) {
        if (m_port.bytesAvailable() >= bytesExpected) {
            response = m_port.read(bytesExpected);
        }
    }
}

QByteArray CommunicationPort::sendReadCommand(quint8 index, quint16 numberOfRegisters)
{
    m_port.flush();

    QByteArray arr;
    QDataStream out(&arr, QIODevice::WriteOnly);

    out << slaveId
        << readRegisterCode
        << static_cast<quint16>(readRegistersStart + index)
        << numberOfRegisters
        << сhecksum(arr.data(), static_cast<unsigned short>(arr.size()));

    m_port.write(arr);

    // Expected response:
    // slaveId + functionCode + numberOfBytes + data(numberOfRegisters*2) + CRC16
    int headerSize = 3;
    int crcSize = sizeof(quint16);
    qint64 bytesExpected = headerSize + (numberOfRegisters * 2) + crcSize;
    int timeout = 5000;

    QByteArray response;

    while(m_port.waitForReadyRead(timeout)) {
        if (m_port.bytesAvailable() >= bytesExpected) {
            response = m_port.read(bytesExpected);
        }
    }

    return response.mid(headerSize, numberOfRegisters*2);
}
