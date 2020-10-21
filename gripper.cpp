#include "gripper.h"

using namespace robotiq;

Gripper::Gripper(const QString &portName) :
    m_port(portName)
{

}

void Gripper::init()
{
    deactivate();
    activate();
}

void Gripper::activate(bool value)
{
    m_port.sendCommand(CommunicationPort::CommandTopic::Activation, value);

    if (value) {
        while(!isActivated()) {}
    }
}

bool Gripper::isActivated()
{
    return m_port.query(CommunicationPort::QueryTopic::Activation);
}

void Gripper::emergencyRelease()
{
    m_emergencyRelease = true;
// Activate emergency release
}

void Gripper::move(quint8 position)
{
    setRequestedPosition(position);
    move();
}

void Gripper::setMoving(bool value)
{
    m_port.sendCommand(CommunicationPort::CommandTopic::Move, value);
    m_moving = value;
}

void Gripper::setRequestedPosition(quint8 position)
{
    m_port.sendCommand(CommunicationPort::CommandTopic::RequestedPosition, position);
    m_requestedPosition = position;
}

void Gripper::setSpeed(quint8 speed)
{
    m_port.sendCommand(CommunicationPort::CommandTopic::Speed, speed);
    m_speed = speed;
}

void Gripper::setForce(quint8 force)
{
    m_port.sendCommand(CommunicationPort::CommandTopic::Force, force);
    m_force = force;
}
