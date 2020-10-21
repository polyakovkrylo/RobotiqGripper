#pragma once

#include "communication_port.h"

#include <limits>

namespace robotiq {

/*!
 * \brief The Gripper driver class
 *
 * Provides functionality to control Robotiq Hand-E Gripper
 */
class Gripper : QObject
{
    Q_OBJECT
public:
    /*!
     * \brief Moving direction of the gripper
     */
    enum class MovingDirection {
        Opening,
        Closing
    };

    /*!
     * \brief Position of the gripper
     */
    enum class Position {
        FullyOpen = 0,
        FullyClosed = std::numeric_limits<quint8>::max()
    };

    Gripper(const QString &portName);

    /*!
     * \brief Initializion of the gripper
     *
     * The function sends a set of commands needed to initialize the gripper
     *
     * \warning This is a blocking method, the program is blocked until
     * the gripper is initialized
     *
     * \sa activate(bool)
     */
    void init();

    /*!
     * \brief Activate the gripper
     * \sa activate(bool)
     */
    inline void activate() {activate(true);}

    /*!
     * \brief Deactivate the gripper
     * \sa activate(bool)
     */
    inline void deactivate() {activate(false);}

    /*!
     * \brief Activation function
     *
     * Activates the gripper and waits till the activation
     * is completed
     *
     * \param value activation/deactivation flag
     *
     * \warning this is a blocking function, the program is blocked
     * until activation is completed
     */
    void activate(bool value);

    /*!
     * \brief Query activation status of the gripper
     * \return Activation status
     */
    bool isActivated();

    /*!
     * \brief Launch emergency release
     * \warning The gripper should be reinitialized after emergency release
     */
    void emergencyRelease();

    /*!
     * \brief Move to the fully open position
     * \sa move(quint8)
     */
    inline void open() {move(static_cast<quint8>(Position::FullyOpen));}

    /*!
     * \brief Move to the fully closed position
     */
    inline void close(){move(static_cast<quint8>(Position::FullyClosed));}

    /*!
     * \brief Start moving to the requested positon
     * \sa setMoving(bool)
     * \sa setRequestedPosition(quint8)
     */
    inline void move() {setMoving(true);}

    /*!
     * \brief Stop moving
     * \sa setMoving(bool)
     */
    inline void stop() {setMoving(false);}

    /*!
     * \brief Move to the position
     * \param position requested position
     * \sa setMoving(bool)
     * \sa setRequestedPosition(quint8)
     */
    void move(quint8 position);

    /*!
     * \brief Start/stop moving to the requested position
     * \param value Moving flag
     * \sa setRequestedPosition(quint8)
     */
    void setMoving(bool value);

    /*!
     * \brief Set the desired position without moving to it
     * \param position Requested position
     * \sa move(quint8)
     */
    void setRequestedPosition(quint8 position);

    /*!
     * \brief Set speed
     * \param speed
     */
    void setSpeed(quint8 speed);

    /*!
     * \brief Set force limit
     * \param force Maximum force
     */
    void setForce(quint8 force);

    /*!
     * \brief Is the gripper set to moving
     * \return moving allowed
     */
    inline bool isMoving() {return m_moving;}

    /*!
     * \brief Is emengency release activated
     * \return emergency release
     */
    inline bool isEmengencyRelease(){return m_emergencyRelease;}

    /*!
     * \brief Emergency release direction
     *
     * In case of emergency release, the gripper will move in this direction
     *
     * \return direction
     */
    inline MovingDirection emergencyReleaseDirection() {return m_emergencyReleaseDirection;}

    /*!
     * \brief Current requested position
     * \note Requested position might differ from the actual position
     * \return Requested position
     */
    inline quint8 requestedPosition() const {return m_requestedPosition;}

    /*!
     * \brief Requested speed
     * \sa setSpeed(quint8)
     * \return Maximum allowed speed
     */
    inline quint8 speed() const {return m_speed;}

    /*!
     * \brief Maximum allowed force
     * \return Maximum allowed force
     */
    inline quint8 force() const {return m_force;}

private:
    CommunicationPort m_port;

    bool m_moving {false};
    bool m_emergencyRelease {false};
    MovingDirection m_emergencyReleaseDirection {MovingDirection::Opening};
    quint8 m_requestedPosition {static_cast<quint8>(Position::FullyOpen)};
    quint8 m_speed {std::numeric_limits<quint8>::max()};
    quint8 m_force {std::numeric_limits<quint8>::max()};
};

}

