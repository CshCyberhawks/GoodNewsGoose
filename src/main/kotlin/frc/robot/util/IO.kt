package frc.robot.util

import edu.wpi.first.wpilibj.Joystick

object IO {
    private val rightJoy = Joystick(0)
    private val leftJoy = Joystick(1)

    val moveX
        get() = rightJoy.x

    val moveY
        get() = rightJoy.y

    val moveTwist
        get() = leftJoy.x
}