package frc.robot.util

import edu.wpi.first.wpilibj.Joystick

object IO {
    private val rightJoy = Joystick(0)
    private val leftJoy = Joystick(1)

    private const val hosas = true

    val quickThrottle
        get() = leftJoy.pov

    val normalThrottle
        get() = leftJoy.getRawButtonPressed(3)

    val fastThrottle
        get() = leftJoy.getRawButtonPressed(4)

    val gyroReset
        get() = rightJoy.getRawButtonPressed(2) 

    val moveX
        get() = rightJoy.x

    val moveY
        get() = rightJoy.y

    val moveTwist
        get() = if (hosas) {leftJoy.x} else rightJoy.twist
}
