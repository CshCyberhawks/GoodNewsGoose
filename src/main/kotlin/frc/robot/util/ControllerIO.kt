package frc.robot.util

import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.wpilibj.XboxController
import kotlin.math.abs

object ControllerIO {
    private val xbox = XboxController(2)

    val spinClaw: Boolean
        get() = abs(xbox.rightTriggerAxis) > 0.1

    private var toggleTiltLast = false
    val toggleTilt: Boolean
        get() {
            val current = xbox.aButton
            val toggled = current && !toggleTiltLast
            toggleTiltLast = current
            return toggled
        }

    val controlArmAngle
        get() = MiscCalculations.calculateDeadzone(-xbox.leftY, 0.1)

    val extensionManualControl
        get() = MiscCalculations.calculateDeadzone(xbox.rightY, 0.1)

    val extensionExtended
        get() = xbox.rightBumper

    val extensionRetracted
        get() = xbox.leftBumper

    val armAlignClosed: Boolean
        get() = xbox.xButton

    val armAlignFloorCube
        get() = xbox.startButton

    val armAlignFloorCone
        get() = xbox.backButton

    val clawSpit: Boolean
        get() = abs(xbox.leftTriggerAxis) > 0.1

    val armAlignHigh
        get() = xbox.yButtonPressed

    val armPlaceHigh
        get() = xbox.yButtonReleased

    val armAlignMid
        get() = xbox.bButtonPressed

    val armPlaceMid
        get() = xbox.bButtonReleased

    val armAlignShelf
        get() = xbox.pov != -1
}