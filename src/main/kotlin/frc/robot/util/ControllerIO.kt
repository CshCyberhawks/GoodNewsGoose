package frc.robot.util

import cshcyberhawks.swolib.math.MiscCalculations
import edu.wpi.first.wpilibj.XboxController

object ControllerIO {
    private val xbox = XboxController(2)

    private var lastToggleGrabber = false
    val spinClaw: Boolean
        get() {
            val current = xbox.xButton
            val toggled = current && !lastToggleGrabber
            lastToggleGrabber = current
            return toggled
        }

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

    val armAlignTop
        get() = xbox.pov == 0
    val armAlignMid
        get() = xbox.pov == 90
    val armClose
        get() = xbox.pov == 180
    val commandCancel
        get() = xbox.pov == 270

    val armAlignClosed: Boolean
        get() = xbox.startButton
    val armAlignUp
        get() = xbox.yButton

    val armAlignDown
        get() = xbox.bButton

    val clawSpit
        get() = xbox.backButton
}