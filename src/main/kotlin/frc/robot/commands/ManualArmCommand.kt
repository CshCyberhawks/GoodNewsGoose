package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.util.ControllerIO

/**
 * @property subsystem
 */
class ManualArmCommand(private val subsystem: ArmSystem) : CommandBase() {
    // Called when the command is initially scheduled.
    override fun initialize() {
        subsystem.desiredArmAngle = 35.0

//        setCurrentCommand(AutoArmPosition(subsystem, 90.0, 0.0, false, true))
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        if (ControllerIO.toggleTilt) {
            subsystem.desiredTilt = !subsystem.desiredTilt
        }

        if (ControllerIO.extensionExtended) {
            subsystem.desiredExtensionPosition = ArmConstants.armExtensionMid
//            subsystem.desiredExtensionPosition = ExtensionPosition.EXTENDED

        }
        if (ControllerIO.extensionRetracted) {
            if (!subsystem.extensionInBeamBreak.get()) {
                subsystem.desiredExtensionPosition = 0.0
            }
//            subsystem.desiredExtensionPosition = ExtensionPosition.RETRACTED
        }

        subsystem.desiredArmAngle += ControllerIO.controlArmAngle * 2

        if (ControllerIO.armAlignUp) {
            subsystem.desiredArmAngle = if (subsystem.desiredTilt) {
                ArmConstants.armMidAngle + 15
            } else {
                ArmConstants.armMidAngle
            }
        }

        if (ControllerIO.armAlignDown) {
            subsystem.desiredArmAngle = ArmConstants.armInAngle
        }

        if (ControllerIO.armAlignClosed) {
            subsystem.desiredArmAngle = ArmConstants.armInAngle
            subsystem.desiredExtensionPosition = 0.0
//            subsystem.desiredExtensionPosition = ExtensionPosition.RETRACTED
            subsystem.desiredTilt = false
        }

        if (ControllerIO.armAlignFloor) {
            subsystem.desiredArmAngle = 45.0
            subsystem.desiredTilt = true
        }

//        if (ControllerIO.armAlignTop) {
//            subsystem.desiredArmAngle = ArmConstants.armHighAngle
//            subsystem.desiredExtensionPosition = ArmConstants.armExtensionOut
//            subsystem.desiredTilt = true
//        }
//
//        if (ControllerIO.armAlignMid) {
//            subsystem.desiredArmAngle = ArmConstants.armMidAngle
//            subsystem.desiredExtensionPosition = ArmConstants.armExtensionOut
//            subsystem.desiredTilt = false
//        }
//
//        if (ControllerIO.armAlignFloor) {
//            subsystem.desiredArmAngle = ArmConstants.armFloorAngle
//            subsystem.desiredExtensionPosition = ArmConstants.armExtensionOut
//            subsystem.desiredTilt = false
//        }

        subsystem.run()
    }

    // Called once the command ends or is interruppted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
