package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.commands.auto.AngleMovement
import frc.robot.commands.auto.ExtensionMovement
import frc.robot.commands.auto.GenericArmMovement
import frc.robot.commands.auto.TiltMovement
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.util.ControllerIO

/**
 * @property subsystem
 */
class ManualArmCommand(private val subsystem: ArmSystem) : CommandBase() {
    private val armQueue = arrayListOf<GenericArmMovement>()

    // Called when the command is initially scheduled.
    override fun initialize() {
        subsystem.desiredArmAngle = 35.0

//        setCurrentCommand(AutoArmPosition(subsystem, 90.0, 0.0, false, true))
    }

    private fun armLogic() {
        if (armQueue.size != 0) {
            if (!armQueue[0].isRunning) {
                armQueue[0].run()
            }

            if (armQueue[0].isDone()) {
                armQueue.removeAt(0)
            }

            return
        }

        if (ControllerIO.toggleTilt) {
            subsystem.desiredTilt = !subsystem.desiredTilt
        }

        if (ControllerIO.extensionExtended) {
            subsystem.desiredExtensionPosition = ArmConstants.armExtensionOut
//            subsystem.desiredExtensionPosition = ExtensionPosition.EXTENDED

        }
        if (ControllerIO.extensionRetracted) {
            if (!subsystem.extensionInBeamBreak.get()) {
                subsystem.desiredExtensionPosition = 0.0
            }
//            subsystem.desiredExtensionPosition = ExtensionPosition.RETRACTED
        }

        subsystem.desiredArmAngle += ControllerIO.controlArmAngle * 2

        if (ControllerIO.armAlignClosed) {
            armQueue.clear()
            armQueue.add(ExtensionMovement(subsystem, ArmConstants.armExtensionIn))
            armQueue.add(TiltMovement(subsystem, false))
            armQueue.add(AngleMovement(subsystem, ArmConstants.armInAngle))
        }

        if (ControllerIO.armAlignShelf) {
            armQueue.clear()
            subsystem.desiredArmAngle = ArmConstants.armShelfAngle
        }

        if (ControllerIO.armAlignFloorCube) {
            armQueue.clear()
            subsystem.desiredArmAngle = ArmConstants.armFloorCubeAngle
            subsystem.desiredTilt = true
            subsystem.desiredExtensionPosition = ArmConstants.armExtensionIn
        }

        if (ControllerIO.armAlignFloorCone) {
            armQueue.clear()
            subsystem.desiredArmAngle = ArmConstants.armFloorConeAngle
            subsystem.desiredTilt = true
            subsystem.desiredExtensionPosition = ArmConstants.armExtensionOut
        }

        if (ControllerIO.armAlignHigh) {
            armQueue.clear()
            armQueue.add(AngleMovement(subsystem, ArmConstants.armMidAngle))
            armQueue.add(TiltMovement(subsystem, true))
            armQueue.add(AngleMovement(subsystem, ArmConstants.armHighAngle))
            armQueue.add(ExtensionMovement(subsystem, ArmConstants.armExtensionOut))
        }

        if (ControllerIO.armAlignMid) {
            armQueue.clear()
            armQueue.add(AngleMovement(subsystem, ArmConstants.armMidAngle))
            armQueue.add(ExtensionMovement(subsystem, ArmConstants.armExtensionMid))
        }
//
//        if (ControllerIO.armAlignMid) {
//            subsystem.desiredArmAngle = ArmConstants.armMidAngle
//            subsystem.desiredExtensionPosition = ArmConstants.armExtensionOut
//            subsystem.desiredTilt = false
//        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        armLogic()

        subsystem.run()
    }

    // Called once the command ends or is interruppted.
    override fun end(interrupted: Boolean) {}

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
