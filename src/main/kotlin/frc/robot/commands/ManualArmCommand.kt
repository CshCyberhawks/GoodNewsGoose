package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.util.ControllerIO

/**
 * @property subsystem
 */

private interface GenericArmMovement {
    var isRunning: Boolean

    fun run()

    fun isDone(): Boolean
}

private class AngleMovement(private val subsystem: ArmSystem, private val angle: Double) : GenericArmMovement {
    override var isRunning: Boolean = false

    override fun run() {
        subsystem.desiredArmAngle = angle
        isRunning = true
    }

    override fun isDone(): Boolean = subsystem.isFinished()
}

private class ExtensionMovement(private val subsystem: ArmSystem, private val extensionPosition: Double) : GenericArmMovement {
    override var isRunning: Boolean = false

    override fun run() {
        subsystem.desiredExtensionPosition = extensionPosition
        isRunning = true
    }

    override fun isDone(): Boolean = subsystem.isFinished()
}

private class TiltMovement(private val subsystem: ArmSystem, private val tiltPosition: Boolean) : GenericArmMovement {
    override var isRunning: Boolean = false

    override fun run() {
        subsystem.desiredTilt = tiltPosition
        isRunning = true
    }

    override fun isDone(): Boolean = true
}


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

        if (ControllerIO.armAlignClosed) {
            armQueue.add(ExtensionMovement(subsystem, ArmConstants.armExtensionIn))
            armQueue.add(TiltMovement(subsystem, false))
            armQueue.add(AngleMovement(subsystem, ArmConstants.armInAngle))
        }

        if (ControllerIO.armAlignShelf) {
            subsystem.desiredArmAngle = ArmConstants.armMidAngle
        }

        if (ControllerIO.armAlignFloorCube) {
            subsystem.desiredArmAngle = 45.0
            subsystem.desiredTilt = true
            subsystem.desiredExtensionPosition = ArmConstants.armExtensionIn
        }

        if (ControllerIO.armAlignTop) {
            armQueue.add(AngleMovement(subsystem, ArmConstants.armMidAngle))
            armQueue.add(TiltMovement(subsystem, true))
            armQueue.add(AngleMovement(subsystem, ArmConstants.armHighAngle))
            armQueue.add(ExtensionMovement(subsystem, ArmConstants.armExtensionMid))
        }

        if (ControllerIO.armAlignMid) {
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
