package frc.robot.commands.auto

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.auto.arm.AutoArmPosition
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem

public class AutoPlaceHigh(armSystem: ArmSystem, clawSystem: ClawSystem) : SequentialCommandGroup() {
    init {
        addCommands(
                AutoArmPosition(armSystem, clawSystem, ArmConstants.armHighAngle, ArmConstants.armExtensionIn, true),
                AutoArmPosition(armSystem, clawSystem, extensionPositionInput = ArmConstants.armExtensionOut),
                AutoArmPosition(armSystem, clawSystem, armAngleInput = ArmConstants.armHighAngle - 5),
                AutoArmPosition(armSystem, clawSystem, clawStateInput = ClawState.Spitting),
                WaitCommand(1.0),
                AutoArmPosition(armSystem, clawSystem, armAngleInput = ArmConstants.armHighAngle, extensionPositionInput = ArmConstants.armExtensionIn, clawStateInput = ClawState.Idle),
                AutoArmPosition(armSystem, clawSystem, armAngleInput = ArmConstants.armInAngle),
                AutoArmPosition(armSystem, clawSystem, tiltInput = false),
        )
    }
}