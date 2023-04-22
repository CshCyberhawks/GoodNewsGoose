package frc.robot.commands.auto

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.auto.arm.AutoArmPosition
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem

public class AutoPlaceMid(armSystem: ArmSystem, clawSystem: ClawSystem) : SequentialCommandGroup() {
    init {
        addCommands(
                AutoArmPosition(armSystem, clawSystem, 85.0, ArmConstants.armExtensionIn, false),
                AutoArmPosition(armSystem, clawSystem, 85.0, ArmConstants.armExtensionMid, false),
                AutoArmPosition(armSystem, clawSystem, 85.0, ArmConstants.armExtensionMid, false, ClawState.Spitting),
                WaitCommand(1.0),
                AutoArmPosition(armSystem, clawSystem, 85.0, ArmConstants.armExtensionIn, false),
                AutoArmPosition(armSystem, clawSystem, 35.0, ArmConstants.armExtensionIn, false),
                AutoArmPosition(armSystem, clawSystem, 35.0, ArmConstants.armExtensionIn, false),
        )
    }
}