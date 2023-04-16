package frc.robot.commands.auto

import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.arm.AutoArmPosition
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawState
import frc.robot.subsystems.ClawSystem

public class AutoPlace(armSystem: ArmSystem, clawSystem: ClawSystem) : SequentialCommandGroup() {
    init {
        addCommands(
                AutoArmPosition(armSystem, clawSystem, 126.0, ArmConstants.armExtensionIn, true),
                AutoArmPosition(armSystem, clawSystem, 126.0, ArmConstants.armExtensionOut, true),
                AutoArmPosition(armSystem, clawSystem, 120.0, ArmConstants.armExtensionOut, true),
                AutoArmPosition(armSystem, clawSystem, 120.0, ArmConstants.armExtensionOut, true, ClawState.Spitting),
                AutoArmPosition(armSystem, clawSystem, 126.0, ArmConstants.armExtensionIn, true),
                AutoArmPosition(armSystem, clawSystem, 35.0, ArmConstants.armExtensionIn, true),
                AutoArmPosition(armSystem, clawSystem, 35.0, ArmConstants.armExtensionIn, false),
        )
    }
}