package frc.robot.commands.auto.arm

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.AngleMovement
import frc.robot.commands.auto.ExtensionMovement
import frc.robot.commands.auto.TiltMovement
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem

public class AutoAlignClosed(armSystem: ArmSystem) : SequentialCommandGroup() {
    init {
        addCommands(
            AutoArmPosition(armSystem, listOf(
                ExtensionMovement(armSystem, ArmConstants.armExtensionIn),
                TiltMovement(armSystem, false),
                AngleMovement(armSystem, ArmConstants.armInAngle)
            ))
        )
    }
}