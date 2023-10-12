package frc.robot.commands.auto.arm

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.AngleMovement
import frc.robot.commands.auto.ExtensionMovement
import frc.robot.commands.auto.TiltMovement
import frc.robot.constants.ArmConstants
import frc.robot.subsystems.ArmSystem

public class AutoAlignHigh(armSystem: ArmSystem) : SequentialCommandGroup() {
    init {
        addCommands(
            AutoArmPosition(armSystem, listOf(
                AngleMovement(armSystem, ArmConstants.armMidAngle),
                TiltMovement(armSystem, true),
                AngleMovement(armSystem, ArmConstants.armHighAngle),
                ExtensionMovement(armSystem, ArmConstants.armExtensionMid)
            ))
        )
    }
}