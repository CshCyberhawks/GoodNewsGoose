package frc.robot.commands.auto.arm.align

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.arm.AutoArmPosition
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ExtensionPosition

class ArmAlignTop(private val armSystem: ArmSystem): SequentialCommandGroup() {
    init {
        addCommands(
            AutoArmPosition(armSystem, 130.0, ExtensionPosition.RETRACTED, true, false),
            AutoArmPosition(armSystem, 130.0, ExtensionPosition.EXTENDED, true, false)
        )
    }
}