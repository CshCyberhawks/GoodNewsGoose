package frc.robot.commands.auto.arm.align

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.arm.AutoArmPosition
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ExtensionPosition

class ArmAlignMid(private val armSystem: ArmSystem): SequentialCommandGroup() {
    init {
        addCommands(
            AutoArmPosition(armSystem, 95.0, ExtensionPosition.RETRACTED, false, false),
            AutoArmPosition(armSystem, 95.0, ExtensionPosition.EXTENDED, false, false)
        )
    }
}