package frc.robot.commands.auto.arm.align

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import frc.robot.commands.auto.arm.AutoArmPosition
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ExtensionPosition

class ArmAlignClosed(private val armSystem: ArmSystem) : SequentialCommandGroup() {
    init {
        addCommands(
                AutoArmPosition(armSystem, 130.0, 3600.0, false, false),
                AutoArmPosition(armSystem, 130.0, 0.0, false, false),
                AutoArmPosition(armSystem, 45.0, 0.0, false, false),
        )
    }
}