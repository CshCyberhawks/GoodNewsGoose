package frc.robot.commands

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.autonomous.commands.LimeLightAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPositionAndExecute
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.FieldPosition
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import TimedFinish
import cshcyberhawks.swolib.autonomous.paths.AutoPathManager
import cshcyberhawks.swolib.math.Vector2
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ExtensionPosition

// define an empty SequentialCommandGroup
class TestingAuto(private val swerveAuto: SwerveAuto, private val gyro: GenericGyro, private val armSystem: ArmSystem, private val autoPathManager: AutoPathManager) :
    SequentialCommandGroup() {

    // define the constructor
    init {
        gyro.setYawOffset()
        // add the commands to the SequentialCommandGroup
        addCommands(
            AutoArmPosition(armSystem, 126.0, ExtensionPosition.RETRACTED, true, false),
            AutoArmPosition(armSystem, 126.0, ExtensionPosition.EXTENDED, true, false),
            AutoArmPosition(armSystem, 120.0, ExtensionPosition.EXTENDED, true, false),
            AutoArmPosition(armSystem, 120.0, ExtensionPosition.EXTENDED, true, true),
            AutoArmPosition(armSystem, 126.0, ExtensionPosition.RETRACTED, true, true),
            AutoArmPosition(armSystem, 40.0, ExtensionPosition.RETRACTED, false, true),
            AutoArmPosition(armSystem, 40.0, ExtensionPosition.RETRACTED, false, false),
            //            GoToPosition(swerveAuto, Vector2(0.0 - 4.5))
//            autoPathManager.paths["TaxiAndBalance"]!!
        )
    }
}