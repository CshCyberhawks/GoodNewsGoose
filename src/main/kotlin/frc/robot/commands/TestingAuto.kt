package frc.robot.commands

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import cshcyberhawks.swolib.autonomous.paths.AutoPathManager
import cshcyberhawks.swolib.math.FieldPosition
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import frc.robot.commands.auto.AutoPlaceHigh
import frc.robot.commands.auto.AutoPlaceMid
import frc.robot.commands.auto.DumbAutoBalance
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawSystem

// define an empty SequentialCommandGroup
class TestingAuto(private val swerveAuto: SwerveAuto, private val gyro: GenericGyro, private val armSystem: ArmSystem, private val autoPathManager: AutoPathManager, private val swerveSystem: SwerveDriveTrain, private val clawSystem: ClawSystem) :
        SequentialCommandGroup() {

    // define the constructor
    init {
        gyro.setYawOffset()
        // add the commands to the SequentialCommandGroup
        addCommands(
                AutoPlaceHigh(armSystem, clawSystem),
//                AutoPlaceMid(armSystem, clawSystem),
                GoToPosition(swerveAuto, Vector2(0.0, -4.4)),
//                                AutoPlaceHigh(armSystem, clawSystem)
//                GoToPosition(swerveAuto, Vector2(0.0, -2.6)),
//                DumbAutoBalance(gyro, swerveSystem),
//                GoToPosition(swerveAuto, FieldPosition(swerveAuto.swo.fieldPosition.x, swerveAuto.swo.fieldPosition.y, gyro.getYaw() + 5))

//                AutoPlace()
//            autoPathManager.paths["BalanceStart"]!!,
//            AutoArmPosition(armSystem, 35.0, ExtensionPosition.RETRACTED, false, false),
//                GoToPosition(swerveAuto, FieldPosition(0.0, -2.6, 15.0))
//            autoPathManager.paths["Balance"]!!,
//            autoPathManager.paths["TaxiTop"]!!,
//            LockWheels(swerveSystem)
        )
    }
    //balance
//    GoToPosition(swerveAuto, Vector2(0.0, -2.55)),
    //            GoToPosition(swerveAuto, FieldPosition(0.0, -2.55, 15.0))
}
