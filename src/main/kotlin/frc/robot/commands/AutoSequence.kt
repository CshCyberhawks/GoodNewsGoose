package frc.robot.commands

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import cshcyberhawks.swolib.autonomous.paths.AutoPathManager
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import frc.robot.commands.auto.Configurations.BumbsideBalance
import frc.robot.commands.auto.Configurations.Top2Piece
import frc.robot.commands.auto.Configurations.PlaceAndTaxiAndBalanceHigh
import frc.robot.commands.auto.Configurations.PlaceAndTaxiBump
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawSystem

enum class AutoSequenceType {
    Test,
    None,
    TaxiTop,
    TaxiBalanceTop,
    BumpsideBalance,
    Top2Piece,
    PlaceAndTaxiBump
}
class AutoSequence(private val swerveAuto: SwerveAuto, private val gyro: GenericGyro, private val armSystem: ArmSystem, private val autoPathManager: AutoPathManager, private val swerveSystem: SwerveDriveTrain, private val clawSystem: ClawSystem, private val currentSequence: AutoSequenceType) :
    SequentialCommandGroup() {
    init {
        gyro.setYawOffset()
        when (currentSequence) {
            AutoSequenceType.None -> {}
            AutoSequenceType.Test -> testFunction()
            AutoSequenceType.TaxiTop -> taxiTop()
            AutoSequenceType.TaxiBalanceTop -> taxiBalanceTop()
            AutoSequenceType.BumpsideBalance -> {
                BumbsideBalance(swerveAuto, gyro, armSystem, autoPathManager, swerveSystem, clawSystem).schedule()
            }
            AutoSequenceType.Top2Piece -> {
                Top2Piece(swerveAuto, gyro, armSystem, autoPathManager, swerveSystem, clawSystem).schedule()
            }
            AutoSequenceType.PlaceAndTaxiBump -> {
                PlaceAndTaxiBump(swerveAuto, gyro, armSystem, autoPathManager, swerveSystem, clawSystem).schedule()
            }
        }
    }

    fun taxiTop() {
        PlaceAndTaxiHigh(swerveAuto, gyro, armSystem, autoPathManager, swerveSystem, clawSystem).schedule()
    }

    fun taxiBalanceTop() {
        PlaceAndTaxiAndBalanceHigh(swerveAuto, gyro, armSystem, autoPathManager, swerveSystem, clawSystem).schedule()
    }

    fun testFunction() {
        println("Asdfasdfasf")
    }

}
