package frc.robot.commands

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import cshcyberhawks.swolib.autonomous.paths.AutoPathManager
import cshcyberhawks.swolib.math.FieldPosition
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import frc.robot.commands.auto.Configurations.PlaceAndTaxiAndBalanceHigh
import frc.robot.commands.auto.DumbAutoBalance
import frc.robot.subsystems.ArmSystem
import frc.robot.subsystems.ClawSystem

enum class AutoSequenceType {
    Test,
    None,
    TaxiTop,
    TaxiBalanceTop,
}
class AutoSequence(private val swerveAuto: SwerveAuto, private val gyro: GenericGyro, private val armSystem: ArmSystem, private val autoPathManager: AutoPathManager, private val swerveSystem: SwerveDriveTrain, private val clawSystem: ClawSystem, private val currentSequence: AutoSequenceType) :
    SequentialCommandGroup() {
    init {
        gyro.setYawOffset()
        when (currentSequence) {
            AutoSequenceType.Test -> testFunction()
            AutoSequenceType.TaxiTop -> taxiTop()
            AutoSequenceType.TaxiBalanceTop -> taxiBalanceTop()
            AutoSequenceType.None -> {}
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