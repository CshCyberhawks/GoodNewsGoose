package frc.robot.commands.auto

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.Polar
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveOdometry
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase

class AutoBalance(val gyro: GenericGyro, val swerveAuto: SwerveAuto, val swo: SwerveOdometry) :
        CommandBase() {

    var roll: Double = 0.0
    var pitch: Double = 0.0

    val deadzone: Double = 3.0

    private lateinit var pos: GoToPosition

    override fun initialize() {
        setPos()
    }

    fun setPos() {
        val pitchRoll: Vector2 = gyro.mergePitchRoll()
        val negPitchRoll: Vector2 =
                Vector2.fromPolar(
                        Polar(
                                -Polar.fromVector2(pitchRoll).theta,
                                Polar.fromVector2(pitchRoll).r / 40
                        )
                )
        val robotPosMeters: Vector2 = Vector2(swo.fieldPosition.x, swo.fieldPosition.y)
        val position: Vector2 = negPitchRoll
        pos = GoToPosition(swerveAuto, position)
        roll = gyro.getRoll()
        pitch = gyro.getPitch()
        SmartDashboard.putNumber("AutoBalance Position X", position.x)
        SmartDashboard.putNumber("AutoBalance Position Y", position.y)
        SmartDashboard.putNumber("Pitch", pitch)
        SmartDashboard.putNumber("Roll", roll)
    }

    override fun execute() {
        //        if (MiscCalculations.calculateDeadzone(gyro.getRoll() - roll, deadzone) != 0.0 ||
        // MiscCalculations.calculateDeadzone(gyro.getPitch() - pitch, deadzone) != 0.0) {
        setPos()
        //        }
    }

    override fun end(interrupted: Boolean) {
        pos.cancel()
    }

    override fun isFinished(): Boolean {
        // SmartDashboard.putNumber("rollPitchMag", Polar.fromVector2(gyro.mergePitchRoll()).r)
        return Polar.fromVector2(gyro.mergePitchRoll()).r < 6.0
        // return false
    }
}
