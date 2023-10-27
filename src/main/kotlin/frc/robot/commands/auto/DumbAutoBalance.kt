package frc.robot.commands.auto

import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Polar
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase


class DumbAutoBalance(val gyro: GenericGyro, val driveTrain: SwerveDriveTrain) : CommandBase() {

    override fun initialize() {
        setPos()
    }

    private var levelStart = 0.0
    private var isTiming = false

    fun setPos() {
        val pitchRoll: Vector2 = gyro.mergePitchRoll()
        var negPitchRoll: Vector2 = Vector2()
        if (Polar.fromVector2(gyro.mergePitchRoll()).r >= 9.0) {
            negPitchRoll = Vector2.fromPolar(Polar(-Polar.fromVector2(pitchRoll).theta, .09))
        }

        val position: Vector2 = negPitchRoll
        driveTrain.drive(position, 0.0)
//        driveTrain.drive(Vector2(0.0, 0.0), 0.0)

        SmartDashboard.putNumber("AutoBalance Position X", position.x)
        SmartDashboard.putNumber("AutoBalance Position Y", position.y)
        SmartDashboard.putNumber("Pitch", gyro.getRoll())
        SmartDashboard.putNumber("Roll", gyro.getPitch())
    }

    override fun execute() {
        setPos()
        SmartDashboard.putNumber("rollPitchMag", Polar.fromVector2(gyro.mergePitchRoll()).r)
    }

    override fun end(interrupted: Boolean) {
//        driveTrain.drive(Vector2(), .1)
        driveTrain.kill()
    }

    override fun isFinished(): Boolean {

        if (Polar.fromVector2(gyro.mergePitchRoll()).r < 9.0 && !isTiming) {
            levelStart = MiscCalculations.getCurrentTimeSeconds()
            isTiming = true
        }

        if (isTiming && Polar.fromVector2(gyro.mergePitchRoll()).r >= 9.0) {
            isTiming = false
        }

//        SmartDashboard.putNumber("rollPitchMag", Polar.fromVector2(gyro.mergePitchRoll()).r)
        SmartDashboard.putNumber("timer: ", MiscCalculations.getCurrentTimeSeconds() - levelStart)
        if (isTiming && MiscCalculations.getCurrentTimeSeconds() - levelStart >= 150.0) {
            return false
        }
        return false
    }
}
