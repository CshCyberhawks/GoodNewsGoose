package frc.robot.commands.auto

import cshcyberhawks.swolib.autonomous.commands.GoToPosition
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

    fun setPos() {
        val pitchRoll: Vector2 = gyro.mergePitchRoll()
        val negPitchRoll: Vector2 = Vector2.fromPolar(Polar(-Polar.fromVector2(pitchRoll).theta, .15))
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
    }

    override fun end(interrupted: Boolean) {
        driveTrain.kill()
    }

    override fun isFinished(): Boolean {
        SmartDashboard.putNumber("rollPitchMag", Polar.fromVector2(gyro.mergePitchRoll()).r)
        return Polar.fromVector2(gyro.mergePitchRoll()).r < 2.5
    }
}
