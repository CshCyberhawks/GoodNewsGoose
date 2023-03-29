package frc.robot.commands.auto

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Polar
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveOdometry
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import frc.robot.Robot


class AutoBalance(val gyro: GenericGyro, val swerveAuto: SwerveAuto, val swo: SwerveOdometry) : CommandBase() {

    lateinit var pos: GoToPosition

    var roll: Double = 0.0
    var pitch: Double = 0.0

    val deadzone: Double = 3.0

    override fun initialize() {
        setPos()
        pos.schedule()
    }

    fun setPos() {
        val pitchRoll: Vector2 = gyro.mergePitchRoll()
        val negPitchRoll: Vector2 = Vector2.fromPolar(Polar(-Polar.fromVector2(pitchRoll).theta, 0.3))
        val robotPosMeters: Vector2 = Vector2(swo.fieldPosition.x, swo.fieldPosition.y)
        val position: Vector2 = robotPosMeters + negPitchRoll
        pos = GoToPosition(swerveAuto, position)
        roll = gyro.getRoll()
        pitch = gyro.getPitch()
        SmartDashboard.putNumber("AutoBalance Position X", position.x)
        SmartDashboard.putNumber("AutoBalance Position Y", position.y)
    }

    override fun execute() {
        if (pos.isScheduled == false || pos.isFinished()) {
            setPos()
            pos.schedule()
        } else if (MiscCalculations.calculateDeadzone(gyro.getRoll() - roll, deadzone) != 0.0 || MiscCalculations.calculateDeadzone(gyro.getPitch() - pitch, deadzone) != 0.0) {
            pos.cancel()
            setPos()
            pos.schedule()
        }
    }

    override fun end(interrupted: Boolean) {
        swerveAuto.kill()
    }

    override fun isFinished(): Boolean {
        SmartDashboard.putNumber("rollPitchMag", Polar.fromVector2(gyro.mergePitchRoll()).r)
        return Polar.fromVector2(gyro.mergePitchRoll()).r < 6.0
    }
}
