package frc.robot.commands

import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.math.FieldPosition
import cshcyberhawks.swolib.math.Vector2
import edu.wpi.first.wpilibj2.command.CommandBase



class AutoLimelightSingleAxis(val swerveAuto: SwerveAuto, val limelight: Limelight, val targetHeight: Double, val axis: Axis, val pip: Int, val setAngle: Boolean = false) : CommandBase() {
    enum class Axis {
        X, Y
    }

    init {
        addRequirements(swerveAuto.swerveSystem)
    }

    var didSetDesired: Boolean = false

    fun setPos() {
        if (didSetDesired) return
        if (limelight.getCurrentPipeline().toInt() != pip) return
        if (axis == Axis.X) {
            swerveAuto.desiredPosition =
                FieldPosition(
                    Vector2(
                        limelight.getPosition(swerveAuto.swo, targetHeight, swerveAuto.gyro).x,
                        swerveAuto.swo.fieldPosition.y
                    ),
                    0.0
                )
        }
        if (axis == Axis.Y) {
            swerveAuto.desiredPosition =
                FieldPosition(
                    Vector2(
                        swerveAuto.swo.fieldPosition.x,
                        limelight.getPosition(swerveAuto.swo, targetHeight, swerveAuto.gyro).y
                    ),
                    0.0
                )
        }
        if (setAngle == true)
        {
            swerveAuto.setDesiredAngleRelative(limelight.getHorizontalOffset())
        }
        didSetDesired = true
    }

    override fun initialize() {

    }

    override fun execute() {
        setPos()
        swerveAuto.move()
    }

    override fun isFinished(): Boolean {
        return swerveAuto.isFinishedMoving()
    }

    override fun end(int: Boolean) {
        swerveAuto.kill()
    }
}
