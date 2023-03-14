package frc.robot.commands

import cshcyberhawks.swolib.autonomous.SwerveAuto
import cshcyberhawks.swolib.autonomous.commands.GoToPosition
import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.FieldPosition
import cshcyberhawks.swolib.math.Vector2
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase

class AutoLimelightSingleAxis(
    val swerveAuto: SwerveAuto,
    val limelight: Limelight,
    private val targetHeight: Double,
    private val axis: Axis,
    private val pipe: Int,
    private val setAngle: Boolean = false
) : CommandBase() {
    enum class Axis {
        X,
        Y
    }

    private var command: CommandBase? = null

    private fun setPos() {
        if (command != null) {
            return
        }

        val position = limelight.getPosition(
            swerveAuto.swo,
            targetHeight,
            swerveAuto.gyro
        )

        if (position.isEmpty) {
            return
        }

        // // if (limelight.pipeline != pip) return
        if (axis == Axis.X) {
            command =
                GoToPosition(
                    swerveAuto,
                    FieldPosition(
                        Vector2(
                            position.get().x,
                            swerveAuto.swo.fieldPosition.y
                        ),
                        0.0
                    )
                )
        }
        if (axis == Axis.Y) {
            command =
                GoToPosition(
                    swerveAuto,
                    FieldPosition(
                        Vector2(
                            swerveAuto.swo.fieldPosition.x,
                            position.get().y
                        ),
                        0.0
                    )
                )
        }
        if (setAngle) {
            swerveAuto.setDesiredAngleRelative(limelight.getHorizontalOffset().get())
        }
        command?.schedule()
        println("set desired")
    }

    override fun initialize() {
        setPos()
        println("init")
    }

    override fun execute() {}

    override fun isFinished(): Boolean {
        SmartDashboard.putBoolean("single axis", swerveAuto.isFinishedMoving() && command != null)
        return swerveAuto.isFinishedMoving() && command != null
    }

    override fun end(int: Boolean) {
        swerveAuto.kill()
        println("done")
    }
}
