package cshcyberhawks.swolib.autonomous

import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.AngleCalculations
import cshcyberhawks.swolib.math.FieldPosition
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.swerve.SwerveDriveTrain
import cshcyberhawks.swolib.swerve.SwerveOdometry
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import kotlin.math.abs

class SwerveAuto(
    val xPID: ProfiledPIDController,
    val yPID: ProfiledPIDController,
    val twistPID: PIDController,
    val angleDeadzone: Double,
    val positionDeadzone: Double,
    val twistFeedForward: Double,
    val swo: SwerveOdometry,
    val swerveSystem: SwerveDriveTrain,
    val gyro: GenericGyro,
    val debugLogging: Boolean = false
) {
    var desiredPosition: FieldPosition = FieldPosition(0.0, 0.0, 0.0)
        set(value) {
            xPID.goal = TrapezoidProfile.State(value.x, 0.0)
            yPID.goal = TrapezoidProfile.State(value.x, 0.0)
            field = value
        }

    fun setDesiredEndVelocity(velo: Vector2) {
        this.xPID.goal = TrapezoidProfile.State(xPID.goal.position, velo.x)
        this.yPID.goal = TrapezoidProfile.State(yPID.goal.position, velo.x)
    }

    val autoShuffleboardTab = Shuffleboard.getTab("Auto")

    //make all the shuffleboard items entries
//    val xPIDShuffleboard = autoShuffleboardTab.add("X PID", xPID)
//    val yPIDShuffleboard = autoShuffleboardTab.add("Y PID", yPID)
//    val twistPIDShuffleboard = autoShuffleboardTab.add("Twist PID", twistPID)

    val translationTwistShuffleboard = autoShuffleboardTab.add("Translation Twist", 0.0).entry

    val xPIDOutputShuffle = autoShuffleboardTab.add("X PID OUT", 0.0).entry
    val yPIDOutputShuffle = autoShuffleboardTab.add("Y PID OUT", 0.0).entry
    val twistPIDOutputShuffle = autoShuffleboardTab.add("Twist PID OUT", 0.0).entry


    init {
        twistPID.enableContinuousInput(0.0, 1.0)
    }

    // twists and translates
    fun move() {
        var translation: Vector2 = Vector2(0.0, 0.0)
        var twist: Double = 0.0

        if (!isAtDesiredPosition()) {
            translation = calculateTranslation()
        }

        val atDesiredAngle = isAtDesiredAngle()
        if (!atDesiredAngle) {
            twist = calculateTwist(desiredPosition.angle)
        }

        if (debugLogging) {
            translationTwistShuffleboard.setDouble(twist)
        }

        swerveSystem.drive(translation, twist)
    }

    private fun calculateTwist(desiredAngle: Double): Double {

        //ryan suggested this
        val pidVal = twistPID.calculate(gyro.getYaw() / 360, desiredAngle / 360)

        if (abs(pidVal) < 0.1) {
            return -pidVal
        }

        val twistFF = if (pidVal > 0.0) twistFeedForward else -twistFeedForward

        if (debugLogging) {
            twistPIDOutputShuffle.setDouble(pidVal)
        }

        // TODO: Fix this for other gyros
        return -(pidVal + twistFF)
    }

    private fun calculateTranslation(): Vector2 {
        val xPIDOutput =
            xPID.calculate(
                swo.fieldPosition.x
            )
        val yPIDOutput =
            yPID.calculate(
                swo.fieldPosition.y
            )


        if (debugLogging) {
            xPIDOutputShuffle.setDouble(xPIDOutput)
            yPIDOutputShuffle.setDouble(yPIDOutput)
        }

        return Vector2(xPIDOutput, yPIDOutput)
    }

    private fun isAtDesiredAngle(): Boolean {
        return MiscCalculations.calculateDeadzone(
            AngleCalculations.wrapAroundAngles(
                gyro.getYaw() -
                        desiredPosition.angle
            ),
            this.angleDeadzone
        ) == 0.0
    }

    private fun isAtDesiredPosition(): Boolean {
        return (MiscCalculations.calculateDeadzone(
            desiredPosition.x - swo.fieldPosition.x,
            positionDeadzone
        ) == 0.0 &&
                MiscCalculations.calculateDeadzone(
                    desiredPosition.y - swo.fieldPosition.y,
                    positionDeadzone
                ) == 0.0)
    }

    fun setDesiredAngleRelative(desiredAngle: Double) {
        desiredPosition = FieldPosition(
            desiredPosition.x,
            desiredPosition.y,
            AngleCalculations.wrapAroundAngles(gyro.getYaw() + desiredAngle)
        )
    }

    fun isFinishedMoving(): Boolean {
        return isAtDesiredAngle() && isAtDesiredPosition()
    }

    fun kill() {
        swerveSystem.drive(Vector2(0.0, 0.0), 0.0)
    }
}
