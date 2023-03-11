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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import kotlin.math.abs

class SwerveAuto(
    val xPID: ProfiledPIDController,
    val yPID: ProfiledPIDController,
    val twistPID: ProfiledPIDController,
    val angleDeadzone: Double,
    val positionDeadzone: Double,
    val swo: SwerveOdometry,
    val swerveSystem: SwerveDriveTrain,
    val gyro: GenericGyro,
    val debugLogging: Boolean = false
) {
    var desiredPosition: FieldPosition = FieldPosition(0.0, 0.0, 0.0)
        set(value) {
            xPID.goal = TrapezoidProfile.State(value.x, 0.0)
            yPID.goal = TrapezoidProfile.State(value.y, 0.0)
            xPID.reset(swo.fieldPosition.x)
            yPID.reset(swo.fieldPosition.y)
            twistPID.setGoal(field.angle)
            field = value
        }

    fun setDesiredEndVelocity(velo: Vector2) {
        this.xPID.goal = TrapezoidProfile.State(xPID.goal.position, velo.x)
        this.yPID.goal = TrapezoidProfile.State(yPID.goal.position, velo.y)
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
        twistPID.enableContinuousInput(0.0, 360.0)
    }

    private fun calculateTwist(desiredAngle: Double): Double {
        //ryan suggested this
        val pidVal = twistPID.calculate(gyro.getYaw()) / 360

        if (debugLogging) {
            twistPIDOutputShuffle.setDouble(pidVal)
        }

        return pidVal
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

        SmartDashboard.putNumber("X PID", xPIDOutput)
        SmartDashboard.putNumber("Y PID", yPIDOutput)

        if (debugLogging) {
            xPIDOutputShuffle.setDouble(xPIDOutput)
            yPIDOutputShuffle.setDouble(yPIDOutput)
        }

        return Vector2(xPIDOutput, yPIDOutput)
    }

    // twists and translates
    fun move() {
        var translation: Vector2 = Vector2(0.0, 0.0)
        var twist: Double = 0.0

        SmartDashboard.putBoolean("At Des Pos", isAtDesiredPosition())
        if (!isAtDesiredPosition()) {
            translation = calculateTranslation()
        }

        val atDesiredAngle = isAtDesiredAngle()
        SmartDashboard.putBoolean("At Des Angle", atDesiredAngle)
        if (!atDesiredAngle) {
            twist = calculateTwist(desiredPosition.angle)
        }

        if (debugLogging) {
            translationTwistShuffleboard.setDouble(twist)
        }

        swerveSystem.drive(translation, twist)
    }

    private fun isAtDesiredAngle(): Boolean {
        return AngleCalculations.wrapAroundAngles(desiredPosition.angle - gyro.getYaw()) < angleDeadzone || AngleCalculations.wrapAroundAngles(desiredPosition.angle - gyro.getYaw()) > 360 - angleDeadzone
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
