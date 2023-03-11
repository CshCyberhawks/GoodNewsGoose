package cshcyberhawks.swolib.swerve

import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.limelight.Limelight
import cshcyberhawks.swolib.math.MiscCalculations
import cshcyberhawks.swolib.math.Polar
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.math.Vector3
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.smartdashboard.Field2d
import kotlin.math.cos
import kotlin.math.sin

class SwerveOdometry(
    private var swerveDriveTrain: SwerveDriveTrain,
    private var gyro: GenericGyro,
    private val swoToMeters: Double,
    private val startingPosition: Vector3 = Vector3(0.0, 0.0, 0.0),
    private val limelight: Limelight? = null,
    private val debugLogging: Boolean = false
) {
    var fieldPosition = Vector3() + startingPosition
    var lastTime = MiscCalculations.getCurrentTime()

    public val field2d = Field2d()


    val odometryShuffleTab = Shuffleboard.getTab("Odometry")
    val xPosition = odometryShuffleTab.add("X Position", 0.0).withPosition(0, 0).withSize(2, 1).entry
    val yPosition = odometryShuffleTab.add("Y Position", 0.0).withPosition(2, 0).withSize(2, 1).entry

    fun getVelocity(): Vector3 {
        var total = Vector2()

        val wheelVectors = swerveDriveTrain.swerveConfiguration.getWheelVectors()
        for (wheel in wheelVectors) {
            val wheelVector = Vector2.fromPolar(wheel)
            total += wheelVector
        }
        total /= wheelVectors.size

        val polar = Polar.fromVector2(total)
        polar.theta += gyro.getYaw()
        total = Vector2.fromPolar(polar)

        // Pitch and roll might be flipped
        val x = total.x * cos(Math.toRadians(gyro.getPitch())) / swoToMeters
        val y = total.y * cos(Math.toRadians(gyro.getRoll())) / swoToMeters
        val z =
            (total.x * sin(Math.toRadians(gyro.getPitch())) +
                    total.y * sin(Math.toRadians(gyro.getRoll()))) / swoToMeters
//        return Vector3(x, y, z)
        return Vector3(total.x, total.y, 0.0)
    }

    fun updatePosition() {
        fieldPosition += getVelocity() * (MiscCalculations.getCurrentTime() - lastTime)

//        if (limelight != null) {
//            val limelightPosition = limelight.getBotPose()
//            if (limelightPosition != null) {
//                fieldPosition = (limelightPosition + startingPosition)
//            }
//        }

        if (debugLogging) {
            xPosition.setDouble(fieldPosition.x)
            yPosition.setDouble(fieldPosition.y)
        }

        updateField()
        lastTime = MiscCalculations.getCurrentTime()
    }

    fun updateField() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            field2d.setRobotPose(this.fieldPosition.y, -this.fieldPosition.x, gyro.getYawRotation2d())
        } else {
            // Random constants are offsets for the field origin being bottom right in our code
            field2d.setRobotPose(16.54 - this.fieldPosition.y, this.fieldPosition.x, gyro.getYawRotation2d().plus(
                Rotation2d(Math.PI)
            ))
        }
    }
}
