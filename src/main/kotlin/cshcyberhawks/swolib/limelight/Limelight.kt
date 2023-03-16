package cshcyberhawks.swolib.limelight

import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.AngleCalculations
import cshcyberhawks.swolib.math.FieldPosition
import cshcyberhawks.swolib.math.Polar
import cshcyberhawks.swolib.math.Vector2
import cshcyberhawks.swolib.math.Vector3
import cshcyberhawks.swolib.swerve.SwerveOdometry
import edu.wpi.first.apriltag.AprilTag
import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.cscore.HttpCamera
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import java.util.Optional
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.tan

class Limelight(
    name: String,
    private val cameraHeight: Double,
    private val cameraAngle: Double = 0.0,
    private val cameraDistance: Double = 0.0,
    private val aprilTagHeight: Double = 0.0,
    ledMode: LedMode = LedMode.Pipeline,
    cameraMode: CameraMode = CameraMode.VisionProcessor,
    pipeline: Int = 0,
    streamMode: StreamMode = StreamMode.Standard,
    snapshotMode: SnapshotMode = SnapshotMode.Reset,
    crop: Array<Number> = arrayOf(0, 0, 0, 0),
    private val fiducialPipeline: Int = 0
) {
    private val limelight: NetworkTable
    private val tab: ShuffleboardTab = Shuffleboard.getTab("Limelight: $name")

    private val camName: String = name
    val feed: HttpCamera

    var pipeline: Int
        get() = limelight.getEntry("getpipe").getDouble(0.0).toInt()
        set(value) {
            if (value < 0 || value > 9) error("Invalid pipeline value")
            else limelight.getEntry("pipeline").setNumber(value)
        }
    //    companion object {
    //        public var viewTab: ShuffleboardTab = Shuffleboard.getTab("Limelight View")
    //        private var currentFeed: HttpCamera? = null
    //        private var server = CameraServer.addSwitchedCamera("LimeLight Feed")
    //        public var widget = viewTab.add("LLFeed", server.getSource())
    //
    //        fun openCamera(ll: Limelight, sizeX: Int = 3, sizeY: Int = 3) {
    //            val feed = ll.feed
    //            if (feed == currentFeed) return
    //            server.setSource(feed)
    //            currentFeed = feed
    //        }
    //    }

    fun putToTab(name: String, data: Optional<Double>) {
        if (!data.isEmpty) {
            tab.add(name, data.get())
        }
    }

    init {
        if (pipeline < 0 || pipeline > 9) error("Invalid pipeline value")
        else if (crop.size != 4) error("Invalid crop array")

        limelight = NetworkTableInstance.getDefault().getTable(name)
        limelight.getEntry("ledMode").setNumber(ledMode.ordinal)
        limelight.getEntry("camMode").setNumber(cameraMode.ordinal)
        limelight.getEntry("pipeline").setNumber(pipeline)
        limelight.getEntry("stream").setNumber(streamMode.ordinal)
        limelight.getEntry("snapshot").setNumber(snapshotMode.ordinal)
        limelight.getEntry("crop").setNumberArray(crop)

        tab.add("$name Has Target", this.hasTarget())
        putToTab("$name Horizontal Offset", this.getHorizontalOffset())
        putToTab("$name Vertical Offset", this.getVerticalOffset())
        putToTab("$name Area", this.getArea())
        tab.add("$name Current Pipeline", pipeline)
        tab.add("$name Target 3D", this.getTarget3D())
        tab.add("$name Cam Pose", this.getCamDebug())
        putToTab("$name Target ID", this.getTargetID())
        tab.add("$name Bot Pose", this.getBotDebug())

        feed = if (name == "limelight-front") {
            HttpCamera("Limelight Feed-Front", "http://10.28.75.11:5800")
        } else /* (name == "limelight-back") */ {
            HttpCamera("Limelight Feed-Back", "http://10.28.75.74:5800")
        }

//        tab.add("LLFeed $name", feed).withPosition(0, 0).withSize(8, 4)
//        PortForwarder.add(5800, "limelight.local", 5800)
    }

    /** @return Whether the limelight has any valid targets. */
    private fun hasTarget(): Boolean = limelight.getEntry("tv").getDouble(0.0) == 1.0

    /** @return Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees). */
    fun getHorizontalOffset(): Optional<Double> {
        val out = limelight.getEntry("tx").getDouble(Double.NaN)
        return if (!out.isNaN()) Optional.of(out) else Optional.empty()
    }
    /** @return Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
    fun getVerticalOffset(): Optional<Double> {
        val out = limelight.getEntry("ty").getDouble(Double.NaN)
        return if (!out.isNaN()) Optional.of(out) else Optional.empty()
    }
    
    /** @return Target Area (0% of image to 100% of image) */
    private fun getArea(): Optional<Double> {
        val out = limelight.getEntry("ta").getDouble(Double.NaN)
        return if (!out.isNaN()) Optional.of(out) else Optional.empty()
    }

    fun getLatency(): Optional<Double> {
        val out = limelight.getEntry("tl").getDouble(Double.NaN)
        return if (!out.isNaN()) Optional.of(out) else Optional.empty()
    }

    fun getShortest(): Optional<Double> {
        val out = limelight.getEntry("tshort").getDouble(Double.NaN)
        return if (!out.isNaN()) Optional.of(out) else Optional.empty()
    }

    fun getLongest(): Optional<Double> {
        val out = limelight.getEntry("tlong").getDouble(Double.NaN)
        return if (!out.isNaN()) Optional.of(out) else Optional.empty()
    }

    fun getHorizontalLength():Optional<Double> {
        val out = limelight.getEntry("thor").getDouble(Double.NaN)
        return if (!out.isNaN()) Optional.of(out) else Optional.empty()
    }

    fun getVerticalLength(): Optional<Double> {
        val out = limelight.getEntry("tvert").getDouble(Double.NaN)
        return if (!out.isNaN()) Optional.of(out) else Optional.empty()
    }

    fun getTarget3D(): Array<Number> = limelight.getEntry("camtran").getNumberArray(arrayOf<Number>())

    private fun getTargetID(): Optional<Double> {
        val out = limelight.getEntry("tid").getDouble(Double.NaN)
        return if (!out.isNaN()) Optional.of(out) else Optional.empty()
    }

    fun getJSON(): ByteArray = limelight.getEntry("json").getRaw(byteArrayOf())

    fun getCamPose(): Optional<FieldPosition> {
        val data = limelight.getEntry("campose").getDoubleArray(arrayOf())
        var pose: Pose3d? = null
        if (data.isNotEmpty()) {
            val translation = Translation3d(data[0], data[1], data[2])
            val rotation = Rotation3d(data[3], data[4], data[5])
            pose = Pose3d(translation, rotation)
        }
        if (pose != null) {
            val fieldPosition = FieldPosition(pose.x, pose.y, pose.rotation.z)
            return Optional.of(fieldPosition)
        } else {
            return Optional.empty()
        }
    }

    private fun getCamDebug(): Array<Double> {
        val data = limelight.getEntry("campose").getDoubleArray(arrayOf())
        var pose: Pose3d? = null
        if (data.isNotEmpty()) {
            val translation = Translation3d(data[0], data[1], data[2])
            val rotation = Rotation3d(data[3], data[4], data[5])
            pose = Pose3d(translation, rotation)
        }
        if (pose == null) {
            return arrayOf(0.0, 0.0, 0.0)
        }
        return arrayOf(pose.x, pose.y, pose.rotation.z)
    }

    fun getBotPosition(): Optional<Vector3> {
        val data = limelight.getEntry("botpose").getDoubleArray(arrayOf())
        if (!hasTarget() || data[0] == 0.000 || pipeline != fiducialPipeline) {
            return Optional.empty()
        }
        return Optional.of(Vector3(data[0], data[1], data[2]))
    }

    fun getBotYaw(): Optional<Double> {
        val data = limelight.getEntry("botpose").getDoubleArray(arrayOf())
        if (!hasTarget() || data[0] == 0.000 || pipeline != fiducialPipeline) {
            return Optional.empty()
        }
        return Optional.of(data[5])
    }

    /** @return uses targetDistance() to "more accurately" estimate bot yaw
     * (accounts for camera distance from bot center)*/
    fun getBotYawClose(): Optional<Double> {
        val limelightRotation = getHorizontalOffset()
        val targetDistance = findTargetDistance(aprilTagHeight)
        return if (limelightRotation.isPresent)
            Optional.of(
                atan2(
                    (targetDistance.get() * sin(-limelightRotation.get())),
                            (cameraDistance + targetDistance.get() * cos(-limelightRotation.get())))
            )
        else Optional.empty()
    }

    private fun getBotDebug(): Array<Double> {
        val data = limelight.getEntry("botpose").getDoubleArray(arrayOf())
        if (data.isEmpty()) {
            return arrayOf()
        }
        return arrayOf(data[0], data[1], data[2])
    }

    fun getDetectorClass(): Optional<Double> {
        val out = limelight.getEntry("tclass").getDouble(Double.NaN)
        return if (out.isNaN()) Optional.empty() else Optional.of(out)
    }

    fun getColorUnderCrosshair(): Array<Number> =
        limelight.getEntry("tc").getNumberArray(arrayOf<Number>())

    /** @return Distance from target (meters). */
    private fun findTargetDistance(ballHeight: Double): Optional<Double> {
        val vOffset = getVerticalOffset()
        return if (vOffset.isEmpty || !hasTarget()) Optional.empty() else Optional.of(
            (abs(cameraHeight - ballHeight)) / tan(
                Math.toRadians(vOffset.get() + cameraAngle)
            )
        )
    }

    fun getColor(): Array<Number> = limelight.getEntry("tc").getNumberArray(arrayOf())

    fun getPosition(swo: SwerveOdometry, ballHeight: Double, gyro: GenericGyro): Optional<Vector2> {
        val optDistance = findTargetDistance(ballHeight) // .639
        if (optDistance.isEmpty) {
            return Optional.empty()
        }
        val distance = optDistance.get()
        SmartDashboard.putNumber("Limelight Distance", distance)
        val angle: Double =
            AngleCalculations.wrapAroundAngles(getHorizontalOffset().get() + gyro.getYaw()) // 357

        return Optional.of(Vector2.fromPolar(Polar(angle, distance)) + Vector2(swo.fieldPosition.x, swo.fieldPosition.y))
    }
}
