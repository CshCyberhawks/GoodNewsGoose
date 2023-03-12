package cshcyberhawks.swolib.limelight

import cshcyberhawks.swolib.hardware.interfaces.GenericGyro
import cshcyberhawks.swolib.math.*
import cshcyberhawks.swolib.swerve.SwerveOdometry
import edu.wpi.first.cscore.HttpCamera
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import java.util.*
import kotlin.math.tan

class Limelight(
        name: String,
        private val cameraHeight: Double,
        private val cameraAngle: Double,
        ledMode: LedMode = LedMode.Pipeline,
        cameraMode: CameraMode = CameraMode.VisionProcessor,
        pipeline: Int = 0,
        streamMode: StreamMode = StreamMode.Standard,
        snapshotMode: SnapshotMode = SnapshotMode.Reset,
        crop: Array<Number> = arrayOf(0, 0, 0, 0)
) {
    private val limelight: NetworkTable
    private val tab: ShuffleboardTab
    private val camName: String = name
    val feed: HttpCamera

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

        tab = Shuffleboard.getTab("Limelight: $name")
        tab.add("$name Has Target", this.hasTarget())
        tab.add("$name Horizontal Offset", this.getHorizontalOffset())
        tab.add("$name Vertical Offset", this.getVerticalOffset())
        tab.add("$name Area", this.getArea())
        tab.add("$name Rotation", this.getRotation())
        tab.add("$name Current Pipeline", this.getCurrentPipeline())
        tab.add("$name Target 3D", this.getTarget3D())
        tab.add("$name Target ID", this.getTargetID())
        tab.add("$name Cam Pose", this.getCamDebug())
        tab.add("$name Bot Pose", this.getBotDebug())

        val ip: String
        if (camName == "limelight-front") {
            ip = "http://10.28.75.11:5800"
        } else /* (name == "limelight-back") */ {
            ip = "http://10.28.75.13:5800"
        }

        feed = HttpCamera(camName, ip)
    }
    /** @return Whether the limelight has any valid targets. */
    private fun hasTarget(): Boolean = limelight.getEntry("tv").getDouble(0.0) == 1.0

    /** @return Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees). */
    fun getHorizontalOffset(): Double = limelight.getEntry("tx").getDouble(0.0)

    /** @return Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
    private fun getVerticalOffset(): Double = limelight.getEntry("ty").getDouble(0.0)

    /** @return Target Area (0% of image to 100% of image) */
    private fun getArea(): Double = limelight.getEntry("ta").getDouble(0.0)

    private fun getRotation(): Double = limelight.getEntry("ts").getDouble(0.0)

    fun getLatency(): Double = limelight.getEntry("tl").getDouble(0.0)

    fun getShortest(): Double = limelight.getEntry("tshort").getDouble(0.0)

    fun getLongest(): Double = limelight.getEntry("tlong").getDouble(0.0)

    fun getHorizontalLength(): Double = limelight.getEntry("thor").getDouble(0.0)

    fun getVerticalLength(): Double = limelight.getEntry("tvert").getDouble(0.0)

    fun getCurrentPipeline(): Int = limelight.getEntry("getpipe").getDouble(0.0).toInt()

    private fun getTarget3D(): Array<Number> =
            limelight.getEntry("camtran").getNumberArray(arrayOf<Number>())

    private fun getTargetID(): Double = limelight.getEntry("tid").getDouble(0.0)

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
    fun getBotPose(): Vector3 {
        val data = limelight.getEntry("botpose").getDoubleArray(arrayOf())
        if (data.isEmpty()) {
            return Vector3(0.0, 0.0, 0.0)
        }
        return Vector3(data[0], data[1], data[2])
    }
    private fun getBotDebug(): Array<Double> {
        val data = limelight.getEntry("botpose").getDoubleArray(arrayOf())
        if (data.isEmpty()) {
            return arrayOf(0.0, 0.0, 0.0)
        }
        return arrayOf(data[0], data[1], data[2])
    }
    fun getDetectorClass(): Double = limelight.getEntry("tclass").getDouble(0.0)

    fun getColorUnderCrosshair(): Array<Number> =
            limelight.getEntry("tc").getNumberArray(arrayOf<Number>())

    /** @return Distance from target (meters). */
    private fun findTargetDistance(ballHeight: Double): Double =
            if (hasTarget())
                    (cameraHeight - ballHeight) *
                            tan(Math.toRadians(getVerticalOffset() + cameraAngle))
            else -1.0

    fun getColor(): Array<Number> = limelight.getEntry("tc").getNumberArray(arrayOf(-1))

    fun getPosition(swo: SwerveOdometry, ballHeight: Double, gyro: GenericGyro): Vector2 {
        val distance: Double = findTargetDistance(ballHeight) // .639
        val angle: Double =
                AngleCalculations.wrapAroundAngles(getHorizontalOffset() + gyro.getYaw()) // 357

        var ret = Vector2.fromPolar(Polar(angle, distance))
        ret.y = -ret.y
        ret += Vector2(swo.fieldPosition.x, swo.fieldPosition.y)

        return ret
    }
    fun setPipeline(pipeline: Int) {
        if (pipeline < 0 || pipeline > 9) error("Invalid pipeline value")
        else limelight.getEntry("pipeline").setNumber(pipeline)
    }
}
