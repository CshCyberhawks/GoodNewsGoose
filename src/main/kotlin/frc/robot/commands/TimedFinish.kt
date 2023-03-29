import edu.wpi.first.util.WPIUtilJNI
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.CommandBase

class TimedFinish(val time: Double = 1.0) : CommandBase() {

    var startTime = 0.0

    override fun initialize() {
        startTime = WPIUtilJNI.now() * 1.0e-6
    }

    override fun execute() {}

    override fun end(interrupted: Boolean) {}

    override fun isFinished(): Boolean {
        SmartDashboard.putNumber("Time now", WPIUtilJNI.now() * 1.0e-6)
        SmartDashboard.putNumber("Start Time", startTime)
        return WPIUtilJNI.now() * 1.0e-6 - startTime > time
    }
}
