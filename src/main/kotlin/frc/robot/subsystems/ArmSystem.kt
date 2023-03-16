package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DutyCycleEncoder
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.MotorConstants

class ArmSystem() : SubsystemBase() {
    private enum class TraversalPosition {
        EXTENDED,
        RETRACTED,
        UNKNOWN
    }

    private val tiltSolenoids: Array<Solenoid> = arrayOf(Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.tiltSolenoid1), Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.tiltSolenoid2))
    private val armAngleMotor = CANSparkMax(MotorConstants.armAngleMotor, CANSparkMaxLowLevel.MotorType.kBrushed)
    private val traversalMotor = CANSparkMax(MotorConstants.traversalMotor, CANSparkMaxLowLevel.MotorType.kBrushed)

    private val armAngleEncoder = DutyCycleEncoder(MotorConstants.armAngleEncoder)
    private val traversalExtendedSwitch = DigitalInput(0)
    private val traversalRetractedSwitch = DigitalInput(1)

    private val armAngleDegrees
        get() = armAngleEncoder.get() * 360 % 360
    private var traversalPosition = TraversalPosition.EXTENDED
    private var lastKnownTraversalPosition = TraversalPosition.EXTENDED

    private val armAnglePID = ProfiledPIDController(0.01, 0.0, 0.0, TrapezoidProfile.Constraints(90.0, 10.0))

    var desiredTilt = false
    var desiredArmAngle = 45.0
        set(value) {
            armAnglePID.goal = TrapezoidProfile.State(value, 0.0)
            armAnglePID.reset(armAngleDegrees)
            field = value
        }
    var desiredTraversalVelocity = 0.0

    init {
        armAngleEncoder.distancePerRotation = 360.0
    }

    override fun periodic() {
        // traversalPosition = if (traversalExtendedSwitch.get()) {
        //     TraversalPosition.EXTENDED
        // } else if (traversalRetractedSwitch.get()) {
        //     TraversalPosition.RETRACTED
        // } else {
        //     TraversalPosition.UNKNOWN
        // }

        // if (traversalPosition != TraversalPosition.UNKNOWN) {
        //     lastKnownTraversalPosition = traversalPosition
        // }

        for (solenoid in tiltSolenoids) {
            solenoid.set(desiredTilt)
        }

        armAngleMotor.set(armAnglePID.calculate(armAngleDegrees))
        traversalMotor.set(desiredTraversalVelocity)
        desiredTraversalVelocity = 0.0
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
