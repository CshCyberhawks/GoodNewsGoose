package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel
import edu.wpi.first.wpilibj.PneumaticsModuleType
import edu.wpi.first.wpilibj.Solenoid
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.MotorConstants

class ClawSystem : SubsystemBase() {
    private val solenoid = Solenoid(MotorConstants.pcm, PneumaticsModuleType.CTREPCM, MotorConstants.clawSolenoid)
    private val motor = CANSparkMax(MotorConstants.clawMotor, CANSparkMaxLowLevel.MotorType.kBrushed)

    var desiredVelocity = 0.0
    var solenoidValue = false

    override fun periodic() {
        motor.set(desiredVelocity)
        solenoid.set(solenoidValue)
    }

    override fun simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
