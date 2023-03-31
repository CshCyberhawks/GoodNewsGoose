package frc.robot.constants

object MotorConstants {
    // TalonSRX Motors
    const val frontRightTurnMotor: Int = 8
    const val frontLeftTurnMotor: Int = 7
    const val backRightTurnMotor: Int = 9
    const val backLeftTurnMotor: Int = 6

    // Falcon Motors
    const val frontRightDriveMotor: Int = 4
    const val frontLeftDriveMotor: Int = 3
    const val backRightDriveMotor: Int = 5
    const val backLeftDriveMotor: Int = 2

    // Encoders
    const val frontRightEncoder = 12
    const val frontLeftEncoder = 11
    const val backRightEncoder = 13
    const val backLeftEncoder = 10

    val turnEncoderOffsets: Array<Double> = arrayOf(339.9609375, 246.796875, 311.1328125, 23.291015625)

    const val armAngleMotor = 22
    const val extensionMotor = 21
    const val tiltSolenoid = 2
    const val grabberSolenoid = 0
    const val brakeSoleniod = 1
    const val armAngleEncoder = 1
    const val extensionEncoder = 0
    const val extensionExtendedSwitch = 2
    const val extensionRetractedSwitch = 3
    const val gyroPort = 30
    const val pcm = 31

    const val clawSolenoid = 40
    const val clawMotor = 41
}
