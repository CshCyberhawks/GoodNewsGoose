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

    val turnEncoderOffsets: Array<Double> = arrayOf(333.28125, 245.302734375, 308.671875, 18.544921875)

    const val armAngleMotor = 20
    const val traversalMotor = 21
    const val tiltSolenoid1 = 0
    const val tiltSolenoid2 = 1
    const val grabberSolenoid = 2
    const val armAngleEncoder = 0
    const val traversalExtendedSwitch = 1
    const val traversalRetractedSwitch = 2

    const val pcm = 31
}
