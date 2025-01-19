package frc.robot.swerve;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants;

public final class SwerveConstants {
    public static final COTSTalonFXSwerveConstants chosenModule =
            COTSTalonFXSwerveConstants.SDS.MK4.Falcon500(COTSTalonFXSwerveConstants.SDS.MK4.driveRatios.L1);

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(21.5);
    public static final double wheelBase = Units.inchesToMeters(18.5);
    public static final double wheelCircumference = chosenModule.wheelCircumference;

    /* Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final double driveBaseRadius = new Translation2d(wheelBase / 2.0, trackWidth / 2.0).getNorm();

    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;

    /* Motor Inverts */
    public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
    public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

    // Swerve Current Limiting
    // See https://v6.docs.ctr-electronics.com/en/2024/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;

    public static final int driveCurrentLimit = 35;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;

    /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
     * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    /* Angle Motor PID Values */
    //public static final double angleKP = chosenModule.angleKP;
    public static final double angleKP = Preferences.getDouble(Constants.Keys.angle_kPKey, 100.0);
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;

    /* Drive Motor PID Values */
    //public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
    public static double driveKP = Preferences.getDouble(Constants.Keys.drive_kPKey, 0.12);

    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;

    /* Drive Motor Characterization Values From SYSID */
    public static double driveKS = Preferences.getDouble(Constants.Keys.drive_kSKey, 0.32);
    public static double driveKV = Preferences.getDouble(Constants.Keys.drive_kVKey, 1.51);
    public static double driveKA = Preferences.getDouble(Constants.Keys.drive_kAKey, 0.27);

    /* Swerve Profiling Values */
    /** Meters per Second */
    //public static final double maxSpeed = 3.0; //TODO: This must be tuned to specific robot

    /**
     * Radians per Second
     */
    //public static final double maxAngularVelocity = 8.0; //TODO: This must be tuned to specific robot

    public static double maxSpeed = Preferences.getDouble(Constants.Keys.maxSpeedKey, 4.17);
    public static double maxAngularVelocity = Preferences.getDouble(Constants.Keys.maxAngularVelocityKey, 29.65);

    /* Neutral Modes */
    public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

    /*
      Constants specific to each wheel module.
      Identifiers can be assigned and verified in the Phoenix tuning software
      Google "talonfx tuning software" to find it and follow its documentation.
      In brief, the tuning software connects through the driver station software to the
      roboRIO. It should automatically detect and report all devices on the CAN bus.

      (cb) A "record" is just a java class that's meant to hold data. It's much
      easier to define, because you don't have to declare all the member variables
      and make a constructor that accepts all the variables then assigns all those
      constructor arguments to the member variables
     */
    public record Module(
            // The CAN device identifier for the motor that drives a wheel
            int driveMotorID,
            // The CAN device identifier for the motor that steers a wheel
            int angleMotorID,
            // The CAN device identifier for the encoder that reads the wheel's direction. Its more reliable than the encoder on the angleMotor
            int canCoderID,

            // The canCoder's zero position will not point forward. This offset corrects
            // for any error by subtracting the coder's position from the wheel's position
            // when pointing forward.
            // TODO: Make a tool that simplifies getting these measurements so they can
            // be fixed during a match. Perhaps pull these from preferences, too. Then
            // they can be updated directly from the SmartDashboard UI without rebuilding code
            Rotation2d angleOffset
    ) {}

    /* Front Left Module - Module 0 */
    public static final Module mod0 = new Module(
        3,
        4,
        5,
        Rotation2d.fromDegrees(-101.601563)
    );

    /* Front Right Module - Module 1 */
    public static final Module mod1 = new Module(
        6,
        7,
        8,
        Rotation2d.fromDegrees(-58.095703)
    );

    /* Back Left Module - Module 2 */
    public static final Module mod2 = new Module(
        9,
        10,
        11,
        Rotation2d.fromDegrees(48.603516)
    );

    /* Back Right Module - Module 3 */
    public static final Module mod3 = new Module(
        12,
        13,
        14,
        Rotation2d.fromDegrees(-153.632813)
    );
}
