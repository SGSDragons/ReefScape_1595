package frc.robot.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = SwerveConstants.cancoderInvert;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = SwerveConstants.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = SwerveConstants.angleNeutralMode;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.angleGearRatio;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(SwerveConstants.angleEnableCurrentLimit)
                .withSupplyCurrentLimit(SwerveConstants.angleCurrentLimit)
                .withSupplyCurrentLowerLimit(SwerveConstants.angleCurrentThreshold)
                .withSupplyCurrentLowerTime(SwerveConstants.angleCurrentThresholdTime);

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = SwerveConstants.angleKP;
        swerveAngleFXConfig.Slot0.kI = SwerveConstants.angleKI;
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.angleKD;

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = SwerveConstants.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = SwerveConstants.driveNeutralMode;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.driveGearRatio;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits
                .withSupplyCurrentLimitEnable(SwerveConstants.driveEnableCurrentLimit)
                .withSupplyCurrentLimit(SwerveConstants.driveCurrentLimit)
                .withSupplyCurrentLowerLimit(SwerveConstants.driveCurrentThreshold)
                .withSupplyCurrentLowerTime(SwerveConstants.driveCurrentThresholdTime);

        /* PID Config */  
        swerveDriveFXConfig.Slot0.kP = SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = SwerveConstants.driveKD;

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = SwerveConstants.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = SwerveConstants.openLoopRamp;

        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = SwerveConstants.closedLoopRamp;
    }
}