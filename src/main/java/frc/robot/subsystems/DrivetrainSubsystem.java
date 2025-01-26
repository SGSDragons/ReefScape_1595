// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.studica.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.swerve.SwerveConstants;
import frc.robot.swerve.SwerveModule;

import java.util.List;

public class DrivetrainSubsystem extends SubsystemBase {
  public SwerveDriveOdometry swerveDriveOdometry;
  public List<SwerveModule> swerveModules;
  public SwerveDrivePoseEstimator poseEstimator;

  // The Gyroscope (Measures Yaw/Rotation Angle)
  private final AHRS navx;
  private double highestMeasuredVelocity = 0;

  private SysIdRoutine sysIdRoutine;

  public DrivetrainSubsystem() {

    navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
    navx.reset();


    swerveModules = List.of(
      new SwerveModule(0, SwerveConstants.mod0),
      new SwerveModule(1, SwerveConstants.mod1),
      new SwerveModule(2, SwerveConstants.mod2),
      new SwerveModule(3, SwerveConstants.mod3)
    );
    
    swerveDriveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());

    poseEstimator = new SwerveDrivePoseEstimator(
            SwerveConstants.swerveKinematics,
            navx.getRotation2d(),
            getModulePositions(),
            new Pose2d()
    );
  }

  public void reset() {
    drive(new Translation2d(0.0, 0.0), 0.0, true, true);
  }

  public void drive(Translation2d translation, double anglularVelocity, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
      SwerveConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(),
                translation.getY(),
                anglularVelocity,
                getHeading()
              )
              : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                anglularVelocity)
              );
      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed());

      for (SwerveModule module : swerveModules) {
        module.setDesiredState(swerveModuleStates[module.moduleNumber], isOpenLoop);
      }
  }

//  public void drive (ChassisSpeeds chassisSpeeds) {
//    SwerveModuleState[] swerveModuleStates =
//      SwerveConstants.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
//    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxSpeed());
//
//    for (SwerveModule module : swerveModules) {
//      module.setDesiredState(swerveModuleStates[module.moduleNumber], false);
//    }
//  }
//
  // Used by SwerveControllerCommand in Auto
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxSpeed());

    for (SwerveModule module : swerveModules) {
      module.setDesiredState(desiredStates[module.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for(SwerveModule module : swerveModules) {
      states[module.moduleNumber] = module.getState();
    }

    return states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule module : swerveModules) {
      positions[module.moduleNumber] = module.getPosition();
    }

    return positions;
  }

  public Pose2d getPose() {
    return swerveDriveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading) {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public void zeroHeading() {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
  }

  public void resetHeading(Pose2d pose) {
    swerveDriveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  public Rotation2d getGyroYaw() {
    return Rotation2d.fromDegrees(-navx.getAngle()); //Negated because NavX CW Positive while ChassisSpeeds requires CCW Positive
  }

  private void telemetry() {
    for(SwerveModule module : swerveModules){
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Angle", module.getAngle().getDegrees());
            SmartDashboard.putNumber("Module " + module.moduleNumber + " Velocity", module.getState().speedMetersPerSecond);
            if (module.getState().speedMetersPerSecond > highestMeasuredVelocity) {
              highestMeasuredVelocity = module.getState().speedMetersPerSecond;
            }
          }     
    SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    SmartDashboard.putNumber("MaximumSpeed", highestMeasuredVelocity);
    SmartDashboard.putNumber("NavX", -navx.getAngle()); //CCW Positive
  }

  // Makes System Identification Routine for Mathematical Analysis. Runs two
  // tests that apply specific voltages to motors and logs their positions and
  // velocities.
  public SysIdRoutine createSystemIdentificationRoutine() {
    // System Identification
    // See https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
    // In brief, the System Identification routine applies voltages to the drive
    // wheels and measures their reactions. The goal is to estimate the ideal
    // coefficients (kS, kV and kA) for the DC Motor Feed Forward equation to
    // make the system respond best to control system inputs. Once analyzed, the
    // results should be saved to SwerveConstants.driveKS, driveKV and driveKA

    SysIdRoutine.Config config = new SysIdRoutine.Config(
        Units.Volts.per(Units.Second).of(0.75),
        Units.Volts.of(0.75),
        Units.Seconds.of(5));

    SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
        volts -> swerveModules.forEach(m -> m.setDriveVoltage(volts)),
        log -> swerveModules.forEach(m -> m.logStatus(log)),
        this);

    return new SysIdRoutine(config, mechanism);
  }

  // Drive System Identification Commands for Testing and Analysis
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return createSystemIdentificationRoutine().quasistatic(direction);
  }
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return createSystemIdentificationRoutine().dynamic(direction);
  }

  @Override
  public void periodic() {
    swerveDriveOdometry.update(getGyroYaw(), getModulePositions());
    poseEstimator.update(getGyroYaw(), getModulePositions());

    LimelightHelpers.SetRobotOrientation("limelight", -navx.getAngle(), 0.0, 0.0, 0.0, 0.0, 0.0);
    LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (estimate != null) {
      poseEstimator.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
    }
    telemetry();
  }

  @Override
  public void simulationPeriodic() {
  }

  public void recalibrateCANCoders() {
    for (SwerveModule module : swerveModules) {
      module.recalibrateCANCoder();
    }
  }
}
