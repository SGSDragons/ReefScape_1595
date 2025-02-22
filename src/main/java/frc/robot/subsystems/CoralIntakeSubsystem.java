// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralIntakeSubsystem extends SubsystemBase {

  TalonFX intakeRotationMotor;
  TalonFX frontWheelsMotor;
  TalonFX sideWheelsMotor;

  /** Creates a new IntakeSubsystem. */
  public CoralIntakeSubsystem() {

//    intakeRotationMotor = new TalonFX(Constants.IntakeConstants.intakeRotationMotorCanId);
//    frontWheelsMotor = new TalonFX(Constants.IntakeConstants.frontWheelsMotorCanId);
//    sideWheelsMotor = new TalonFX(IntakeConstants.sideWheelsMotorCanId);
//
//    intakeRotationMotor.setNeutralMode(NeutralModeValue.Brake);
//
//    frontWheelsMotor.setControl(new Follower(Constants.IntakeConstants.frontWheelsMotorCanId,...));

//    sideWheelsMotor.setControl(...);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend() {
//    var config = new Slot0Configs();
//    config.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
//    config.kS = getPreference("4bar/static", 0.0);
//    config.kP = getPreference("4bar/proportional", 0.0);
//    config.kD = getPreference("4bar/derivative", 0.0);
//    intakeRotationMotor.getConfigurator().apply(config);
//
//    double extendedPosition = getPreference("extendedPosition", 0.0);
//    PositionVoltage target = new PositionVoltage(extendedPosition);
//    intakeRotationMotor.setControl(target);
  }

  public double getPreference(String key, double fallback) {
    return Preferences.getDouble("intake/" + key, fallback);
  }
}
