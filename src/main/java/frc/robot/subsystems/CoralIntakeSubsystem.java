// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;
import static frc.robot.Constants.HardwareID.CoralIntake.*;

import java.util.function.DoubleSupplier; 

public class CoralIntakeSubsystem extends SubsystemBase {

  TalonFX intakeRotationMotor;
  TalonFX frontWheelsMotor;
  TalonFX sideWheelsMotor;

  public double Extend = getPreference("extend", CoralIntakeConstants.Extend);
  public double Retract = getPreference("retract", CoralIntakeConstants.Retract);
  public double Intake = getPreference("intake", CoralIntakeConstants.Intake);
  public double Outtake = getPreference("outtake", CoralIntakeConstants.Outtake);

  /** Creates a new IntakeSubsystem. */
  public CoralIntakeSubsystem() {

    intakeRotationMotor = new TalonFX(ExtenderCanId);
    frontWheelsMotor = new TalonFX(TopWheelCanId);
    sideWheelsMotor = new TalonFX(LeftWheelsCanId);

    intakeRotationMotor.setNeutralMode(NeutralModeValue.Brake);
    frontWheelsMotor.setNeutralMode(NeutralModeValue.Brake);
    sideWheelsMotor.setNeutralMode(NeutralModeValue.Brake);

    //reconfigurePid();
    //reconfigureSetpoints();
  }

  public void rereadPreferences() {

      final var config = new Slot0Configs();
      config.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
      config.kG = getPreference("gravity", CoralIntakeConstants.kG);
      config.kS = getPreference("static", CoralIntakeConstants.kS);
      config.kP = getPreference("proportional", CoralIntakeConstants.kP);
      config.kI = getPreference("integral", CoralIntakeConstants.kI);
      config.kD = getPreference("derivative", CoralIntakeConstants.kD);
      intakeRotationMotor.getConfigurator().apply(config);

      Extend = getPreference("extend", CoralIntakeConstants.Extend);
      Retract = getPreference("retract", CoralIntakeConstants.Retract);
      Intake = getPreference("intake", CoralIntakeConstants.Intake);
      Outtake = getPreference("outtake", CoralIntakeConstants.Outtake);
  }

  public Command Rotate(double position) {

    return run(() -> {
      final PositionVoltage rotation_request = new PositionVoltage(position).withSlot(0);
      intakeRotationMotor.setControl(rotation_request);
    });
  }

  public Command Extend() {
    return run(() -> Rotate(Extend));
  }

  public Command Retract() {
    return run(() -> Rotate(Retract));
  }

  public Command SpinIntake(DoubleSupplier speed) {  

    return run(() -> {
      frontWheelsMotor.set(speed.getAsDouble());
      sideWheelsMotor.set(speed.getAsDouble());
    });
  }

  @Override
  public void periodic() {
    telemetry();
  }

  public void telemetry() {
      SmartDashboard.putNumber("Intake Motor Velocity", intakeRotationMotor.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Intake Motor Position", intakeRotationMotor.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Intake Motor Voltage", intakeRotationMotor.getMotorVoltage().getValueAsDouble());

      SmartDashboard.putNumber("Intake Motor Velocity", frontWheelsMotor.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Intake Motor Position", frontWheelsMotor.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Intake Motor Voltage", frontWheelsMotor.getMotorVoltage().getValueAsDouble());

      SmartDashboard.putNumber("Intake Motor Velocity", sideWheelsMotor.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Intake Motor Position", sideWheelsMotor.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Intake Motor Voltage", sideWheelsMotor.getMotorVoltage().getValueAsDouble());
  }

  public double getPreference(String key, double fallback) {
    return Preferences.getDouble("CoralIntake/" + key, fallback);
  }
}
