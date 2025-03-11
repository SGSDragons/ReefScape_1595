// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeContants;
import frc.robot.Constants.CoralIntakeConstants;
import static frc.robot.Constants.HardwareID.Algae.*;

import java.util.function.DoubleSupplier; 

public class AlgaeSubsystem extends SubsystemBase {

  TalonFX FourBarMotor;
  SparkMax AlgaeSpin;

  public double Extend = getPreference("extend", AlgaeContants.Extend);
  public double Retract = getPreference("retract", AlgaeContants.Retract);

  public AlgaeSubsystem() {

    FourBarMotor = new TalonFX(FourBarCanId);
    FourBarMotor.setNeutralMode(NeutralModeValue.Brake);

    reconfigurePid();
  }

  public void reconfigurePid() {

      final var config = new Slot0Configs();
      config.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
      config.kG = getPreference("gravity", AlgaeContants.kG);
      config.kS = getPreference("static", AlgaeContants.kS);
      config.kP = getPreference("proportional", AlgaeContants.kP);
      config.kI = getPreference("integral", AlgaeContants.kI);
      config.kD = getPreference("derivative", AlgaeContants.kD);

      FourBarMotor.getConfigurator().apply(config);
  }

  public Command rotate(DoubleSupplier speed) {
    return run(() -> { 
        FourBarMotor.set(speed.getAsDouble()/5);
        //stop();
    });
  }

  public Command spin(DoubleSupplier speed){
    return run(() -> { 
      AlgaeSpin.set(speed.getAsDouble());
      //stop();
  });
  }

  public void stop() {
    var talonFXConfigurator = FourBarMotor.getConfigurator();
    var limitConfigs = new CurrentLimitsConfigs();

    FourBarMotor.getSupplyVoltage();
    FourBarMotor.getMotorVoltage();

    // enable stator current limit
    //limitConfigs.StatorCurrentLimit = getPreference("currentlimit", AlgaeContants.CurrentLimit);
    limitConfigs.StatorCurrentLimit = AlgaeContants.CurrentLimit;
    limitConfigs.StatorCurrentLimitEnable = true;

    talonFXConfigurator.apply(limitConfigs);
  }

  public Command Extend() {

    return run(() -> {

      final PositionVoltage rotation_request = new PositionVoltage(Extend).withSlot(0);
      FourBarMotor.setControl(rotation_request);

    });
  }

  public Command Retract() {

    return run(() -> {

      final PositionVoltage rotation_request = new PositionVoltage(Retract).withSlot(0);
      FourBarMotor.setControl(rotation_request);

    });
  }

  @Override
  public void periodic() {
    telemetry();
  }

  public void telemetry() {
      SmartDashboard.putNumber("Algae Motor Velocity", FourBarMotor.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Algae Motor Position", FourBarMotor.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("Algae Motor Voltage", FourBarMotor.getMotorVoltage().getValueAsDouble());
  }

  public double getPreference(String key, double fallback) {
    return Preferences.getDouble("Algae/" + key, fallback);
  }
}
