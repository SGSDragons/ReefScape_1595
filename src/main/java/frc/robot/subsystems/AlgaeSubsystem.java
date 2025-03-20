// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeContants;
import static frc.robot.Constants.HardwareID.Algae.*;

import java.util.function.DoubleSupplier; 

public class AlgaeSubsystem extends SubsystemBase {

  TalonFX FourBarMotor;
  SparkMax Roller;
  RelativeEncoder RollerEncoder;

  public double Extend = getPreference("extend", AlgaeContants.Extend);
  public double Retract = getPreference("retract", AlgaeContants.Retract);

  public AlgaeSubsystem() {

    FourBarMotor = new TalonFX(FourBarCanId);
    FourBarMotor.setNeutralMode(NeutralModeValue.Brake);
    FourBarMotor.setPosition(0.0);

    Roller = new SparkMax(RollerCanId, MotorType.kBrushless);
    RollerEncoder = Roller.getEncoder();

    reconfigurePid();
  }

  public void reconfigurePid() {

      final var config = new Slot0Configs();
      config.kG = getPreference("gravity", AlgaeContants.kG);
      config.kS = getPreference("static", AlgaeContants.kS);
      config.kP = getPreference("proportional", AlgaeContants.kP);
      config.kI = getPreference("integral", AlgaeContants.kI);
      config.kD = getPreference("derivative", AlgaeContants.kD);

      FourBarMotor.getConfigurator().apply(config);
  }

  public Command rotate(DoubleSupplier speed) {
    return run(() -> { 
        SmartDashboard.putNumber("Algae Target Velocity", speed.getAsDouble() * getPreference("speed", 0.5));
        FourBarMotor.set(speed.getAsDouble() * getPreference("speed", 0.5));
    });
  }

  public void spinRoller(DoubleSupplier speed){
    Roller.set(speed.getAsDouble()/2);
  }

  public Command Roller(DoubleSupplier speed){
    return run(() -> spinRoller(speed));
  }

  public void stop() {
    // var talonFXConfigurator = FourBarMotor.getConfigurator();
    // var limitConfigs = new CurrentLimitsConfigs();

    // FourBarMotor.getSupplyVoltage();
    // FourBarMotor.getMotorVoltage();

    // // enable stator current limit
    // //limitConfigs.StatorCurrentLimit = getPreference("currentlimit", AlgaeContants.CurrentLimit);
    // limitConfigs.StatorCurrentLimit = AlgaeContants.CurrentLimit;
    // limitConfigs.StatorCurrentLimitEnable = true;

    // talonFXConfigurator.apply(limitConfigs);
  }

  private Command setArmPosition(final double target, DoubleSupplier speed) {
    // run the arm to a position. stop the command when within 0.2 rotations, which
    // kills the power and sets the motor to brake.
    final PositionVoltage request = new PositionVoltage(target).withSlot(0);
    return new FunctionalCommand(
      () -> {FourBarMotor.setControl(request);}, 
      () -> {spinRoller(speed);},
      (interrupted) -> {FourBarMotor.set(0.0);},
      () -> 0.2 >  Math.abs(FourBarMotor.getPosition().getValueAsDouble() - target),
      this
    );
  }

  public Command Extend(DoubleSupplier speed) {
    return setArmPosition(Extend,speed);
  }

  public Command Retract(DoubleSupplier speed) {
    return setArmPosition(0.0,speed);
  }

  @Override
  public void periodic() {
    telemetry();
  }

  public void telemetry() {
      SmartDashboard.putNumber("FourBar Motor Velocity", FourBarMotor.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("FourBar Motor Position", FourBarMotor.getPosition().getValueAsDouble());
      SmartDashboard.putNumber("FourBar Motor Voltage", FourBarMotor.getMotorVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Roller Velocity", RollerEncoder.getVelocity());
  }

  public double getPreference(String key, double fallback) {
    return Preferences.getDouble("Algae/" + key, fallback);
  }
}
