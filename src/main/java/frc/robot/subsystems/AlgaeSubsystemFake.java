package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeContants;

public class AlgaeSubsystemFake extends SubsystemBase{
public void reconfigurePid() {}

  public Command rotate(DoubleSupplier speed) {return run(() -> {}); }

  public void spinRoller(DoubleSupplier speed) {}

  public Command Roller(DoubleSupplier speed){ return run(() -> {}); }

  public void stop() {}

  private Command setArmPosition(final double target, DoubleSupplier speed) { return run(() -> {}); }

  public Command Extend(DoubleSupplier speed) { return run(() -> {}); }

  public Command Retract(DoubleSupplier speed) { return run(() -> {}); }

  @Override
  public void periodic() { telemetry(); }

  public void telemetry() {}

  public double getPreference(String key, double fallback) { return 0; }
}
