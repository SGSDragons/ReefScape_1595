package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeSubsystemFake extends SubsystemBase{

  public void reconfigurePid() {}

  public void reconfigureSetpoints() {}

  public Command Rotate(double position) { return run(() -> {}); }

  public Command Extend() {return run(() -> {}); }

  public Command Retract() { return run(() -> {}); }

  public void Intake() {}

  public void Outtake() {}

  @Override
  public void periodic() { telemetry(); }

  public void telemetry() {}
}

