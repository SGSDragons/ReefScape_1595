package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CarriageConstants;
import frc.robot.subsystems.LiftSubsystem.LiftPosition;

public class CarriageSubsystemFake extends SubsystemBase{

    public void rereadPreferences() {}

    public Command spin() { return run(() -> {}); }

    public Command pointMiddle() { return run(() -> {}); }

    public Command pointRight() { return run(() -> {}); }
    
    public Command pointLeft() { return run(() -> {}); }

    public Command shootLeft(DoubleSupplier speed) { return run(() -> {}); }

    public Command shootRight(DoubleSupplier speed) { return run(() -> {}); }

    public Command Middle(DoubleSupplier speed) { return run(() -> {}); }

    @Override
    public void periodic(){ telemetry(); }

    public void telemetry(){ }

    private static double getPreference(String key, double fallback) { return 0; }
}
