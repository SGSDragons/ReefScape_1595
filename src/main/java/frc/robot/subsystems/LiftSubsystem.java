package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LiftConstants;
import frc.robot.PreferenceSet;

public class LiftSubsystem extends SubsystemBase {

    final static PreferenceSet preferences = new PreferenceSet("Lift");

    public static class LiftPosition {
        
        double setPoint;
        String name;

        public LiftPosition(String name, double fallback) {
            this.setPoint = preferences.get(name, fallback);
            this.name = name;
        }

        public void adjust(double shift) {
            setPoint += shift;
            preferences.set(name, setPoint);
        }
    }

    public static final LiftPosition Intake = new LiftPosition("Intake", LiftConstants.intakeheight);
    public static final LiftPosition Shelf = new LiftPosition("Shelf", LiftConstants.Shelf);
    public static final LiftPosition Low  = new LiftPosition("Low", LiftConstants.Low);
    public static final LiftPosition Medium = new LiftPosition("Medium", LiftConstants.Medium);
    public static final LiftPosition High = new LiftPosition("High", LiftConstants.High);

    public void rereadPreferences() {}

    // Returns one of the position instances if the lift is "close"
    // Returns null if it's not close to any position
    LiftPosition getPosition() { return Intake; }

    Command rotateDown() { return run(() -> {}); }

    Command rotateUp() { return run(() -> {}); }

    public Command gotoPosition(LiftPosition position, DoubleSupplier axis) { return run(() -> {}); }

    public Command gotoGround() { return run(() -> {}); }

    public Command move(DoubleSupplier axis) { return run(() -> {}); }

}