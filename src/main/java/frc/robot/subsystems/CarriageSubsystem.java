package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CarriageConstants;
import frc.robot.subsystems.LiftSubsystem.LiftPosition;

import static edu.wpi.first.units.Units.Milliseconds;
import static frc.robot.Constants.HardwareID.Carriage.*;

public class CarriageSubsystem extends SubsystemBase {
    SparkMax coralMotor;
    RelativeEncoder coralEncoder;

    Servo direction;

    LiftSubsystem lift;

    public double outtakeSpeed;
    public double pointRight;
    public double pointLeft;

    public CarriageSubsystem(LiftSubsystem lift) {

        coralMotor = new SparkMax(WheelsMotorCanId, MotorType.kBrushless);
        coralEncoder = coralMotor.getEncoder();

        direction = new Servo(directionServoChannelId);
        direction.set(CarriageConstants.middle);

        rereadPreferences();
    }

    public void rereadPreferences() {
        outtakeSpeed = getPreference("IntakeSpeed", CarriageConstants.outtakeSpeed);
        pointRight = getPreference("Right", CarriageConstants.pointRight);
        pointLeft = getPreference("Left", CarriageConstants.pointLeft);
    }

    public Command spin() {
        LiftPosition position = lift.getPosition();

        // Don't spin if we don't know the position.
        if (position == null) {
            return Commands.none();
        }
        
        final double speed;
        if (position == LiftSubsystem.Intake || position == LiftSubsystem.High) {
            speed = -outtakeSpeed;
        } else {
            speed = outtakeSpeed;
        }

        // Pulse in a direction for 100ms
        Command cmd = Commands.sequence(
            runOnce(() -> coralMotor.set(speed)),
            Commands.waitTime(Milliseconds.of(100)),
            runOnce(() -> coralMotor.set(0.0))
        );
        cmd.addRequirements(this);
        return cmd;

    }

    public Command pointMiddle() {
        return run(() -> direction.set(CarriageConstants.middle));
    }

    public Command pointRight() {
        return run(() -> direction.set(CarriageConstants.pointRight));
    }
    
    public Command pointLeft() {
        return run(() -> direction.set(CarriageConstants.pointLeft));
    }


    @Override
    public void periodic(){;
        telemetry();
    }

    public void telemetry(){
        SmartDashboard.putNumber("Coral Motor Velocity", coralEncoder.getVelocity());
        SmartDashboard.putNumber("Direction Servo Position", direction.getPosition());
    }

    private static double getPreference(String key, double fallback) {
        return Preferences.getDouble("Carriage/" + key, fallback);
    }
}
