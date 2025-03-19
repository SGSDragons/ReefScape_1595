package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CarriageConstants;
import frc.robot.Constants.LiftConstants;
import static frc.robot.Constants.HardwareID.Carriage.*;

public class CarriageSubsystem extends SubsystemBase{

    SparkMax coralMotor;
    RelativeEncoder coralEncoder;

    Servo direction;

    LiftSubsystem lift;

    public double outtakeSpeed = getPreference("IntakeSpeed", CarriageConstants.outtakeSpeed);
    public double pointRight = getPreference("Right", CarriageConstants.pointRight);
    public double pointLeft = getPreference("Left", CarriageConstants.pointLeft);

    public CarriageSubsystem(LiftSubsystem lift) {

        coralMotor = new SparkMax(WheelsMotorCanId, MotorType.kBrushless);
        coralEncoder = coralMotor.getEncoder();

        direction = new Servo(directionServoChannelId);
        direction.set(CarriageConstants.middle);

    }

    public void ConfigureSetpoints(){
        outtakeSpeed = getPreference("IntakeSpeed", CarriageConstants.outtakeSpeed);
        pointRight = getPreference("Right", CarriageConstants.pointRight);
        pointLeft = getPreference("Left", CarriageConstants.pointLeft);
    }

    public void spinCoralIntake(){
        double speed = lift.reversed ? outtakeSpeed : -outtakeSpeed;
        coralMotor.set(speed);
    }

    public Command middle(DoubleSupplier intakespeed){

        return run(() -> { 
            direction.set(CarriageConstants.middle);
            coralMotor.set(intakespeed.getAsDouble());
        });
    }

    public Command shootRight(){

        return run(() -> {
            direction.set(pointRight);
            // if-statement will run IMMEDIATELY, >>fix<<
            if (Math.abs(direction.getPosition() - pointRight) < 0.1) {
                spinCoralIntake();
            }
        });
    }

    public Command shootLeft(){

        return run(() -> {
            direction.set(pointLeft);
            // if-statement will run IMMEDIATELY, >>fix<<
            if (Math.abs(direction.getPosition() - pointLeft) < 0.1) {
                spinCoralIntake();
            }
        });
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
