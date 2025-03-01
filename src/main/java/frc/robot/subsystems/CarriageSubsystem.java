package frc.robot.subsystems;

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

public class CarriageSubsystem extends SubsystemBase{

    SparkMax rotationMotor;
    SparkClosedLoopController rotationController;
    RelativeEncoder rotationEncoder;

    SparkMax coralMotor;
    RelativeEncoder coralEncoder;
    boolean reversed;

    Servo direction;

    LiftSubsystem lift;

    public final double TopAngle = getPreference("TopAngle", CarriageConstants.TopAngle);
    public final double DeafaltAngle = getPreference("DefaultAngle", CarriageConstants.DefaultAngle);
    public final double intakeSpeed = getPreference("IntakeSpeed", CarriageConstants.intakeSpeed);

    CarriageSubsystem(){

        SparkMax rotationMotor = new SparkMax(CarriageConstants.rotationMotorCanId, MotorType.kBrushless);
        SparkClosedLoopController rotationController = rotationMotor.getClosedLoopController();
        RelativeEncoder rotationEncoder = rotationMotor.getEncoder();

        SparkMax coralMotor = new SparkMax(CarriageConstants.coralMotorCanId, MotorType.kBrushless);
        RelativeEncoder coralEncoder = coralMotor.getEncoder();
        boolean reversed = false;

        Servo direction = new Servo(CarriageConstants.directionChannel);

        LiftSubsystem lift = new LiftSubsystem();
    }

    public Command setDeafaultAngle(){

        return run(() -> {
            rotationController.setReference(DeafaltAngle, ControlType.kPosition);
        });
    }

    public Command setTopAngle(){

        return run(() -> {
            rotationController.setReference(TopAngle, ControlType.kPosition);
        });
    }

    public Command spinCoralIntake(){

        return run(() -> {
            double speed = reversed ? intakeSpeed : -intakeSpeed;
            coralMotor.set(speed);
        });
    }

    @Override
    public void periodic(){

        reversed = lift.currentDestination == lift.High ? true : false;
        if (reversed){
            setTopAngle();
        } else{
            setDeafaultAngle();
        }

        telemetry();
    }

    public void telemetry(){
        SmartDashboard.putNumber("Rotation Motor Position", rotationEncoder.getPosition());
        SmartDashboard.putNumber("Rotation Motor Position", coralEncoder.getVelocity());
        SmartDashboard.putNumber("Direction Servo Position", direction.getPosition());
    }

    private static double getPreference(String key, double fallback) {
        return Preferences.getDouble("Carriage/" + key, fallback);
    }
}
