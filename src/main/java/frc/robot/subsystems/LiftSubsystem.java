package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HardwareID.Lift.*;

import frc.robot.Constants.HardwareID;
import frc.robot.Constants.LiftConstants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class LiftSubsystem extends SubsystemBase{

    final TalonFX motor;
    TalonFX leftLiftMotor;

    SparkMax rotationMotor;
    SparkClosedLoopController rotationController;
    RelativeEncoder rotationEncoder;

    final BooleanSupplier bottomReached;
    boolean reversed;

    public final double TopAngle = getPreference("TopAngle", LiftConstants.TopAngle);
    public final double DeafaltAngle = getPreference("DefaultAngle", LiftConstants.DefaultAngle);

    public LiftSubsystem() {
        
        TalonFX rightLiftMotor = new TalonFX(RightMotorCanId);
        TalonFX leftLiftMotor = new TalonFX(LeftMotorCanId);

        leftLiftMotor.setControl(new Follower(rightLiftMotor.getDeviceID(), true));
        rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);
        motor = rightLiftMotor;

        SparkMax rotationMotor = new SparkMax(FlipperCanId, MotorType.kBrushless);
        SparkClosedLoopController rotationController = rotationMotor.getClosedLoopController();
        RelativeEncoder rotationEncoder = rotationMotor.getEncoder();

        // CB: When we wire the limit switch
        //bottomReached = new DigitalInput(0)::get;
        bottomReached = () -> false;
        boolean reversed = false;

        reconfigurePid();
    }

    public void reconfigurePid() {
        final var config = new Slot0Configs();
        config.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        config.kG = getPreference("gravity", LiftConstants.kG);
        config.kS = getPreference("static", LiftConstants.kS);
        config.kP = getPreference("proportional", LiftConstants.kP);
        config.kI = getPreference("integral", LiftConstants.kI);
        config.kD = getPreference("derivative", LiftConstants.kD);

        motor.getConfigurator().apply(config);
    }

    public void stopLift() {
        motor.set(0.0);
    }

    public static class LiftPosition {
        
        double setPoint;
        String name;

        public LiftPosition(String name, double fallback) {
            this.setPoint = getPreference(name, fallback);
            this.name = name;
        }

        public void adjust(double shift) {
            setPoint += shift;
            setPreference(name, setPoint);
        }
    }

    public final LiftPosition Shelf = new LiftPosition("Shelf", LiftConstants.Shelf);
    public final LiftPosition Low  = new LiftPosition("Low", LiftConstants.Low);
    public final LiftPosition Medium = new LiftPosition("Medium", LiftConstants.Medium);
    public final LiftPosition High = new LiftPosition("High", LiftConstants.High);

    public Command gotoPosition(LiftPosition position, DoubleSupplier axis) {

        return run(() -> {
            reversed = position == High ? true : false;

            if (reversed){
                setTopAngle();
            } else{
                setDeafaultAngle();
            }

            position.adjust(0.1*MathUtil.applyDeadband(axis.getAsDouble(), 0.2));

            // create a position closed-loop request, voltage output, slot 0 configs
            final PositionVoltage lift_request = new PositionVoltage(position.setPoint).withSlot(0);
            // set position to 10 rotations
            motor.setControl(lift_request);
        });
    }

    public Command goToGround() {

        return run(() -> {
            if (motor.getPosition().getValueAsDouble() > 0.2) {
                // Drive with a PID when far away for max speed.
                motor.setControl(new PositionVoltage(0));
            } else if (!bottomReached.getAsBoolean()) {
                // Drive slowly until the limit switch is not reached
                motor.set(-0.1);
            } else {
                // It's at the bottom. Stop the motor and rezero the encoder
                motor.set(0.0);
                motor.setPosition(0.0);
            }
            setDeafaultAngle();
            reversed = false;
        });
    }

    public Command move(DoubleSupplier axis) {
         return run(() -> {
            double speed = MathUtil.applyDeadband(axis.getAsDouble(), 0.2)/4;
            motor.set(speed);
         });
    }

    public void setDeafaultAngle(){
        rotationController.setReference(DeafaltAngle, ControlType.kPosition);
    }

    public void setTopAngle(){
         rotationController.setReference(TopAngle, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        telemetry();
    }

    public void telemetry() {
        SmartDashboard.putNumber("Right Motor Velocity", motor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Position", motor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Right Motor Voltage", motor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Rotation Motor Position", rotationEncoder.getPosition());
        //SmartDashboard.putNumber("Left Motor Voltage", leftLiftMotor.getMotorVoltage().getValueAsDouble());
    }

    private static double getPreference(String key, double fallback) {
        return Preferences.getDouble("Lift/" + key, fallback);
    }

    private static void setPreference(String key, double newValue) {
        Preferences.setDouble("Lift/" + key, newValue);
    }

    public double getLiftPostion(){
        return motor.getPosition().getValueAsDouble();
    }
}
