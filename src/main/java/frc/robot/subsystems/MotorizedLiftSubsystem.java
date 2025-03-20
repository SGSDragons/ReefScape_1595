package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

public class MotorizedLiftSubsystem extends LiftSubsystem {

    final TalonFX motor;

    SparkMax rotationMotor;
    SparkClosedLoopController rotationController;
    RelativeEncoder rotationEncoder;

    // SparkMax Spool;
    // SparkClosedLoopController SpoolController;
    // RelativeEncoder SpoolEncoder;

    final BooleanSupplier bottomReached;
    boolean reversed;

    private double intakeAngle;
    private double topAngle;
    private double defaultAngle;

    public MotorizedLiftSubsystem() {
        
        TalonFX rightLiftMotor = new TalonFX(RightMotorCanId);
        TalonFX leftLiftMotor = new TalonFX(LeftMotorCanId);

        leftLiftMotor.setControl(new Follower(rightLiftMotor.getDeviceID(), true));
        rightLiftMotor.setNeutralMode(NeutralModeValue.Brake);
        motor = rightLiftMotor;

        SparkMaxConfig sparkConfig = new SparkMaxConfig();
        sparkConfig.closedLoop.p(0.2); // Found by testing with REV Hardware Client
        sparkConfig.closedLoop.d(0.001);

        rotationMotor = new SparkMax(FlipperCanId, MotorType.kBrushless);
        rotationMotor.configure(sparkConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        rotationController = rotationMotor.getClosedLoopController();
        rotationController.setReference(LiftConstants.IntakeAngle, ControlType.kPosition);
        rotationEncoder = rotationMotor.getEncoder();

        DigitalInput limit = new DigitalInput(LimitSwitchChannelId);
        bottomReached = () -> !limit.get();
        reversed = false;

        rereadPreferences();
    }

    private boolean isNearby(LiftPosition reference, double motorPos) {
        return Math.abs(motorPos - reference.setPoint) < 0.5;
    }
    @Override
    LiftPosition getPosition() {
        double motorPosition = motor.getPosition().getValueAsDouble();
        if (isNearby(Intake, motorPosition)) return Intake;
        if (isNearby(Shelf, motorPosition)) return Shelf;
        if (isNearby(Low, motorPosition)) return Low;
        if (isNearby(Medium, motorPosition)) return Medium;
        if (isNearby(High, motorPosition)) return High;
        return null;
    }

    @Override
    public void rereadPreferences() {
        final var config = new Slot0Configs();
        config.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        config.kG = preferences.get("gravity", LiftConstants.kG);
        config.kS = preferences.get("static", LiftConstants.kS);
        config.kP = preferences.get("proportional", LiftConstants.kP);
        config.kI = preferences.get("integral", LiftConstants.kI);
        config.kD = preferences.get("derivative", LiftConstants.kD);

        motor.getConfigurator().apply(config);

        intakeAngle = preferences.get("IntakeAngle", LiftConstants.IntakeAngle);
        topAngle = preferences.get("TopAngle", LiftConstants.TopAngle);
        defaultAngle  = preferences.get("DefaultAngle", LiftConstants.TopLimit);
    }

    @Override
    public Command rotateDown(){
        return run(() -> rotationMotor.set(-1));
    }

    @Override
    public Command rotateUp(){
        return run(() -> rotationMotor.set(1));
    }

    @Override
    public Command gotoPosition(LiftPosition position, DoubleSupplier axis) {

        return run(() -> {
            if (motor.getPosition().getValueAsDouble() >= defaultAngle){
                motor.set(0.0);
            }
            else {
                position.adjust(0.1*MathUtil.applyDeadband(axis.getAsDouble(), 0.2));

                if (0.1 < Math.abs(motor.getPosition().getValueAsDouble() - position.setPoint)) {
                    // create a position closed-loop request, voltage output, slot 0 configs
                    final PositionVoltage lift_request = new PositionVoltage(position.setPoint).withSlot(0);
                    // set position to 10 rotations
                    motor.setControl(lift_request);
                } else {
                    // Close enough
                    motor.set(0.0);
                }
            }

            final double carriageAngle;
            if (position == High) {
                carriageAngle = topAngle;
            } else if (position == Intake) {
                carriageAngle = intakeAngle;
            } else {
                carriageAngle = defaultAngle;
            }
            rotationController.setReference(carriageAngle, ControlType.kPosition);
        });
    }

    @Override
    public Command gotoGround() {
        PositionVoltage groundPosition = new PositionVoltage(0).withSlot(0);
        return run(() -> {
            if (motor.getPosition().getValueAsDouble() > 0.4) {
                // Drive with a PID when far away for max speed.
                motor.setControl(groundPosition);
            } else if (!bottomReached.getAsBoolean()) {
                // Drive slowly until the limit switch is not reached
                motor.set(-0.1);
            } else {
                // It's at the bottom. Stop the motor and rezero the encoder
                motor.set(0.0);
                //SpoolController.setReference(0, ControlType.kPosition);
                motor.setPosition(0.0);
            }
            //setIntakeAngle();
            //SpoolFollow();
            reversed = false;
            
            rotationController.setReference(defaultAngle, ControlType.kPosition);
        });
    }

    @Override
    public Command move(DoubleSupplier axis) {
         return run(() -> {
            if (motor.getPosition().getValueAsDouble() >= defaultAngle && axis.getAsDouble() > 0){
                motor.set(0.0);
            }
            else{
                double speed = MathUtil.applyDeadband(axis.getAsDouble(), 0.2);
                if (speed < 0 && bottomReached.getAsBoolean()) {
                    speed = 0.0;
                    motor.setPosition(0.0);
                }
                motor.set(speed);
                reversed = false;
            }
            rotationMotor.set(0);
            //setDefaultAngle();
            //SpoolFollow();
         });
    }

    // public void SpoolFollow(){
    //     double speed = motor.getVelocity().getValueAsDouble() * getPreference("ratio", LiftConstants.WiretoLiftRatio);
    //     Spool.set(speed);
    // }

    public void setIntakeAngle() {
        rotationController.setReference(intakeAngle, ControlType.kPosition);
    }

    public void setDefaultAngle() {
        // rotationController.setReference(defaultAngle, ControlType.kPosition);
    }

    public void setTopAngle() {
         rotationController.setReference(topAngle, ControlType.kPosition);
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
    }
}
