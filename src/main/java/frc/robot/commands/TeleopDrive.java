// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.swerve.SwerveConstants;

public class TeleopDrive extends Command {
  private final DrivetrainSubsystem drivetrainSubsystem;

  private Joystick controller;
  private final JoystickButton robotCentric; // When pressed, change steering to robot centric

  public TeleopDrive(DrivetrainSubsystem drivetrainSubsystem, Joystick controller) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.controller = controller;
    robotCentric = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    addRequirements(drivetrainSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double readAxis(XboxController.Axis axis) {
    return MathUtil.applyDeadband(controller.getRawAxis(axis.value), frc.robot.Constants.OperatorConstants.stickDeadband);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationValue = readAxis(XboxController.Axis.kLeftY);
    double strafeValue = readAxis(XboxController.Axis.kLeftX);
    double rotationValue = readAxis(XboxController.Axis.kRightX);

    drivetrainSubsystem.drive(
      new Translation2d(translationValue, strafeValue).times(SwerveConstants.maxSpeed),
      rotationValue * SwerveConstants.maxAngularVelocity,
      !robotCentric.getAsBoolean(),
      true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
