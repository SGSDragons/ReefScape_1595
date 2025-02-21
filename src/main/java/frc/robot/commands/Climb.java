// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {

  private final ClimbSubsystem climbSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  public ClimbDirection direction;

  public static enum ClimbDirection {
    UP(0.0),
    DOWN(0.0);

    double setpoint;
    ClimbDirection(double setpoint) {
        this.setpoint = setpoint;
    }
  }

  public Climb(ClimbSubsystem climbSubsystem, IntakeSubsystem intakeSubsystem, ClimbDirection direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climbSubsystem = climbSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(climbSubsystem, intakeSubsystem);

    this.direction = direction;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (direction) {
      case UP:
        // intakeSubsystem.intakeDown();
        // Intake needs to be down to shift center of gravity
        climbSubsystem.climbUp();
        break;
      case DOWN:
        climbSubsystem.climbDown();
        break;
    }
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
