// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {

  private final ClimbSubsystem climbSubsystem;
  private final CoralIntakeSubsystem coralIntake;
  CommandXboxController climbControl;

  public Climb(ClimbSubsystem climbSubsystem, CoralIntakeSubsystem coralIntake, CommandXboxController climbControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climbSubsystem = climbSubsystem;
    this.coralIntake = coralIntake;
    this.climbControl = climbControl;
    addRequirements(climbSubsystem, coralIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    coralIntake.Spin(coralIntake.Retract);
    // Does this need to be delayed so coralIntake has time to clear?
    // First activation will cause Climb to move to the extended position
    climbSubsystem.activate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (climbSubsystem.hasExtended() && climbControl.povUp().getAsBoolean()) {
      // Second activation will cause Climb to move to the retracted position
      climbSubsystem.activate();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    // Don't ever let this command finish because it needs to keep the coral
    // intake reserved.
    return false;
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    // Don't let other commands disrupt this one.
    return InterruptionBehavior.kCancelIncoming;
  }
}
