// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final DrivetrainSubsystem drivetrain;
  private final SwerveSubsystem swerve = new SwerveSubsystem(Units.MetersPerSecond.of(4.8), new Pose2d());
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //drivetrain = new DrivetrainSubsystem();
    //drivetrain.setDefaultCommand(new TeleopDrive(drivetrain, driverController));

    swerve.setDefaultCommand(swerve.driveCommand(
            () -> driverController.getRawAxis(XboxController.Axis.kLeftX.value),
            () -> driverController.getRawAxis(XboxController.Axis.kLeftY.value),
            () -> driverController.getRawAxis(XboxController.Axis.kRightX.value),
            () -> driverController.getRawAxis(XboxController.Axis.kRightY.value)
    ));

    driverController.rightTrigger().whileTrue(
            swerve.driveTargeting(
                    () -> driverController.getRawAxis(XboxController.Axis.kLeftX.value),
                    () -> driverController.getRawAxis(XboxController.Axis.kLeftY.value),
                    // TODO: Hardcode the Reef centerpoint for red and blue and choose accordingly
                    new Translation2d(5.0, 5.0)
            )
    );

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }

  public Command getTestCommand() {
    return null;
  }
}
