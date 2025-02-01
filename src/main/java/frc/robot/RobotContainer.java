// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Reefscape;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ApproachReef;
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
  private final SwerveSubsystem swerve = new SwerveSubsystem(Units.MetersPerSecond.of(3), Pose2d.kZero);
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.resetOdometry(Reefscape.getStart());

    // Configure the trigger bindings
    configureBindings();
  }

  //      ______________________________(17, 8)
  //      |                                |
  //     B|                                |R
  //     B|                                |R
  //     B|                                |R
  //     B|                                |R
  //      |________________________________|
  //    (0,0)
  //
  // With 0deg heading pointing right.
  //
  // When on the Blue alliance
  //    Left joystick Up (-1 y-axis) maps to positive X.
  //    Left joystick left (-1 x-axis) maps to positive Y.
  // Thus, Blue axis are inverted.

  class DriverSticks {
    private final double inverter = Reefscape.isRedAlliance() ? 1.0 : -1.0;
    double readAxis(XboxController.Axis axis) {
      return driverController.getRawAxis(axis.value);
    }
    public double translateX() { return inverter*readAxis(Axis.kLeftY); }
    public double translateY() { return inverter*readAxis(Axis.kLeftX); }
    public double lookX() { return inverter*driverController.getRawAxis(2); }//readAxis(Axis.kRightX); }
    public double lookY() { return inverter*driverController.getRawAxis(3); }//readAxis(Axis.kRightY); }
  }
  
  private void configureBindings() {

    while (DriverStation.getAlliance().isEmpty()) {
      DriverStation.refreshData();
    }
    DriverSticks driver = new DriverSticks(); 

    swerve.setDefaultCommand(swerve.driveCommand(driver::translateX, driver::translateY, driver::lookX, driver::lookY));

    driverController.leftBumper().whileTrue(
      swerve.driveRelative(driver::translateX, driver::translateY, () -> -driver.readAxis(Axis.kRightX))
    );
    driverController.rightTrigger().whileTrue(swerve.driveTargeting(driver::translateX, driver::translateY));

    driverController.a().whileTrue(new ApproachReef(swerve));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("RedStation1");
  }

  public Command getTestCommand() {
    return null;
  }
}
