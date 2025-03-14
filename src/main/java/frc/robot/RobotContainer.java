// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.Reefscape;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

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
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.operatorControllerPort);
  private final ApproachFactory approaches;

  private final LiftSubsystem lift = new LiftSubsystem();
  //private final ClimbSubsystem climb = new ClimbSubsystem();
  //private final CoralIntakeSubsystem intake = new CoralIntakeSubsystem();
  private final CarriageSubsystem carriage = new CarriageSubsystem(lift);
  private final AlgaeSubsystem algae = new AlgaeSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    while (DriverStation.getAlliance().isEmpty()) {
      DriverStation.refreshData();
    }

    swerve.resetOdometry(Reefscape.getStart());
    approaches = new ApproachFactory(Reefscape.getReefLocation());
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
    public double lookX() { return inverter*readAxis(Axis.kRightX); }
    public double lookY() { return inverter*readAxis(Axis.kRightY); }
  }
  private final DriverSticks driver = new DriverSticks();

  // Controller behaviors when running in teleop mode. These should be tuned
  // for control, precision and speed when playing the game.
  public void engageTeleopMode() {

    DoubleSupplier leftY = () -> -operatorController.getRawAxis(Axis.kLeftY.value);
    DoubleSupplier rightY = () -> -operatorController.getRawAxis(Axis.kRightY.value);

    // Clear any bound triggers and create new bindings
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Drive relative to the field by default.
    // Left joystick up is away from the driver station. Left joystick left moves left on the field
    // Right joystick up looks away from the driver station. Right joystick left looks left on the field
    swerve.setDefaultCommand(swerve.driveCommand(driver::translateX, driver::translateY, driver::lookX, driver::lookY));

    // When holding the right bumper, change joysticks to drive relative to the Robot (left y is "forward", right x turns left/right)
    driverController.rightBumper().whileTrue(swerve.driveRelative(driver::translateX, driver::translateY, () -> -driver.readAxis(Axis.kRightX)));

    // When holding the right trigger, disable right joystick and make robot always face the reef
    driverController.rightTrigger(0.0).whileTrue(swerve.driveTargeting(driver::translateX, driver::translateY));
    driverController.a().whileTrue(new DynamicReefApproach(swerve, approaches));

    //driverController.povUp().onTrue(new Climb(climb, intake, driverController));

    // Lower the lift to its ground position whenever the operator is pushing the lift to another target
    lift.setDefaultCommand(lift.gotoGround());

    // Going to other positions requires holding a button. The joystick can be used
    // to make slow adjustments to the target position. These adjustments are permanent.
    operatorController.y().whileTrue(lift.gotoPosition(lift.High, leftY));
    operatorController.x().whileTrue(lift.gotoPosition(lift.Medium, leftY));
    operatorController.a().whileTrue(lift.gotoPosition(lift.Low, leftY));
    operatorController.b().whileTrue(lift.gotoPosition(lift.Shelf, leftY));

    algae.setDefaultCommand(algae.rotate(rightY));
    //algae.setDefaultCommand(algae.spin(rightY));

    carriage.setDefaultCommand(carriage.middle());
    operatorController.leftBumper().whileTrue(carriage.shootLeft());
    operatorController.rightBumper().whileTrue(carriage.shootRight());
  }

  // Controller behaviors when running in test mode. These are meant for
  // maximum flexibility (eg. moving the lift/climbers to arbitrary positions)
  public void engageTestMode() {

    DoubleSupplier leftY = () -> -operatorController.getRawAxis(Axis.kLeftY.value);
    DoubleSupplier rightY = () -> -operatorController.getRawAxis(Axis.kRightY.value);

    // Clear any bound triggers and create new bindings
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    swerve.setDefaultCommand(swerve.driveRelative(driver::translateX, driver::translateY, () -> -driver.readAxis(Axis.kRightX)));
    swerve.setMotorBrake(false);

    lift.setDefaultCommand(lift.move(leftY));
    //climb.setDefaultCommand(climb.drive(() -> operatorController.getRawAxis(Axis.kRightY.value)));

    //Reread Lift PID constants from preferences

    operatorController.a().whileTrue(lift.gotoPosition(lift.Low, leftY));
    operatorController.x().whileTrue(lift.gotoPosition(lift.Shelf, leftY));

    operatorController.leftTrigger().onTrue(algae.Extend());
    operatorController.leftTrigger().onFalse(algae.Retract());
    operatorController.y().onTrue(lift.runOnce(lift::reconfigurePid));

    algae.setDefaultCommand(algae.rotate(rightY));
    //algae.setDefaultCommand(algae.spin(rightY));

    carriage.setDefaultCommand(carriage.middle());
    operatorController.leftBumper().whileTrue(carriage.shootLeft());
    operatorController.rightBumper().whileTrue(carriage.shootRight());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
    // CB: Do not return a real auto until we've got more safeguards. We don't
    // want a misclick trigger an auto that causes the robot to make large movements
//    Command auto = AutoBuilder.buildAuto("Auto_South-3Coral");
//    return auto;
  }
}
