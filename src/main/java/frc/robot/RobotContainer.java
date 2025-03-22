// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.driverControllerPort);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.operatorControllerPort);

  private final DriveSubsystem drive = new SwerveSubsystem(Units.MetersPerSecond.of(6), Pose2d.kZero);
  private final MotorizedLiftSubsystem lift = new MotorizedLiftSubsystem();
  //private final ClimbSubsystem climb = new ClimbSubsystem();
  private final CoralIntakeSubsystemFake intake = new CoralIntakeSubsystemFake();
  private final CarriageSubsystem carriage = new CarriageSubsystem(lift);
  private final AlgaeSubsystem algae = new AlgaeSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    while (DriverStation.getAlliance().isEmpty()) {
      DriverStation.refreshData();
    }

    if (drive instanceof SwerveSubsystem) {
      SwerveSubsystem swerve = (SwerveSubsystem) drive;
      swerve.resetOdometry(Reefscape.getStart());
    }
    NamedCommands.registerCommand("shelf", lift.gotoPosition(LiftSubsystem.Shelf, () -> 0.0));
    NamedCommands.registerCommand("shoot", carriage.spin());
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
    private final double inverter = -1.0;
    double readAxis(XboxController.Axis axis) {
      return driverController.getRawAxis(axis.value);
    }
    public double translateX() { return inverter*readAxis(Axis.kLeftY); }
    public double translateY() { return inverter*readAxis(Axis.kLeftX); }
    public double lookX() { return inverter*readAxis(Axis.kRightX); }
    public double lookY() { return inverter*readAxis(Axis.kRightY); }
  }


  // Controller behaviors when running in teleop mode. These should be tuned
  // for control, precision and speed when playing the game.
  public void engageTeleopMode() {
    DriverSticks driver = new DriverSticks();

    DoubleSupplier leftY = () -> -operatorController.getRawAxis(Axis.kLeftY.value);
    DoubleSupplier rightY = () -> -operatorController.getRawAxis(Axis.kRightY.value);
    DoubleSupplier lefttrigger = () -> operatorController.getRawAxis(Axis.kLeftTrigger.value);
    DoubleSupplier righttrigger = () -> -operatorController.getRawAxis(Axis.kRightTrigger.value);

    // Clear any bound triggers and create new bindings
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // Drive relative to the field by default.
    // Left joystick up is away from the driver station. Left joystick left moves left on the field
    // Right joystick up looks away from the driver station. Right joystick left looks left on the field
    drive.setDefaultCommand(drive.driveCommand(driver::translateX, driver::translateY, driver::lookX, driver::lookY));

    // When holding the right bumper, change joysticks to drive relative to the Robot (left y is "forward", right x turns left/right)
    driverController.rightBumper().whileTrue(drive.driveRelative(driver::translateX, driver::translateY, () -> -driver.readAxis(Axis.kRightX)));

    // When holding the right trigger, disable right joystick and make robot always face the reef
    driverController.rightTrigger(0.0).whileTrue(drive.driveTargeting(driver::translateX, driver::translateY));

    //driverController.povUp().onTrue(new Climb(climb, intake, driverController));


    // Going to other positions requires holding a button. The joystick can be used
    // to make slow adjustments to the target position. These adjustments are permanent.
    operatorController.a().onTrue(lift.gotoPosition(LiftSubsystem.Shelf, leftY));
    operatorController.x().onTrue(lift.gotoPosition(LiftSubsystem.Low, leftY));
    operatorController.b().onTrue(lift.gotoPosition(LiftSubsystem.Low, leftY));
    operatorController.y().onTrue(lift.gotoPosition(LiftSubsystem.Medium, leftY));

    operatorController.rightTrigger().onTrue(carriage.spin());

    operatorController.povDown().onTrue(lift.gotoGround());
    operatorController.povUp().onTrue(lift.gotoPosition(LiftSubsystem.Intake, leftY));


    operatorController.leftBumper().onTrue(algae.Extend(rightY));
    operatorController.rightBumper().onTrue(algae.Retract(rightY));
    algae.setDefaultCommand(algae.Roller(rightY));
    //algae.setDefaultCommand(algae.rotate(rightY));
    //algae.setDefaultCommand(algae.spin(rightY));
    //operatorController.leftTrigger().onTrue(algae.Roller(righttrigger));
    //operatorController.rightTrigger().onTrue(algae.Roller(lefttrigger));

    // carriage.setDefaultCommand(carriage.middle());
    // operatorController.leftBumper().whileTrue(carriage.shootLeft());
    // operatorController.rightBumper().whileTrue(carriage.shootRight());
  }

  // Controller behaviors when running in test mode. These are meant for
  // maximum flexibility (eg. moving the lift/climbers to arbitrary positions)
  public void engageTestMode() {
    DriverSticks driver = new DriverSticks();

    DoubleSupplier leftY = () -> -operatorController.getRawAxis(Axis.kLeftY.value);
    DoubleSupplier rightY = () -> operatorController.getRawAxis(Axis.kRightY.value);
    DoubleSupplier lefttrigger = () -> operatorController.getRawAxis(Axis.kLeftTrigger.value);
    DoubleSupplier righttrigger = () -> -operatorController.getRawAxis(Axis.kRightTrigger.value);

    // Clear any bound triggers and create new bindings
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    //Drive

    drive.setDefaultCommand(drive.driveRelative(driver::translateX, driver::translateY, () -> -driver.readAxis(Axis.kRightX)));

  //climb.setDefaultCommand(climb.drive(rightY));

    lift.setDefaultCommand(lift.move(leftY));

    //Reread Lift PID constants from preferences
    // operatorController.y().onTrue(lift.runOnce(lift::reconfigurePid));

    operatorController.a().onTrue(lift.gotoPosition(LiftSubsystem.Low, leftY));
    operatorController.x().onTrue(lift.gotoPosition(LiftSubsystem.Shelf, leftY));


    //operatorController.povUp().onTrue(carriage.spin());
    operatorController.leftBumper().whileTrue(carriage.shootLeft(rightY));
    operatorController.rightBumper().whileTrue(carriage.shootRight(rightY));
    carriage.setDefaultCommand(carriage.Middle(rightY));

    operatorController.b().onTrue(carriage.pointMiddle());
    operatorController.leftBumper().onTrue(carriage.pointLeft());
    operatorController.rightBumper().onTrue(carriage.pointRight());

    //algae.setDefaultCommand(algae.Roller(rightY));
    // operatorController.leftTrigger().onTrue(algae.Roller(righttrigger));
    // operatorController.rightBumper().onTrue(algae.Roller(lefttrigger));

    operatorController.y().onTrue(lift.runOnce(lift::rereadPreferences ));
    // operatorController.y().onTrue(lift.runOnce(lift::reconfigurePid));

    //algae.setDefaultCommand(algae.rotate(rightY));
    //algae.setDefaultCommand(algae.spin(rightY));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
public Command getAutonomousCommand() {
  return new DriveForward();
    //return new PathPlannerAuto("middleauto1");

//    Approach ideal;
//    switch(DriverStation.getRawAllianceStation()) {
//      case Blue1: ideal = approaches.forAngleDegrees(60);
//      case Blue2: ideal = approaches.forAngleDegrees(0);
//      case Blue3: ideal = approaches.forAngleDegrees(-60);
//
//      case Red1: ideal = approaches.forAngleDegrees(-120);
//      case Red2: ideal = approaches.forAngleDegrees(180);
//      default: ideal = approaches.forAngleDegrees(120);;
//    }
//
//    PathPlannerPath path = ideal.generatePath(Reefscape.getStart().getTranslation(), Translation2d.kZero);
//
//    List<Pose2d> poses = path.getAllPathPoints().stream()
//            .map(p -> new Pose2d(p.position, Rotation2d.kZero))
//            .toList();
//    swerve.getSwerveDrive().field.getObject("Trajectory").setPoses(poses);
//    Command follow = AutoBuilder.followPath(path);
//
//    return new FunctionalCommand(
//            () -> { System.err.println("INIT"); follow.initialize(); },
//            () -> { System.err.println("STEP"); follow.execute(); },
//            (i) -> { System.err.println("END: " + i); follow.end(i); },
//            () -> follow.isFinished(),
//            swerve
//    );
  }

  class DriveForward extends Command {
    private Instant limit;
    @Override
    public void initialize() {
      limit = Instant.now().plusSeconds(5);
    }

    @Override
    public void execute() {
      if (Instant.now().isAfter(limit)) {
        ((SwerveSubsystem)drive).drive(Translation2d.kZero, 0.0, false);
      } else {
        ((SwerveSubsystem)drive).drive(new Translation2d(0.2, 0.0), 0.0, false);
      }
    }

    @Override
    public boolean isFinished() {
      return Instant.now().isAfter(limit);
    }

    @Override
    public void end(boolean interrupted) {
      ((SwerveSubsystem)drive).drive(Translation2d.kZero, 0.0, false);
    }
  }
}
