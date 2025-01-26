package frc.robot.sysid;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SysIdBot {

    private final DrivetrainSubsystem drivetrain = new DrivetrainSubsystem();
    private final CommandXboxController controller = new CommandXboxController(Constants.OperatorConstants.driverControllerPort);

    public SysIdBot() {

        controller.x().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controller.y().whileTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        controller.a().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        controller.b().whileTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }
}
