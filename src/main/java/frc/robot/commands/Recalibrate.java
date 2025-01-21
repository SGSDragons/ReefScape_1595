package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Recalibrate extends Command {

    private final DrivetrainSubsystem drivetrain;
    public Recalibrate(DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(this.drivetrain);
    }

    @Override
    public void execute() {
        this.drivetrain.recalibrateCANCoders();
    }
}
