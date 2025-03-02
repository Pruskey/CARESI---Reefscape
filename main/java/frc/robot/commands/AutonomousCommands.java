package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;

public class AutonomousCommands {

    private final Drivetrain drivetrain;
    private final ArmSubsystem arm;

    public AutonomousCommands(Drivetrain drivetrain, ArmSubsystem arm) {
        this.drivetrain = drivetrain;
        this.arm = arm;
    }

    // Drive forward a set distance
    public Command driveToDistanceCommand(double distance) {
        return new InstantCommand(() -> drivetrain.driveToDistance(distance), drivetrain);
    }

    // Turn to a set angle
    public Command turnToAngleCommand(double angle) {
        return new InstantCommand(() -> drivetrain.turnToAngle(angle), drivetrain);
    }

    // Set arm to a specific position
    public Command setArmPositionCommand(double position) {
        return new InstantCommand(() -> arm.setPosition(position), arm);
    }

    // Example autonomous sequence
    public Command getExampleAutonomous() {
        return new SequentialCommandGroup(
            driveToDistanceCommand(3.0), // Drive forward
            new WaitCommand(2),          // Wait 2 seconds
            setArmPositionCommand(49 * 300), // Move arm to a specific position
            new WaitCommand(1),          // Wait 1 second
            driveToDistanceCommand(2.0)  // Drive forward again
        );
    }
}
