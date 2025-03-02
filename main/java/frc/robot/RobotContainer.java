package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RotSubsystem;
import frc.robot.commands.AutonomousCommands;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private Drivetrain drivetrain = new Drivetrain();
  private Joystick m_driverController = new Joystick(0);
  private CommandXboxController DriveController = new CommandXboxController(1);
  private CommandXboxController IntakeController = new CommandXboxController(0);
  private ArmSubsystem arm = new ArmSubsystem();
  private IntakeSubsystem Intake = new IntakeSubsystem();
  private RotSubsystem rot = new RotSubsystem();
  private ClimbSubsystem climb = new ClimbSubsystem();
  private AutonomousCommands autoCommands = new AutonomousCommands(drivetrain, arm);

  private final double MAX_ARM_POSITION = -9 * 300;
  private boolean swapControls = false;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    DriveController.start().onTrue(new InstantCommand(this::toggleControlScheme));

    drivetrain.setDefaultCommand(new DriveCommands(drivetrain, getDriveController()));

    getIntakeController().b().onTrue(new InstantCommand(() -> climb.setPower(0.2), climb))
    .onFalse(new InstantCommand(() -> climb.setPower(0), climb));

    getIntakeController().a().onTrue(new InstantCommand(() -> arm.setPower(0.2), arm))
        .onFalse(new InstantCommand(() -> arm.setPower(0), arm));

    getIntakeController().y().onTrue(new InstantCommand(() -> {
      if (arm.getPosition() > MAX_ARM_POSITION) {
        arm.setPower(-0.2);
      } else {
        arm.setPower(0.05);
      }
    }, arm)).onFalse(new InstantCommand(() -> arm.setPower(0), arm));

    getIntakeController().axisGreaterThan(3, 0.2).onTrue(new InstantCommand(() -> Intake.setPowerInt(-0.5), Intake))
        .onFalse(new InstantCommand(() -> Intake.setPowerInt(0), Intake));

    getIntakeController().axisGreaterThan(2, 0.2).onTrue(new InstantCommand(() -> Intake.setPowerInt(0.2), Intake))
        .onFalse(new InstantCommand(() -> Intake.setPowerInt(0), Intake));

    getIntakeController().povDown().onTrue(new InstantCommand(() -> arm.setPosition(49 * 300)));
    getIntakeController().povLeft().onTrue(new InstantCommand(() -> arm.setPosition(33 * 300)));
    getIntakeController().povUp().onTrue(new InstantCommand(() -> arm.setPosition(-8 * 300)));
    getIntakeController().povRight().onTrue(new InstantCommand(() -> arm.setPosition(28 * 300)));

    getIntakeController().rightBumper().onTrue(new InstantCommand(() -> rot.setPosition(25)));
    getIntakeController().leftBumper().onTrue(new InstantCommand(() -> rot.setPosition(0)));
    getIntakeController().x().onTrue(new InstantCommand(() -> rot.setPosition(50)));
  }

  private void toggleControlScheme() {
    swapControls = !swapControls;
    SmartDashboard.putBoolean("Control Scheme Swapped", swapControls);
  }

  private CommandXboxController getDriveController() {
    return swapControls ? DriveController : IntakeController;
  }

  private CommandXboxController getIntakeController() {
    return swapControls ? IntakeController : DriveController;
  }

  public Command getAutonomousCommand() {
    return autoCommands.getExampleAutonomous();
  }
}
