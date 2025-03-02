// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private Drivetrain drivetrain = new Drivetrain();
  private Joystick m_driverController = new Joystick(0);
  private CommandXboxController controllerS = new CommandXboxController(1);
  private CommandXboxController controller = new CommandXboxController(0);
  private ArmSubsystem arm = new ArmSubsystem();
  private IntakeSubsystem Intake = new IntakeSubsystem();
  private RotSubsystem rot = new RotSubsystem();
  private ClimbSubsystem climb = new ClimbSubsystem();
  private AutonomousCommands autoCommands = new AutonomousCommands(drivetrain, arm);

  private final double MAX_ARM_POSITION = -9 * 300;

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    
    //Drivetrain
    drivetrain.setDefaultCommand(new DriveCommands(drivetrain, m_driverController));
    
    controller.axisGreaterThan(0, 0);
    controller.button(0);


    //Mecher braço manualmente no analógico
    /*controllerS.axisGreaterThan(1, 0.2).onTrue(new InstantCommand(() -> arm.setPower(0.2), arm))
    .onFalse(new InstantCommand(() -> arm.setPower(0), arm));

    controllerS.axisLessThan(1, -0.2).onTrue(new InstantCommand(() -> arm.setPower(-0.2), arm))
    .onFalse(new InstantCommand(() -> arm.setPower(0), arm));
    */

    controllerS.a().onTrue(new InstantCommand(() -> arm.setPower(0.2), arm))
    .onFalse(new InstantCommand(() -> arm.setPower(0), arm));

    controllerS.y().onTrue(new InstantCommand(() -> {
      if (arm.getPosition() > MAX_ARM_POSITION) {
        arm.setPower(-0.2);
      } else {
        arm.setPower(0.05);
      }
    }, arm)).onFalse(new InstantCommand(() -> arm.setPower(0), arm));

    /*controllerS.y().onTrue(new InstantCommand(() -> arm.setPower(-0.2), arm))
    .onFalse(new InstantCommand(() -> arm.setPower(0), arm));*/

    //Power Intake
    controllerS.axisGreaterThan(3, 0.2).onTrue(new InstantCommand(() -> Intake.setPowerInt(-0.5), Intake))
    .onFalse(new InstantCommand(() -> Intake.setPowerInt(0), Intake));

    controllerS.axisGreaterThan(2, 0.2).onTrue(new InstantCommand(() -> Intake.setPowerInt(0.2), Intake))
    .onFalse(new InstantCommand(() -> Intake.setPowerInt(0), Intake));


    //Power Climb
    /*controller.x().onTrue(new InstantCommand(() -> climb.setPower(-0.5), climb))
    .onFalse(new InstantCommand(() -> climb.setPower(0), climb));*/

    controller.b().onTrue(new InstantCommand(() -> climb.setPower(0.5), climb))
    .onFalse(new InstantCommand(() -> climb.setPower(0), climb));

    

    //PID Braco
    controllerS.povDown().onTrue(new InstantCommand(() -> arm.setPosition(49 * 300)));
    controllerS.povLeft().onTrue(new InstantCommand(() -> arm.setPosition(33 * 300)));
    controllerS.povUp().onTrue(new InstantCommand(() -> arm.setPosition(-8 * 300)));
    controllerS.povRight().onTrue(new InstantCommand(() -> arm.setPosition(28 * 300)));

    //Girar Intake
    controllerS.rightBumper().onTrue(new InstantCommand(() -> rot.setPosition(25)));
    controllerS.leftBumper().onTrue(new InstantCommand(() -> rot.setPosition(0)));
    controllerS.x().onTrue(new InstantCommand(() -> rot.setPosition(50)));
    
   

    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return Autos.exampleAuto(m_exampleSubsystem);
    return autoCommands.getExampleAutonomous();
  }
}
