package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommands extends Command {
  private final IntakeSubsystem s_Int;

  public IntakeCommands(IntakeSubsystem s_Int) {
    this.s_Int = s_Int;
    addRequirements(s_Int);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
