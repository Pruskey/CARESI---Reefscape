package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommands extends Command {
  private final ClimbSubsystem s_Climb;

  public ClimbCommands(ClimbSubsystem s_Climb) {
    this.s_Climb = s_Climb;
    addRequirements(s_Climb);
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
