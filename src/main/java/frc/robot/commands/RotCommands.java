package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotSubsystem;

public class RotCommands extends Command {
  private final RotSubsystem s_Rot;

  public RotCommands(RotSubsystem s_Rot) {
    this.s_Rot = s_Rot;
    addRequirements(s_Rot);
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
