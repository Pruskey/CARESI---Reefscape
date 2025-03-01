package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class DriveCommands extends Command {
    private Drivetrain s_Drivetrain; 
    private Joystick m_driverController;

    public DriveCommands (Drivetrain s_Drivetrain, Joystick m_driverController) {
        this.s_Drivetrain = s_Drivetrain;
        this.m_driverController = m_driverController;
        addRequirements(s_Drivetrain);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        s_Drivetrain.drive(m_driverController);
    }
    
}
