// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Represents a differential drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  SparkMax leftLeader = new SparkMax(Constants.DrivetrainConstants.kLeftLeaderID, MotorType.kBrushless);
  SparkMax leftFollower = new SparkMax(Constants.DrivetrainConstants.kLeftFollowerID, MotorType.kBrushless);
  SparkMax rightLeader = new SparkMax(Constants.DrivetrainConstants.kRightLeaderID, MotorType.kBrushless);
  SparkMax rightFollower = new SparkMax(Constants.DrivetrainConstants.kRightFollowerID, MotorType.kBrushless);
//e
  MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader, leftFollower);
  MotorControllerGroup rightGroup = new MotorControllerGroup(rightLeader, rightFollower);

  private DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  Joystick m_driverController = new Joystick(Constants.OperatorConstants.kDriverControllerID);

  private double previousSpeed = 0.0;
  private final double accelerationRate = 0.05; // Adjust this value for smoother acceleration

  public Drivetrain() {
    MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader, leftFollower);
    MotorControllerGroup rightGroup = new MotorControllerGroup(rightLeader, rightFollower);
  }

  public void drive(Joystick m_driverController) {
    double targetSpeed = -m_driverController.getRawAxis(1); // Get joystick input
    double turn = m_driverController.getRawAxis(4) * 0.8;

    // Gradually increase or decrease speed
    if (targetSpeed > previousSpeed + accelerationRate) {
        targetSpeed = previousSpeed + accelerationRate;
    } else if (targetSpeed < previousSpeed - accelerationRate) {
        targetSpeed = previousSpeed - accelerationRate;
    }

    previousSpeed = targetSpeed; // Store last speed

    drive.arcadeDrive(turn, -targetSpeed);
  }

  @Override
  public void periodic() {
  }
}
