// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

/** Represents a differential drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  SparkMax leftLeader = new SparkMax(Constants.DrivetrainConstants.kLeftLeaderID, MotorType.kBrushless);
  SparkMax leftFollower = new SparkMax(Constants.DrivetrainConstants.kLeftFollowerID, MotorType.kBrushless);
  SparkMax rightLeader = new SparkMax(Constants.DrivetrainConstants.kRightLeaderID, MotorType.kBrushless);
  SparkMax rightFollower = new SparkMax(Constants.DrivetrainConstants.kRightFollowerID, MotorType.kBrushless);

    private RelativeEncoder leftEncoder = leftLeader.getEncoder();
    private RelativeEncoder rightEncoder = rightLeader.getEncoder();

    private Pigeon2 gyro = new Pigeon2(Constants.DrivetrainConstants.kPigeonID);

    private PIDController distancePID = new PIDController(0.01, 0, 0);
    private PIDController yawPID = new PIDController(0.01, 0, 0);

  MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader, leftFollower);
  MotorControllerGroup rightGroup = new MotorControllerGroup(rightLeader, rightFollower);

  private DifferentialDrive drive = new DifferentialDrive(leftGroup, rightGroup);

  Joystick m_driverController = new Joystick(Constants.OperatorConstants.kDriverControllerID);

  public double currentYaw = gyro.getYaw().getValueAsDouble();
  private double previousSpeed = 0.0;
  private final double accelerationRate = 0.05; // Adjust this value for smoother acceleration

  public Drivetrain() {
    MotorControllerGroup leftGroup = new MotorControllerGroup(leftLeader, leftFollower);
    MotorControllerGroup rightGroup = new MotorControllerGroup(rightLeader, rightFollower);
  }

  public void drive(CommandXboxController m_driverController2) {
    double targetSpeed = -m_driverController2.getRawAxis(1); // Get joystick input
    double turn = m_driverController2.getRawAxis(4) * 0.8;

    // Gradually increase or decrease speed
    if (targetSpeed > previousSpeed + accelerationRate) {
        targetSpeed = previousSpeed + accelerationRate;
    } else if (targetSpeed < previousSpeed - accelerationRate) {
        targetSpeed = previousSpeed - accelerationRate;
    }

    previousSpeed = targetSpeed; // Store last speed

    drive.arcadeDrive(turn, -targetSpeed);
  }

  public void driveToDistance(double targetDistance) {
    double leftDistance = leftEncoder.getPosition();
    double rightDistance = rightEncoder.getPosition();
    double averageDistance = (leftDistance + rightDistance) / 2.0;

    double speed = distancePID.calculate(averageDistance, targetDistance);

    leftLeader.set(speed);
    rightLeader.set(speed);
  }

  public void turnToAngle(double targetAngle) {
  double currentYaw = gyro.getYaw().getValueAsDouble();
  double yawCorrection = yawPID.calculate(currentYaw, targetAngle);

  leftLeader.set(yawCorrection);
  rightLeader.set(-yawCorrection);
  }

  public void resetSensors() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
    gyro.setYaw(0);
}

  @Override
  public void periodic() {
        double currentYaw = gyro.getYaw().getValueAsDouble();
        SmartDashboard.putNumber("Left Distance", leftEncoder.getPosition());
        SmartDashboard.putNumber("Right Distance", rightEncoder.getPosition());
        SmartDashboard.putNumber("Yaw", currentYaw);
  }
}
