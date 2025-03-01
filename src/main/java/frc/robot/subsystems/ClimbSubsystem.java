package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSubsystem extends SubsystemBase {
  private final SparkMax climbMotor;
  private final SparkMaxConfig motorConfig;

  public ClimbSubsystem() {
    climbMotor = new SparkMax(Constants.ArmConstants.climbMotorID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    climbMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  public void setPower(double power) {
    climbMotor.set(power);
  }

}
