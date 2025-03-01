package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkFlex IntakeMotor;
  private final SparkFlexConfig motorConfig;

  public IntakeSubsystem() {
    IntakeMotor = new SparkFlex(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();
    IntakeMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  public void setPowerInt(double power) {
    IntakeMotor.set(power);
  }

}
