package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class RotSubsystem extends SubsystemBase {
  private final SparkMax rotMotor;
  private final SparkMaxConfig motorConfig;
  private final SparkClosedLoopController closedLoop;

  public RotSubsystem() {
    rotMotor = new SparkMax(Constants.IntakeConstants.rotateMotorID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.025, ClosedLoopSlot.kSlot0) 
      .d(0.001)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
    motorConfig.encoder
      .positionConversionFactor(1);

    rotMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoop = rotMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
  }

  public void setPower(double power) {
    rotMotor.set(power);
  }
  public void setPosition(double position) {
    closedLoop.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

}
