package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  private final SparkFlex armMotor;
  private final SparkFlexConfig motorConfig;
  private final SparkClosedLoopController closedLoop;
  private RelativeEncoder encoder;
  private double position;

  public ArmSubsystem() {
    armMotor = new SparkFlex(Constants.ArmConstants.armMotorID, MotorType.kBrushless);
    motorConfig = new SparkFlexConfig();
    motorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.01, ClosedLoopSlot.kSlot0)
      .d(0.0065)
      .outputRange(-1, 1, ClosedLoopSlot.kSlot0);
    motorConfig.encoder
      .positionConversionFactor(300);

    armMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    closedLoop = armMotor.getClosedLoopController();
  }

  @Override
  public void periodic() {
    encoder = armMotor.getEncoder();
    position = encoder.getPosition() * 0.03;
    SmartDashboard.putNumber("armMotor encoder: ", position);
  }

  public void setPower(double power) {
    armMotor.set(power);
  }

  public void setPosition(double position) {
    closedLoop.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

public double getPosition() {
  return encoder.getPosition();    
}
  

}
