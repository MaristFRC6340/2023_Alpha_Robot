package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
    private CANSparkMax wristMotor;
    private RelativeEncoder armRelativeEncoder;

    public WristSubsystem() {
        wristMotor = new CANSparkMax(7, MotorType.kBrushless);
        armRelativeEncoder = wristMotor.getEncoder();
    }

    public void setWristMotorPower(double power) {
        wristMotor.set(power);
    }

    public double getWristPosition() {
        return armRelativeEncoder.getPosition();
      }

      
      public RelativeEncoder getWristEncoder() {
        return armRelativeEncoder;
      }
      
      public CANSparkMax getWristMotor() {
        return wristMotor;
      }
      
}
