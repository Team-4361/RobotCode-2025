package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class WinchSubsystem extends SubsystemBase {
    private final SparkMax winchMotor;
    private final RelativeEncoder winchEncoder;

    public WinchSubsystem() {
        winchMotor = new SparkMax(Constants.climberConstants.WINCH_MOTOR_ID, MotorType.kBrushless);
        winchEncoder = winchMotor.getEncoder();
        winchEncoder.setPosition(0);
    }

    public void setWinchSpeed(double speed) {
    if (Math.abs(speed) < Constants.climberConstants.DEADZONE) {
        stopWinch(); // Stop motor if within deadzone
    } else {
        winchMotor.set(Math.max(Constants.climberConstants.MIN_SPEED, Math.min(Constants.climberConstants.MAX_SPEED, speed)));
    }
}

    public void stopWinch() {
        winchMotor.set(0);
    }

    public double getWinchPosition() {
        return winchEncoder.getPosition();
    }
}
