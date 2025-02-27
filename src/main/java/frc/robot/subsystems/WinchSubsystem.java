package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class WinchSubsystem extends SubsystemBase {
    private final SparkMax winchMotor;
    private final RelativeEncoder winchEncoder;

    private static final double MAX_SPEED = 1.0;
    private static final double MIN_SPEED = -1.0;

    public WinchSubsystem() {
        winchMotor = new SparkMax(Constants.climberConstants.WINCH_MOTOR_ID, MotorType.kBrushless);
        winchEncoder = winchMotor.getEncoder();
        winchEncoder.setPosition(0);
    }

    public void setWinchSpeed(double speed) {
        winchMotor.set(Math.max(MIN_SPEED, Math.min(MAX_SPEED, speed)));
    }

    public void stopWinch() {
        winchMotor.set(0);
    }
}
