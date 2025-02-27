package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;



public class WinchSubsystem extends SubsystemBase {
    private final SparkMax winchMotor;
    private final RelativeEncoder winchEncoder;
    private final PIDController winchPID;

    private static final double WINCH_SPEED_MULTIPLIER = 0.8; // Adjust max speed
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double TARGET_ROTATIONS = 90.0 / 360.0; // Convert degrees to rotations

    private double targetPosition = 0.0;

    public WinchSubsystem() {
        winchMotor = new SparkMax(Constants.climberConstants.WINCH_MOTOR_ID, MotorType.kBrushless);
        winchEncoder = winchMotor.getEncoder();
        winchPID = new PIDController(kP, kI, kD);
        winchEncoder.setPosition(0);
    }

    public void setWinchSpeed(double speed) {
        winchMotor.set(speed * WINCH_SPEED_MULTIPLIER);
    }

    public void moveWinchTo90Degrees() {
        targetPosition = TARGET_ROTATIONS;
        double pidOutput = winchPID.calculate(winchEncoder.getPosition(), targetPosition);
        winchMotor.set(pidOutput);
    }

    public void stopWinch() {
        winchMotor.set(0);
    }
}