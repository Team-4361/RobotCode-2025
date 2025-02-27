package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class WinchSubsystem extends SubsystemBase {
    private final SparkMax winchMotor;
    private final RelativeEncoder winchEncoder;
    private final PIDController winchPID;
    private final XboxController controller;

    private static final double BASE_WINCH_SPEED = 0.5;
    private double winchSpeedMultiplier = BASE_WINCH_SPEED;
    private static final double SPEED_INCREMENT = 0.1;
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double GEAR_RATIO = 2.0; // Adjust for actual gear ratio
    private static final double TARGET_ROTATIONS = (90.0 / 360.0) * GEAR_RATIO; 

    private double targetPosition = 0.0;

    public WinchSubsystem(XboxController controller) {
        this.controller = controller;
        winchMotor = new SparkMax(Constants.climberConstants.WINCH_MOTOR_ID, MotorType.kBrushless);
        winchEncoder = winchMotor.getEncoder();
        winchPID = new PIDController(kP, kI, kD);
        winchEncoder.setPosition(0);
    }

    public void setWinchSpeed(double speed) {
        winchMotor.set(speed * winchSpeedMultiplier);
    }

    public void moveWinchTo90Degrees() {
        targetPosition = TARGET_ROTATIONS;
        double pidOutput = winchPID.calculate(winchEncoder.getPosition(), targetPosition);
        winchMotor.set(pidOutput);
    }

    public void stopWinch() {
        winchMotor.set(0);
    }

    @Override
    public void periodic() {
        if (controller.getRawButtonPressed(5)) { // Left D-pad (increase speed)
            winchSpeedMultiplier = Math.min(winchSpeedMultiplier + SPEED_INCREMENT, 1.0);
        } else if (controller.getRawButtonPressed(6)) { // Right D-pad (decrease speed)
            winchSpeedMultiplier = Math.max(winchSpeedMultiplier - SPEED_INCREMENT, 0.1);
        }
    }
}
