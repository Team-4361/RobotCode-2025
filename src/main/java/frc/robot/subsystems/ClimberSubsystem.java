package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

public class ClimberSubsystem extends SubsystemBase {
    private SparkMax climberMotor;
    private RelativeEncoder encoder;
    private Joystick driverStationJoystick;

    private static final int CLIMBER_MOTOR_ID = 6;
    private static final double POSITION_TOLERANCE = 0.02;
    private static final double kP = 0.5;
    private static final double kI = 0.2;
    private static final double kD = 0.0;

    private double integral = 0.0;
    private double previousError = 0.0;
    private double targetPosition = 0.0;

    public ClimberSubsystem() {
        climberMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
        driverStationJoystick = new Joystick(0);

        SparkMaxConfig config = new SparkMaxConfig();
        config.encoder.positionConversionFactor(9.0 / 40.0);
        climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        encoder = climberMotor.getEncoder();
    }
    
    public void teleopPeriodic() {
        if (driverStationJoystick.getRawButtonPressed(4)) {
            targetPosition = 30.6;
        }
        if (driverStationJoystick.getRawButtonPressed(5)) {
            targetPosition = 0;
        }
        if (driverStationJoystick.getRawButtonPressed(1)) {
            targetPosition = 3;
        }
        if (driverStationJoystick.getRawButtonPressed(2)) {
            targetPosition = 0.0;
        }
        if (driverStationJoystick.getRawButtonPressed(3)) {
            encoder.setPosition(0);
            targetPosition = 0.0;
        }
        if (driverStationJoystick.getRawButtonPressed(6)) {
            targetPosition = -3;
        }
        if (driverStationJoystick.getRawButtonPressed(7)) {
            targetPosition = -6;
        }
        if (driverStationJoystick.getRawButtonPressed(8)) {
            targetPosition = 6;
        }

        double currentPosition = encoder.getPosition();
        double error = targetPosition - currentPosition;

        integral += error * 0.02;
        double derivative = (error - previousError) / 0.02;

        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        if (Math.abs(error) > POSITION_TOLERANCE) {
            climberMotor.set(pidOutput);
        } else {
            climberMotor.set(0);
        }

        previousError = error;

        System.out.println("Current Position: " + currentPosition);
        System.out.println("Target Position: " + targetPosition);
        System.out.println("PID Output: " + pidOutput);
    }

    public void stop() {
        climberMotor.stopMotor();
    }
}