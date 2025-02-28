package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class BucketSubsystem extends SubsystemBase {
    private final SparkMax motor;
    private final Encoder encoder;
    private final PIDController pidController;

    private static final int MOTOR_ID = 15; // Change if needed
    private static final int CPR = 2048; // Encoder counts per revolution
    private static final double MOTOR_GEAR_RATIO = 1.0;
    private static final double KP = 0.0666;
    private static final double KI = 0.00002;
    private static final double KD = 0.0010;

    private static final double MAX_POWER = 0.3; // Adjust this value to limit speed
    
    private double targetAngle1 = 0.0; 

    public BucketSubsystem() {
        motor = new SparkMax(MOTOR_ID, MotorType.kBrushed);
        encoder = new Encoder(0, 1); // External encoder on RoboRIO

        encoder.setDistancePerPulse(360.0 / (CPR * MOTOR_GEAR_RATIO));
        pidController = new PIDController(KP, KI, KD);
        pidController.setTolerance(0.5);

       
    }

    public void forwardBucket() {
        targetAngle1 += 5.0;
    }

    public void backwardsBucket() {
        targetAngle1 -= 5.0;
    }

    public void resetBucket() {
        encoder.reset();
        targetAngle1 = 0.0;
    }

    public void zeroBucket() {
        targetAngle1 = 0.0;
    }

    public void stopBucket() {
        motor.stopMotor();
    }

    @Override
    public void periodic() {
        double currentAngle = encoder.getDistance();
        double pidOutput = pidController.calculate(currentAngle, targetAngle1);

        pidOutput = Math.max(-1, Math.min(1, pidOutput));

        pidOutput *= MAX_POWER;

        if (!pidController.atSetpoint()) {
            motor.set(pidOutput);
        } else {
            motor.set(0);
        }
    }
}
