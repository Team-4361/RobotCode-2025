package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class WinchSubsystem extends SubsystemBase {
    private SparkMax winchMotor;
    private RelativeEncoder winchEncoder;
    private Joystick driverStationJoystick;
    private DigitalInput limitSwitch;
    private PIDController winchPID;

    private static final int WINCH_MOTOR_ID = 15; // Change to your CAN ID
    private static final double WINCH_SPEED = 0.799; // Speed for manual movement
    private static final int LIMIT_SWITCH_PORT = 0; // DIO Port for limit switch

    private static final double kP = 0.1; // Proportional gain (tune this)
    private static final double kI = 0.0; // Integral gain (if needed)
    private static final double kD = 0.0; // Derivative gain (if needed)
    
    private double targetPosition = 0.0; // Setpoint for the winch position

    public WinchSubsystem() {
        winchMotor = new SparkMax(WINCH_MOTOR_ID, MotorType.kBrushless);
        winchEncoder = winchMotor.getEncoder();
        driverStationJoystick = new Joystick(0);
        limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        winchPID = new PIDController(kP, kI, kD);

        winchEncoder.setPosition(0); // Reset encoder at startup
    }

    public void teleopPeriodic() {
        // Safety: Stop motor if limit switch is triggered
        if (limitSwitch.get()) {
            winchMotor.set(0);
            System.out.println("Limit Switch Activated! Stopping Winch.");
            return;
        }

        // Manual Controls (Override PID)
        if (driverStationJoystick.getRawButton(1)) {  
            winchMotor.set(WINCH_SPEED); // Move up
            return;
        } else if (driverStationJoystick.getRawButton(2)) {  
            winchMotor.set(-WINCH_SPEED); // Move down
            return;
        }

        // Set target positions using joystick buttons
        if (driverStationJoystick.getRawButtonPressed(3)) { 
            targetPosition = 10.0; // Move to position 10
        }
        if (driverStationJoystick.getRawButtonPressed(4)) { 
            targetPosition = 20.0; // Move to position 20
        }
        if (driverStationJoystick.getRawButtonPressed(5)) { 
            targetPosition = 30.0; // Move to position 30
        }
        if (driverStationJoystick.getRawButtonPressed(6)) { 
            targetPosition = 0.0; // Reset to bottom
        }

        // PID Calculation
        double currentPos = winchEncoder.getPosition();
        double pidOutput = winchPID.calculate(currentPos, targetPosition);

        // Constrain output to safe range (-1 to 1)
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        // Apply PID output to motor
        winchMotor.set(pidOutput);

        // Debugging info
        System.out.println("Current Position: " + currentPos);
        System.out.println("Target Position: " + targetPosition);
        System.out.println("PID Output: " + pidOutput);
    }

    public void stop() {
        winchMotor.stopMotor();
    }
}
