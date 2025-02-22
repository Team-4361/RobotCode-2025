package frc.robot.commands.climber;

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

/*public class KerklunkSubsystem extends SubsystemBase {
    private SparkMax sparkMax;
    private SparkMax lClimberSparkMax;
    private SparkMax rClimberSparkMax;
    public static double ClimberSpeed = 0.5;
    public static int leftClimberMotorID = 9;
    public static int rightClimberMotorID = 10;
    public static PIDController ClimberPID = new PIDController(0.0, 0.0, 0.0);
    private RelativeEncoder eEncoderL;
    private RelativeEncoder eEncoderR;
    private RelativeEncoder encoder;
    private Joystick driverStationJoystick; // Driver Station Joystick

    private static final int SPARK_MAX_CAN_ID = 6;
    private static final double POSITION_TOLERANCE = 0.02;

    private static final double kP = 0.5;
    private static final double kI = 0.2;
    private static final double kD = 0.0;

    private double integral = 0.0;
    private double previousError = 0.0;
    private double targetPosition = 0.0;
    private double ClimberPosition = 0.0;

    public ClimberSubsystem() {
        sparkMax = new SparkMax(SPARK_MAX_CAN_ID, MotorType.kBrushless);
        lClimberSparkMax = new SparkMax(leftClimberMotorID, MotorType.kBrushless);
        rClimberSparkMax = new SparkMax(rightClimberMotorID, MotorType.kBrushless);
        driverStationJoystick = new Joystick(0); // Initialize Joystick on USB Port 0

        SparkMaxConfig config = new SparkMaxConfig();
        SparkMaxConfig eConfig = new SparkMaxConfig();
        config.encoder.positionConversionFactor(9.0 / 40.0);
        sparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        lClimberSparkMax.configure(eConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rClimberSparkMax.configure(eConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = sparkMax.getEncoder();
        eEncoderL = lClimberSparkMax.getEncoder();
        eEncoderR = rClimberSparkMax.getEncoder();
    }
    
    public void teleopPeriodic() {
        if (driverStationJoystick.getRawButtonPressed(4)) { // Button 4 -> Move Climber to 30.6
            ClimberPosition = 30.6;
        }
        if (driverStationJoystick.getRawButtonPressed(5)) { // Button 5 -> Move Climber to 0
            ClimberPosition = 0;
        }

        if (driverStationJoystick.getRawButtonPressed(1)) { // Button 1 (Trigger) -> Move up
            targetPosition = 3;
        }
        if (driverStationJoystick.getRawButtonPressed(2)) { // Button 2 -> Reset position
            targetPosition = 0.0;
        }
        if (driverStationJoystick.getRawButtonPressed(3)) { // Button 3 -> Reset encoder
            encoder.setPosition(0);
            targetPosition = 0.0;
        }
        if (driverStationJoystick.getRawButtonPressed(6)) { // Button 6 -> Move down
            targetPosition = -3;
        }
        if (driverStationJoystick.getRawButtonPressed(7)) { // Button 7 -> Decrease position
            targetPosition = -6;
        }
        if (driverStationJoystick.getRawButtonPressed(8)) { // Button 8 -> Increase position
            targetPosition = 6;
        }

        double currentPosition = encoder.getPosition();
        double error = targetPosition - currentPosition;

        integral += error * 0.02;
        double derivative = (error - previousError) / 0.02;

        double pidOutput = (kP * error) + (kI * integral) + (kD * derivative);
        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        if (Math.abs(error) > POSITION_TOLERANCE) {
            sparkMax.set(pidOutput);
        } else {
            sparkMax.set(0);
        }

        previousError = error;

        System.out.println("Current Position: " + currentPosition);
        System.out.println("Target Position: " + targetPosition);
        System.out.println("PID Output: " + pidOutput);
    }

    public void stop() {
        sparkMax.stopMotor();
        lClimberSparkMax.stopMotor();
        rClimberSparkMax.stopMotor();
    }
}*/
