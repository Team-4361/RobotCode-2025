package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BucketSubsystem extends SubsystemBase {
    private SparkMax coral;
    private final DigitalInput sensor1;
    private final DigitalInput sensor2;
   // private RelativeEncoder bucketEncoder;
    //private PIDController bucketPID;

    
    //private double targetPosition = 0.0;
    //private static final double GEAR_RATIO = 500.0; // 500:1 gear ratio
    //private static final double COUNTS_PER_REV = 42.0; // 42 counts per revolution
    //private static final double DEGREES_PER_MOTOR_REV = 360.0; // 1 motor rev = 360 degrees

   // private static final double POSITION_CONVERSION_FACTOR = 1;

    public BucketSubsystem() {
        coral = new SparkMax(Constants.Coral.BUCKET_ID, MotorType.kBrushless);
        this.sensor1 = new DigitalInput(Constants.Coral.PHOTOELECTRIC_SENSOR_1_PORT);
        this.sensor2 = new DigitalInput(Constants.Coral.PHOTOELECTRIC_SENSOR_2_PORT);        
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        coral.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        //driverStationJoystick = new Joystick(0);
        //limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
        //bucketPID = new PIDController(Constants.Coral.KP, Constants.Coral.KI, Constants.Coral.KD);        
    }


    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Got Coral", getSensor1());
        SmartDashboard.putBoolean("Has Coral", getSensor2());
       if (Constants.isDebug) {
        SmartDashboard.putString("Sensor 1 value", "" + sensor1.get());
        SmartDashboard.putString("Sensor 2 value", "" + sensor2.get());
        }
    }


    public boolean getSensor1()
    {
        return sensor1.get();
    }
    public boolean getSensor2()
    {
        return sensor2.get();
    }
    public void stop()
    {
        coral.set(0.0);
    }
    public void release()
    {
        //invert depending on rotation 
        coral.set(0.3); 
    }
    public void setMotor()
    {
        //sensors are normally true, when they change to false, the action is triggered
        if(!getSensor2())
        {
            coral.stopMotor();
        }
        else if(!getSensor1())
        {
            coral.set(0.70);
        }

        
    }

}
