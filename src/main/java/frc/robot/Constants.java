// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final boolean isManual = true;
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(30)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(15.1);
  public static final boolean isDebug = true;
  
  public static final class Coral
  { public static final int CPR_TALON = 2048;
    public static final double MOTOR_GEAR_RATIO = 1.0;
    public static final int LEFT_ELEVATOR_ID = 9;
    public static final int RIGHT_ELEVATOR_ID = 14;
    public static final int BUCKET_ID = 15;
    public static final double KP = 0.0666;
    public static final double KI = 0.00002;
    public static final double KD = 0.0010; 
    public static double L1_POS;
    public static double L2_POS;
    public static double L3_POS;
    public static double L4_POS;
    public static final double ELEVATOR_SPEED = 0.25;
    
  }
  
  public static final class climberConstants
  {
    public static final int KERKLUNK_PORT = 0; //placehodlr for when we get actual port
    public static final int WINCH_MOTOR_ID = 10;
    public static final double WINCH_SPEED = 1;
  }
  
  // Maximum speed of the robot in meters per second, used to limit acceleration.
    public static final class drivingConstants
    {         /** The Left Joystick ID (typically 0) */
      public static final int LEFT_STICK_ID = 0;
      /** The Right Joystick ID (typically 1) */
      public static final int  RIGHT_STICK_ID = 1;
      /** The Xbox Controller (typically 2) */
      public static final int XBOX_ID = 2; 

    }
//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }


  public static final class DrivebaseConstants
  {

    /** Hold time on motor brakes when disabled*/
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    /**  Joystick Deadband */
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
/** AE = Algae Elevator
 * 
*/

public static class AE {
  public static final int ROTATE_MOTOR_ID = 16;
  public static final int OUTER_MOTOR_ID = 17;
  public static final double AE_SPEED = 0.05;
  public static final double POSITION_TOLERANCE = 0.02;
  public static final double kP = 1.0;
  public static final double kI = 0.2;
  public static final double kD = 0.1;


}

  public static class Algae {
    public static final int LEFT_MOTOR_ID = 12;  // Set to your left motor's CAN ID
    public static final int RIGHT_MOTOR_ID = 13; // Set to your right motor's CAN ID
    public static final int ALGAE_MOTOR_ID = 11; // Set to your motor id for the thing that moves algae up and down i gues
    public static final double ALGAE_SPEED = 0.40; //motorspeed
    public static final double POSITION_TOLERANCE = 0.02;
    public static final double kP = 1.0;
    public static final double kI = 0.2;
    public static final double kD = 0.1;
  

  } 
  public static class ElevatorConstants {
    public static final double kElevatorGearing = 35.0; // Gear ratio
    public static final double kCarriageMass = 5.0; // Mass of the elevator carriage (kg), adjust as needed
    public static final double kElevatorDrumRadius = 0.02; // Drum radius (meters), adjust based on real mechanism
    public static final double kMinElevatorHeightMeters = 0.0; // Minimum elevator height in meters
    public static final double kMaxElevatorHeightMeters = 2.4384; // 8 feet in meters (8 * 0.3048)
    public static final double ELEVATOR_TOLERANCE = 0.02;
    public static final double ELEVATOR_SPEED = 0.15;
    public static final double kElevatorRampRate = 0.1;

    // Conversion factors
    public static final double kRotaionToMeters = (2 * Math.PI * kElevatorDrumRadius) / kElevatorGearing;
    public static final double kRPMtoMPS = kRotaionToMeters / 60.0;

    // PID constants (tune these)
    public static final double kElevatorKp = 0.5; // wpi recommened 
    public static final double kElevatorKi = 0.0; 
    public static final double kElevatorKd = 0.0;

    // Motion profiling constraints
    public static final double kElevatorMaxVelocity = Meters.of(1.5).per(Second).in(MetersPerSecond); // meters per second TO DO:need to do actual set up for mps
    public static final double kElevatorMaxAcceleration = Meters.of(1.0).per(Second).per(Second).in(MetersPerSecondPerSecond); // meters per second^2 TO DO: HOW DID WE CALCULATE THIS?

    // Feedforward constants (tune these)
    public static final double kElevatorkS = 0.2; // Static friction voltage
    public static final double kElevatorkG = 2.28; // Gravity feedforward (wpi recommended)
    public static final double kElevatorkV = 3.07; // Velocity feedforward (wpi recommeneded)
    public static final double kElevatorkA = 0.41; // Acceleration feedforward (wpi recommended)
    public static final double kElevatorDefaultTolerance = 0.2;
    public static double kMaxVelocity;

  }



}
