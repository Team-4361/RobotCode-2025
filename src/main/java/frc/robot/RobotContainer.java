// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.algae.AlgaeExtrudeCommand;
import frc.robot.commands.algae.AlgaeSuckCommand;
import frc.robot.commands.algae.AlgaeDownCommand;
import frc.robot.commands.algae.AlgaeUpCommand;
import frc.robot.commands.coral.BucketAuto;
import frc.robot.commands.coral.ElevatorDownCommand;
import frc.robot.commands.coral.ElevatorUpCommand;
import frc.robot.commands.coral.ReleaseCoralCommand;
import frc.robot.commands.coral.SensorCommand;
import frc.robot.commands.coral.elevatorPosUp;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.algaesubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.KerklunkSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.commands.climber.KerklunkCommand;
import frc.robot.commands.climber.WinchUpCommand;
import frc.robot.commands.climber.WinchDownCommand;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
 
  final CommandJoystick joystickL = new CommandJoystick(Constants.drivingConstants.LEFT_STICK_ID);
  final CommandJoystick joystickR = new CommandJoystick(Constants.drivingConstants.RIGHT_STICK_ID);
  final CommandXboxController driverXbox = new CommandXboxController(Constants.drivingConstants.XBOX_ID);
  // The robot's subsystems and commands are defined here...
  private final KerklunkSubsystem kerklunk = new KerklunkSubsystem();  
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();
  private final BucketSubsystem bucket = new BucketSubsystem();
  private final algaesubsystem algae = new algaesubsystem();
  private final WinchSubsystem winch = new WinchSubsystem();
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                   "swerve/neo"));
  private SendableChooser<Command> autoChooser;
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> joystickL.getY(),//was multiplied by -1
                                                                () -> joystickL.getX())
                                                            .withControllerRotationAxis(() -> joystickR.getZ())
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(joystickL::getX,
                                                                                             joystickR::getY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -joystickL.getY(),
                                                                        () -> joystickL.getX())
                                                                    .withControllerRotationAxis(() -> joystickR.getX())
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  joystickL.getRawAxis(
                                                                                                                      0) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  joystickL.getRawAxis(
                                                                                                                      0) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

   private final Command teleopFlightDriveCommand = drivebase.driveFieldOriented(
    SwerveInputStream.of(
        drivebase.getSwerveDrive(),
        () -> -joystickL.getY(),  // Forward/Backward
        () -> -joystickL.getX()   // Left/Right
    )
    .withControllerRotationAxis(() -> -joystickR.getTwist()) // Rotation using right stick twist
    .deadband(OperatorConstants.DEADBAND) // Apply deadband as a setting
    .scaleTranslation(0.8)
    .allianceRelativeControl(true)
);

  public RobotContainer()
  {
    NamedCommands.registerCommand("ElevatorUp", new ElevatorUpCommand(elevator));
    NamedCommands.registerCommand("ElevatorDown", new ElevatorDownCommand(elevator));
    NamedCommands.registerCommand("ExtrudeCoral", new BucketAuto(bucket));
    NamedCommands.registerCommand("L2", new elevatorPosUp(elevator, Constants.Coral.L2_POS, 0.20));

    // Configure the trigger bindings
    configureBindings();
    bucket.setDefaultCommand(new SensorCommand(bucket));

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    //autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    //Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    //Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        //driveDirectAngle);
    //Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    //Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        ///driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(teleopFlightDriveCommand);
    } else
    {
      drivebase.setDefaultCommand(teleopFlightDriveCommand);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    else
    {
      joystickL.button(12).onTrue((Commands.runOnce(drivebase::zeroGyro)));
      //driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
     /*driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ); */ 
      /*
      joystickL.button(4).whileTrue(
        drivebase.driveToPose(
          new Pose2d(6.001, 3.974, Rotation2d.fromDegrees(5.813)))
                        );
           */
      //joystickL.button(3).whileTrue(drivebase.driveToPose(new Pose2d(3.192, 1.934, new Rotation2d(4.07))));
      //joystickL.button(3).whileTrue(drivebase.driveToPose(new Pose2d(3.192, 1.934, new Rotation2d(10))));

            //Change X and Y values to change positon - This command all it does it move the robot to any position on the FIELD!!! 
      // X and Y values are in meters btw
      //In Radiants
      /*
       * 
       * ONLY UNCOMMENT WHEN VISION TESTING or after testing in an actual field!!!!!!!
       */

      //joystickL.button(2).whileTrue(drivebase.driveToPose(new Pose2d(1.197, 1.058, new Rotation2d(0.925025))));

      joystickL.button(4).whileTrue(new KerklunkCommand(kerklunk, 0.0));
      joystickL.button(6).whileTrue(new KerklunkCommand(kerklunk, 180.0));
      //joystickL.button(3).onTrue(new BucketAuto(bucket));
      //joystickL.button(3).whileTrue(new drivebase.driveToPose(new Pose2d(6.001, 3.974, rotation2d( 5.813))));
            //Xbox controller
            /* driverXbox.rightTrigger().whileTrue(new AlgaeExtrudeCommand(algae));
            driverXbox.leftTrigger().whileTrue(new AlgaeSuckCommand(algae));
            driverXbox.a().whileTrue(new AlgaeDownCommand(algae));
            driverXbox.b().whileTrue(new elevatorPosUp(elevator, 97.2, 1));
            driverXbox.x().whileTrue(new elevatorPosUp(elevator, 46, 1));
            */
          //Keypad
           //Remember to make sure the light is on (press the button labeled "On/Off" to toggle); Num Lock button
            driverXbox.rightTrigger().whileTrue(new AlgaeExtrudeCommand(algae)); //Subtraction button
            driverXbox.leftTrigger().whileTrue(new AlgaeSuckCommand(algae)); //Plus button
            driverXbox.povLeft().whileTrue(new AlgaeUpCommand(algae)); //o button
            driverXbox.povRight().whileTrue(new AlgaeDownCommand(algae)); //oo button
            driverXbox.leftStick().whileTrue(new KerklunkCommand(kerklunk, 0.0)); //7 button
            driverXbox.rightStick().whileTrue(new KerklunkCommand(kerklunk, 180.0)); //8 button

       //1 button
            //driverXbox.b().whileTrue(new elevatorPosUp(elevator,Constants.Coral.L2_POS, 0.25)); // 2 button
            driverXbox.a().whileTrue(new elevatorPosUp(elevator, Constants.Coral.L2_POS, 0.70));
            driverXbox.x().whileTrue(new elevatorPosUp(elevator, Constants.Coral.L3_POS, 0.70)); // 3 button
            driverXbox.y().whileTrue(new elevatorPosUp(elevator, Constants.Coral.L4_POS, 0.70));
            driverXbox.b().whileTrue(new ReleaseCoralCommand(bucket)); 
            //driverXbox.y().whileTrue(new elevatorPosUp(elevator, Constants.Coral.L4_POS, 1));// 4 button
            driverXbox.povDown().whileTrue(new ElevatorDownCommand(elevator)); // 9 button
            driverXbox.povUp().whileTrue(new ElevatorUpCommand(elevator)); //multiply (*) button
            driverXbox.rightBumper().whileTrue(new WinchUpCommand(winch)); //Enter button
            driverXbox.leftBumper().whileTrue(new WinchDownCommand(winch));//Decimal (.) button
            }
          }
      
        
      
        /*private Rotation2d rotation2d(double d) {
          // TODO Auto-generated method stub
          throw new UnsupportedOperationException("Unimplemented method 'rotation2d'");
        }
      
        /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("GET_OUT");
  }*/
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}