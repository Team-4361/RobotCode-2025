// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.algae.AlgaeDownCommand;
import frc.robot.commands.algae.AlgaeExtrudeCommand;
import frc.robot.commands.algae.AlgaeSuckCommand;
import frc.robot.commands.algae.AlgaeUpCommand;
import frc.robot.commands.climber.KerklunkCommand;
import frc.robot.commands.coral.BucketMoveToPosition;
import frc.robot.commands.coral.BucketMoveToPosition;
import frc.robot.commands.coral.MoveElevatorPos;
import frc.robot.commands.coral.L2Move;
import frc.robot.subsystems.BucketSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.KerklunkSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.subsystems.algaesubsystem;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AlignToAprilTagCommand;
import frc.robot.subsystems.swervedrive.AprilTagAligner;

import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
 
  final CommandJoystick joystickL = new CommandJoystick(0);
  final CommandJoystick joystickR = new CommandJoystick(1);
  final CommandXboxController driverXbox = new CommandXboxController(Constants.drivingConstants.XBOX_ID);



  private final AprilTagAligner tagAligner = new AprilTagAligner("YourCameraName", swerve);

  public static SwerveSubsystem swerve = new SwerveSubsystem(null);
  public static algaesubsystem algae = new algaesubsystem();
  public static ElevatorSubsystem elevator = new ElevatorSubsystem();
  public static BucketSubsystem bucket = new BucketSubsystem();
  private final WinchSubsystem winchSubsystem = new WinchSubsystem();
  public static KerklunkSubsystem kerklunk = new KerklunkSubsystem(); 
  // The robot's subsystems and commands are defined here...


  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));
  
  
                                                                                

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                //() -> driverXbox.getLeftY() * -1,
                                                                () -> joystickL.getY() * -1,
                                                                () -> joystickL.getX() * -1)
                                                            .withControllerRotationAxis(joystickR::getZ)
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
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    //NamedCommands.registerCommand("ElevatorL", new ElevatorDownCommand(elevator));
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

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      joystickL.button(11).onTrue((Commands.runOnce(drivebase::zeroGyro)));

      joystickL.button(12).whileTrue(new AlignToAprilTagCommand(swerve));



      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      joystickL.button(11).onTrue((Commands.runOnce(drivebase::zeroGyro)));

      joystickL.button(12).whileTrue(new AlignToAprilTagCommand(swerve));

      winchSubsystem.setDefaultCommand(
          Commands.run(() -> {
              double speed = -driverXbox.getLeftY(); // Inverts Y-axis for natural control
              winchSubsystem.setWinchSpeed(speed);
          }, winchSubsystem)
      );
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
          //test
      driverXbox.povLeft().whileTrue(new BucketMoveToPosition(bucket, -5)); // Move bucket to -45 degrees
      driverXbox.povRight().whileTrue(new BucketMoveToPosition(bucket, 5)); // Move bucket to 45 degrees
    driverXbox.leftTrigger().onTrue(new AlgaeSuckCommand(algae));
    driverXbox.rightTrigger().onTrue(new AlgaeExtrudeCommand(algae));
    //xbox.b().toggleOnTrue(m_autonomousCommand)     could use to toggle modes for certain control schemes?
    driverXbox.b().onTrue(new AlgaeUpCommand(algae));
    driverXbox.x().onTrue(new AlgaeDownCommand(algae));
    //driverXbox.a().onTrue(new KerklunkCommand(kerklunk, 90.0));
    //driverXbox.y().onTrue(new KerklunkCommand(kerklunk, 180.0));
    driverXbox.a().whileTrue(new MoveElevatorPos(elevator));
    driverXbox.y().whileTrue(new L2Move(elevator));
    driverXbox.povUp().whileTrue(new MoveElevatorPos(elevator));
    }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}