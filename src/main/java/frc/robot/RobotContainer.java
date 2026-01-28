// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.EjectorSubsystem;
// import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubSystem;
// import frc.robot.subsystems.Vision;
import swervelib.SwerveInputStream;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final SwerveSubSystem drivebase = new SwerveSubSystem();
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  // public final Elevator Sima = new Elevator();
  // private final EjectorSubsystem m_EjectorSubsystem = new EjectorSubsystem();
  // private final USBcamera camera = new UsbCamera(null);
  // public final Vision m_Vision = new Vision(drivebase, "limelight");
  public final Shooter Lebron_James = new Shooter();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // HashMap<String, Command> eventMap = new HashMap<>();

    // eventMap.put("RaiseElevator",Sima.moveTo(OperatorConstants.FOURTH_LEVEL));
    // eventMap.put("LowerElevator", Sima.moveTo(OperatorConstants.GROUND));
    // eventMap.put("Eject",m_EjectorSubsystem.Vomit(0.7));

    // NamedCommands.registerCommands(eventMap);
    // Configure the trigger bindings
    configureBindings();
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
  }

  public SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> (m_driverController.getLeftY() * -1 * OperatorConstants.presicionSpeed) * OperatorConstants.speedLimiter,
                                                                () -> (m_driverController.getLeftX() * -1 * OperatorConstants.presicionSpeed) * OperatorConstants.speedLimiter)
                                                                .withControllerRotationAxis(m_driverController::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);
  
  public SwerveInputStream driveDriectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                                                             .headingWhile(true);

  public Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDriectAngle);
  public Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

  SwerveInputStream driveAngularVelocitysim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                  () -> m_driverController.getLeftY() * -1* OperatorConstants.presicionSpeed,
                                                                  () -> m_driverController.getLeftX() * 1* OperatorConstants.presicionSpeed)
                                                                  .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
                                                                  .deadband(OperatorConstants.DEADBAND)
                                                                  .scaleTranslation(0.8)
                                                                  .allianceRelativeControl(true);
  
  SwerveInputStream driveDriectAnglesim = driveAngularVelocitysim.copy()
                                                                 .withControllerHeadingAxis(() -> Math.sin(
                                                                                                  m_driverController.getRawAxis(2) * Math.PI) *(Math.PI *2),
                                                                                            () -> Math.cos(
                                                                                              m_driverController.getRawAxis(2) * Math.PI) * (Math.PI *2))
                                                                 .headingWhile(true);

 Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDriectAnglesim);
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.start().and(m_driverController.back()).onTrue(drivebase.zeroGyro());
    // m_driverController.a().onTrue(Sima.moveTo(OperatorConstants.FIRST_LEVEL));
    // m_driverController.b().onTrue(Sima.moveTo(OperatorConstants.SECOND_LEVEL));
    // m_driverController.x().onTrue(Sima.moveTo(OperatorConstants.THIRD_LEVEL));
    m_driverController.leftTrigger(0.05)
    .whileTrue(new RunCommand(() -> Lebron_James.setSpeed(m_driverController.getLeftTriggerAxis()* 0.77)))
    .onFalse(new InstantCommand(() -> Lebron_James.setSpeed(0)));

    m_driverController.rightTrigger(0.05)
    .whileTrue(new RunCommand(() ->  Lebron_James.setSpeed(m_driverController.getRightTriggerAxis()* -0.77)))
    .onFalse(new InstantCommand(() ->  Lebron_James.setSpeed(0)));


    m_driverController.leftStick().
    whileTrue(new RunCommand(() -> Constants.OperatorConstants.presicionSpeed = Constants.OperatorConstants.precisionSpeedFinalValue))
    .onFalse(new InstantCommand(() -> Constants.OperatorConstants.presicionSpeed = 1));
    
  }
    
    
    // m_driverController.y().onTrue(Sima.moveTo(OperatorConstants.FOURTH_LEVEL));
    // m_driverController.leftBumper()
    //     .whileTrue(new RunCommand(() -> m_EjectorSubsystem.eject(m_driverController.getRightTriggerAxis()), m_EjectorSubsystem))
    //     .onFalse(new InstantCommand(() -> m_EjectorSubsystem.stop(), m_EjectorSubsystem));
   
    // m_driverController.rightStick().onTrue(Sima.moveTo(OperatorConstants.GROUND));


  //   m_driverController.y().onTrue(new InstantCommand(() -> {
  //     slowMode = !slowMode; // Toggle slow mode state
  //     if (slowMode) {
  //         Constants.OperatorConstants.presicionSpeed = 0.5; // Set to slower speed
  //     } else {
  //         Constants.OperatorConstants.presicionSpeed = 1.0; // Reset to normal speed
  //     }
  // }));

      
    

    

    // m_driverController.y().onTrue(m_Vision.getAlignToAprilTagCommand(drivebase));

    // m_driverController.y().onTrue(m_Vision.getAlignToAprilTagCommand(drivebase));

    // m_driverController.leftStick().onTrue(drivebase.driverobotoriented(new ChassisSpeeds(m_Vision.calculateSteer(), 0, 0)));

  //   m_driverController.rightBumper()
  //     .whileTrue(m_EjectorSubsystem.InvertedVomit(m_driverController.getLeftTriggerAxis())) // Runs while button is held
  //     .onFalse(new InstantCommand(() -> m_EjectorSubsystem.stop(), m_EjectorSubsystem));   // Stops action when button is released  
  // }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    return new PathPlannerAuto("TEST");
  }
}