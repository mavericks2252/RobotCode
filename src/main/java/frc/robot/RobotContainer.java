// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DoNothing;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.RunIntake;
import frc.robot.commands.StartShooter;
import frc.robot.commands.TestAuto;
import frc.robot.commands.VisionTracking;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public static Shooter shooter;
  public static StartShooter startShooter;

  public static Climber climber;

  public static DriveTrain driveTrain;
  public static DoNothing doNothing;
  public static ManualDrive manualDrive;

  public static Intake intake;
  public static RunIntake runIntake;

  public static Vision vision;
  public static VisionTracking visionTracking;

  public static XboxController driverController;
  public static XboxController opereratorController;

  public static PowerDistribution pdh;
  public static PneumaticHub pneumaticHub;

 
  

  
 
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //Robot SubSystems
      driveTrain = new DriveTrain();
      shooter = new Shooter();
      climber = new Climber();
      intake = new Intake();
      vision = new Vision();
     
    //Controlers
      driverController = new XboxController(Constants.driver_controller_port);
      opereratorController = new XboxController(Constants.operator_controller_port);

    // Commands I dont think we need these instances if when we call the button press and other things we create a new instance of the command anyways
      //runIntake = new RunIntake(intake);
      //startShooter = new StartShooter(shooter);
      //visionTracking = new VisionTracking(vision, driveTrain);
      manualDrive = new ManualDrive(driveTrain);   

    // AutoCommands  - See commands comment
      // doNothing = new DoNothing(driveTrain);

    //Set Subsystem Default Commands here
      //driveTrain.setDefaultCommand(manualDrive);
   
    //Create instances of RobotHardware
      pdh = new PowerDistribution(Constants.pdh_port,ModuleType.kRev);
      pneumaticHub = new PneumaticHub(Constants.pneumatic_hub_port);

    // Sendable Chooser here for autoplay Set default option ot DoNothing and add options for all other Auto Plays
        //Each option you add is a new instance of the autoCommands (therfore you dont need to create them above as we had before)
    
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //JoystickButton shootButton = new JoystickButton(opereratorController, Constants.xButton);
    //shootButton.whileHeld(new StartShooter(shooter));

    /*JoystickButton intakeButton = new JoystickButton(opereratorController, Constants.yButton);
    intakeButton.whenPressed(new RunIntake(intake));
    */
    JoystickButton visionTrackButton = new JoystickButton(driverController, Constants.aButton);
    visionTrackButton.whileHeld(new VisionTracking(vision, driveTrain));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Return the getSendableChooser // This will send the autoCommand Selected in the SmartDashboard

     // Get rid of this and add to the FollowPathAndIntake Command Goup

     driveTrain.resetOdemetry(Robot.testTrajectory.getInitialPose());
    
    return new TestAuto(driveTrain, Robot.testTrajectory, intake);// Remove this line of code in the end.

  }
}

//Extra Code to be deleted if not needed

// Create a voltage constraint to ensure we don't accelerate too fast
 /* var autoVoltageConstraint = 
      new DifferentialDriveVoltageConstraint(
          new SimpleMotorFeedforward(
                Constants.ks_volts,
                Constants.kv_volts_seconds_per_meter,
                Constants.ka_volts_seconds_squared_per_meter)  ,
                Constants.k_drive_kinematics, 
                Constants.drivetrain_max_voltage);

    TrajectoryConfig config = 
        new TrajectoryConfig(Constants.k_max_speed_meters_per_second, 
                         Constants.k_max_acceleration_meters_per_second_squared)
                         //Add kinematics to ensure max speed is obeyed
                         .setKinematics(Constants.k_drive_kinematics)
                        //Apply the voltage constraint
                         .addConstraint(autoVoltageConstraint);

    
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction 
    new Pose2d(0,0,new Rotation2d(0)), 
    // Pass through these interiorvWaypoints making an s curve path 
    List.of(
      new Translation2d(2,1),
      new Translation2d(4,-1)
    ),  
    // End position 
    new Pose2d(6,0,new Rotation2d(0)),
    // Pass config
    config);

*/