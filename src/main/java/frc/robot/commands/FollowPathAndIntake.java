// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FollowPathAndIntake extends ParallelDeadlineGroup {
  /** Creates a new FollowPathAndIntake. */
  public FollowPathAndIntake(Trajectory trajectory, DriveTrain driveTrain, Intake intake) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().

   
    super(new InstantCommand(), 
    new RamseteCommand (
      trajectory, // trajectory
      driveTrain::getPose2d, // Pose Supplier
      new RamseteController(Constants.k_ramesete_b,Constants.k_ramesete_zeta),// object does path following conputation and converts to chassis speed
      new SimpleMotorFeedforward(Constants.ks_volts, 
                                 Constants.kv_volts_seconds_per_meter, 
                                 Constants.ka_volts_seconds_squared_per_meter), 
  
      Constants.k_drive_kinematics, // accounts for wheel space 
      driveTrain::getWheelSpeeds, // the wheel speed supplier to the controller
      new PIDController(Constants.kp_drive_velocity,0, 0), // left side PID controller  
      new PIDController(Constants.kp_drive_velocity, 0, 0),// right side PID controller  
      driveTrain::tankDriveVolts,//passing output voltage to drivetrain  
      driveTrain)

      );
    // addCommands(new FooCommand(), new BarCommand());
      addCommands(new RunIntake(intake));
  }
}
