// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Test extends ParallelDeadlineGroup {
  /** Creates a new Test. */
  public Test() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new InstantCommand(), new RunIntake(RobotContainer.intake));
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new StartShooter(RobotContainer.shooter),
      new VisionTracking(RobotContainer.vision , RobotContainer.driveTrain),

      race(
        new ManualDrive(RobotContainer.driveTrain)
        
      )
    
    );
  }
}
