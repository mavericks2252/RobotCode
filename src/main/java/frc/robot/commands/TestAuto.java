// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  DriveTrain driveTrain;
  /** Creates a new TestAuto. */
  public TestAuto(DriveTrain dT, Trajectory trajectory, Intake intake) {
    driveTrain = dT;
    
   
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      
    
      new ParallelRaceGroup(    //  You can just use   race()  inplace of importing ParallelRaceGroup and creating a new command
        RobotContainer.startShooter, 
        new WaitCommand(3) 
        
      ), 

      new FollowPathAndIntake(trajectory, driveTrain, intake)
      
    
      );
  }
}
