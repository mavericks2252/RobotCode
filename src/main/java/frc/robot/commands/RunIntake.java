// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class RunIntake extends CommandBase {
Intake intake;


  /** Creates a new RunIntake. */
  public RunIntake(Intake i) {
  intake = i;
  addRequirements(intake);
  SmartDashboard.putBoolean("Intake Status", false); 
  
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SmartDashboard.putBoolean("Intake Status", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //intake.intakeSpeed(Constants.intakeSpeed);
   // intake.extendIntake();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    //intake.stop();
    //intake.retractIntake();
    SmartDashboard.putBoolean("Intake Status", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
