// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Intake extends SubsystemBase {
  //CANSparkMax intakeMotor;
  Solenoid intakeSolenoid;
  /** Creates a new Intake. */
  public Intake() {
   // intakeMotor = new CANSparkMax(Constants.intake_motor_port, MotorType.kBrushless);
    //intakeMotor.setInverted(false);
    //intakeMotor.restoreFactoryDefaults();

    //intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.intake_solenoid_channel);
    //intakeSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
/*
 public void extendIntake(){

  intakeSolenoid.set(true);

 }

public void retractIntake(){

 intakeSolenoid.set(false);

}

  public void intakeSpeed(double iSpeed){

    intakeMotor.set(iSpeed);

  }

  public void stop(){

    intakeMotor.stopMotor();
    retractIntake();
  }

*/
}
