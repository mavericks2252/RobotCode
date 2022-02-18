// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {

  WPI_TalonFX climberMotor;
  Solenoid climberSolenoid;
  Solenoid hrSolenoid;
  /** Creates a new Climber. */
  public Climber() {


   climberMotor = new WPI_TalonFX(Constants.climber_motor_port);
    climberSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.climber_solenoid_channel);
    hrSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.hr_solenoid_channel);
    climberSolenoid.set(false);
    hrSolenoid.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
