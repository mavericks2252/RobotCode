// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {

  WPI_TalonFX hopperMotor;
  WPI_TalonFX feedWheel;
  DigitalInput ballSensor;




  /** Creates a new Hopper. */
  public Hopper() {

    hopperMotor = new WPI_TalonFX(Constants.hopper_motor_port);
    feedWheel = new WPI_TalonFX(Constants.feed_wheel_motor_port);
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
