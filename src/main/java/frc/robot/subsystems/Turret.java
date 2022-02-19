// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {
  
  WPI_TalonFX turretMotor;
  
  
  
  /** Creates a new Turret. */
  public Turret() {

    turretMotor = new WPI_TalonFX(Constants.turret_motor_port);
    // set inverted incase we need to reverse the direction of the motor
    //turretMotor.configForwardSoftLimitThreshold(forwardSensorLimit)   // Set forward Limit
    // Set FeebackDevice and configure default settings
    // Set Reverse Limit
    // Enable forward and reverse soft limits

    //Configure Kp, Ki, Kd of the turret
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Set PercentOutput Method take in a speed argument


  // Reset Encoder Method (we may need to take in a position to set the turret at dependant upon which auto we are running)


  // ticksToDegrees Method Take in Ticks Return Degrees
      // ticks / ticksPerRevFalcon / gear Ratio * 360 (I think this math is right)  Return this number

  
  // DegressToTicks Method
      // Degrees / 360 * GearRatio * TicksPerRevFalcon (I think this math is right)  Return this number
      

  // We may need a set Position Method (we will need this so if soft limit is hit then then we can set the potion all the way around to other side)
      // Will need to take in degrees and run through degreesToTicks then set the position to that number

  

}
