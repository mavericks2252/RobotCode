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
    
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
