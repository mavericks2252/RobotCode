// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
/*import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;*/
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Shooter extends SubsystemBase {
  WPI_TalonFX shooterMotorMaster;
  WPI_TalonFX shooterMotorSlave;
  WPI_TalonFX aWheel;
  //Solenoid shooterSolenoid;

  /** Creates a new Shooter. */
  public Shooter() {
//Master
    shooterMotorMaster = new WPI_TalonFX(Constants.shooter_motor_master_port);
    shooterMotorMaster.configFactoryDefault();
    shooterMotorMaster.setInverted(true);
    shooterMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

//Slave
    shooterMotorSlave = new WPI_TalonFX(Constants.shooter_motor_slave_port);
    shooterMotorSlave.configFactoryDefault();
    shooterMotorSlave.follow(shooterMotorMaster);
    shooterMotorSlave.setInverted(false);

//Accelerator 
    aWheel = new WPI_TalonFX(Constants.accelerator_wheel_motor_port);
    aWheel.setInverted(true);

   // shooterSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.shooter_solenoid_channel);
   // shooterSolenoid.set(false);

    shooterMotorMaster.config_kF(0, Constants.shooter_kF, Constants.shooter_timeout_ms);
    shooterMotorMaster.config_kP(0, Constants.shooter_kP, Constants.shooter_timeout_ms);
    shooterMotorMaster.config_kI(0, Constants.shooter_kI, Constants.shooter_timeout_ms);
    shooterMotorMaster.config_kD(0, Constants.shooter_kD, Constants.shooter_timeout_ms);
    shooterMotorMaster.config_IntegralZone(0, Constants.shooter_IZone, Constants.shooter_timeout_ms);
    shooterMotorMaster.configClosedloopRamp(Constants.shooter_Ramp_Time, Constants.shooter_timeout_ms);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
  public void shooterMasterPercentOutput(double sSpeed){

    shooterMotorMaster.set(ControlMode.PercentOutput, sSpeed);
  
  }

  public void setShooterVelocity(double RPMs){

  shooterMotorMaster.set(ControlMode.Velocity, rPMsToVelocity(RPMs));    

  }

  public double velocityToRPMs(double velocity){

    return velocity / Constants.Falcon_Ticks_Per_Rev * 600;

  }

  public double rPMsToVelocity(double rpms){

    return rpms * Constants.Falcon_Ticks_Per_Rev / 600;

  }

  public double getRPMs(){

    return velocityToRPMs(shooterMotorMaster.getSelectedSensorVelocity());

  }

  public void stop(){

    shooterMotorMaster.stopMotor();
   

  }

  // set shooterHoodUp Method
  

  // set ShooterHoodDown Method



}
