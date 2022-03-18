// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transfer extends SubsystemBase {

  private WPI_TalonFX sides = new WPI_TalonFX(Constants.transferSides);
  private WPI_TalonFX front = new WPI_TalonFX(Constants.transferFront);
  private WPI_TalonFX rear = new WPI_TalonFX(Constants.transferRear);
  /** Creates a new Transfer. */
  public Transfer() {

    front.configFactoryDefault();
    front.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    front.config_kF(0, 0.06, 30);

    rear.configFactoryDefault();
    rear.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30);
    rear.config_kF(0, 0.06, 30);

    sides.configFactoryDefault();
    sides.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 30); 
    sides.config_kF(0, 0.06, 30);



    sides.setNeutralMode(NeutralMode.Coast);
    front.setNeutralMode(NeutralMode.Coast);
    rear.setNeutralMode(NeutralMode.Coast);

    SupplyCurrentLimitConfiguration transferLimit = new SupplyCurrentLimitConfiguration(true, 20, 25, 0.25);

    front.configSupplyCurrentLimit(transferLimit);
    rear.configSupplyCurrentLimit(transferLimit);
 
  
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("SidesVelocity", sides.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }

  

  public void sideMotor(double speed){
   sides.set(TalonFXControlMode.Velocity, speed);
  }

  public void frontMotor(double speed){
    front.set(TalonFXControlMode.Velocity, -speed);
  }

  public void rearMotor(double speed){
    rear.set(TalonFXControlMode.Velocity, speed);
  }

  public double transferVelocity(){

    return 1;//sides.getSelectedSensorVelocity();

  }

}
