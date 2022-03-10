// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transfer extends SubsystemBase {

  private WPI_TalonFX sides = new WPI_TalonFX(Constants.transferSides);
  private WPI_TalonSRX front = new WPI_TalonSRX(Constants.transferFront);
  private WPI_TalonSRX rear = new WPI_TalonSRX(Constants.transferRear);
  /** Creates a new Transfer. */
  public Transfer() {
    sides.setNeutralMode(NeutralMode.Coast);
    front.setNeutralMode(NeutralMode.Coast);
    rear.setNeutralMode(NeutralMode.Coast);


    front.configPeakCurrentLimit(120);
    rear.configPeakCurrentLimit(120);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("SidesVelocity", sides.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }

  

  public void sideMotor(double speed){
    sides.set(ControlMode.Velocity, speed);
  }

  public void frontMotor(double speed){
    front.set(speed);
  }

  public void rearMotor(double speed){
    rear.set(speed);
  }

  public double transferVelocity(){

    return sides.getSelectedSensorVelocity();

  }

}
