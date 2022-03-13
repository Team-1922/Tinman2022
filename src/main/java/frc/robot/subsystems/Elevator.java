// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//change name to transfer at some point 
// oops, too late 

// why is there a different subsystem called transfer?
// we cant rename this to transfer because it is allready used
// and it would be even more confusing to code -_-
package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private double encoder;
  private XboxController xbox;
  private double posGoal;
  
  private WPI_TalonFX motor1 = new WPI_TalonFX(Constants.elevator1);
  private WPI_TalonFX motor2 = new WPI_TalonFX(Constants.elevator2);
  private Solenoid ElevatorSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);

  private NetworkTable ozram = NetworkTableInstance.getDefault().getTable("OzRam");
  //private Solenoid ElevatorSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0); //THE 1 IS THE PORT ON THE PCM, MAYBE CHANGE

  private double goal = 2048 * 300;

  /** Creates a new elevator. */
  public Elevator() {
    motor1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);

    motor2.set(ControlMode.Follower, motor1.getDeviceID());

    motor1.configMotionCruiseVelocity(35000);
    motor1.configMotionAcceleration(15000);
    motor1.configMotionSCurveStrength(4);

    


  }
  
  @Override
  public void periodic() {
    encoder = motor1.getSelectedSensorPosition();
    SmartDashboard.putNumber("ElevatorEncoder", encoder);
    // This method will be called once per scheduler run
    
    
  }

  

public void moveUp(double mode){
  if (mode == 1){
    posGoal = 2048 * ozram.getEntry("ElevatorRotations").getDouble(10);
  } else{
    posGoal = 53.5 * 2048;
  }


  motor1.set(ControlMode.MotionMagic, posGoal);
}

public void moveDown(double mode){
  if (mode == 1){
   posGoal = 0; 
  } else {
    posGoal = 20 * 2048;
  }
  

  motor1.set(ControlMode.MotionMagic, posGoal);
}

public void stopMovement(){
  motor1.set(ControlMode.PercentOutput, 0);
}

public double trajectory(){
  return motor1.getActiveTrajectoryPosition();
}

public double posGoal(){
  return posGoal;
}




 public void brakeUp(){
  ElevatorSolenoid.set(false);

   //SmartDashboard.putBoolean("BrakeState", ElevatorSolenoid.get());
 }


 public void brakeDown(){
 ElevatorSolenoid.set(true);

 // SmartDashboard.putBoolean("BrakeState", ElevatorSolenoid.get());
}


 public double getElevatorEncoder(){

   return encoder;
 }


}
