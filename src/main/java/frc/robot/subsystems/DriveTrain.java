// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.frontLeft, "DriveBase");
  private WPI_TalonFX frontRight = new WPI_TalonFX(Constants.frontRight, "DriveBase");
  private WPI_TalonFX rearLeft = new WPI_TalonFX(Constants.rearLeft, "DriveBase");
  private WPI_TalonFX rearRight = new WPI_TalonFX(Constants.rearRight, "DriveBase");

  private AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private double inchConversion = Constants.encoderInchConversion;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    rearLeft.set(ControlMode.Follower, frontLeft.getDeviceID());
    rearRight.set(ControlMode.Follower, frontRight.getDeviceID());
    frontRight.setInverted(TalonFXInvertType.Clockwise);
    rearRight.setInverted(InvertType.FollowMaster);
    frontLeft.setInverted(TalonFXInvertType.CounterClockwise);
    rearLeft.setInverted(InvertType.FollowMaster);
   

    SupplyCurrentLimitConfiguration motorlimit = 
    new SupplyCurrentLimitConfiguration(true,  30, 35,  0.25);

    StatorCurrentLimitConfiguration statorLimit = new StatorCurrentLimitConfiguration(true, 60, 60, .2);



    rearLeft.configSupplyCurrentLimit(motorlimit);
    frontLeft.configSupplyCurrentLimit(motorlimit);
    rearRight.configSupplyCurrentLimit(motorlimit);
    frontRight.configSupplyCurrentLimit(motorlimit);


    frontLeft.configStatorCurrentLimit(statorLimit);
    rearLeft.configStatorCurrentLimit(statorLimit);
    frontRight.configStatorCurrentLimit(statorLimit);
    rearRight.configStatorCurrentLimit(statorLimit);
    
     
   
  
  

    frontLeft.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    frontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
  }
    
  
  public void drive(double moveLeft, double moveRight) {
      frontLeft.set(moveLeft);
      frontRight.set(moveRight);
    } 

    public double getLeftEncoder(){
      double leftEncoder = frontLeft.getSelectedSensorPosition();
      leftEncoder = -leftEncoder / inchConversion;
      SmartDashboard.putNumber("LeftEncoder16", leftEncoder);

      return leftEncoder;
    }

    public double getRightEncoder(){
        double rightEncoder = frontRight.getSelectedSensorPosition();
        rightEncoder = -rightEncoder / inchConversion;
        SmartDashboard.putNumber("RightEncoder16", rightEncoder);

        return rightEncoder;

    }

    public double robotAngle(){
      return ahrs.getAngle();
    }


  @Override
  public void periodic() {
    //double angle = ahrs.getAngle();
    //SmartDashboard.putNumber("RobotAngle", angle);

    //SmartDashboard.putNumber("LeftEncoder16", frontLeft.getSelectedSensorPosition());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
