// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.frontLeft);
  private WPI_TalonFX frontRight = new WPI_TalonFX(Constants.frontRight);
  private WPI_TalonFX rearLeft = new WPI_TalonFX(Constants.rearLeft);
  private WPI_TalonFX rearRight = new WPI_TalonFX(Constants.rearRight);

  private double inchConversion = Constants.encoderInchConversion;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain() {
    rearLeft.set(ControlMode.Follower, frontLeft.getDeviceID());
    rearRight.set(ControlMode.Follower, frontRight.getDeviceID());
    frontRight.setInverted(TalonFXInvertType.CounterClockwise);
    rearRight.setInverted(InvertType.FollowMaster);
    frontLeft.setInverted(TalonFXInvertType.Clockwise);
    rearLeft.setInverted(InvertType.FollowMaster);

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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
