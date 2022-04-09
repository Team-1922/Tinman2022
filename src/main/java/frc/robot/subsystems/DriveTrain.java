// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveTrain extends SubsystemBase {
  private String canBus// = "DriveBase";
   = "";
  private WPI_TalonFX frontLeft = new WPI_TalonFX(Constants.frontLeft, canBus);
  private WPI_TalonFX frontRight = new WPI_TalonFX(Constants.frontRight, canBus);
  private WPI_TalonFX rearLeft = new WPI_TalonFX(Constants.rearLeft, canBus);
  private WPI_TalonFX rearRight = new WPI_TalonFX(Constants.rearRight, canBus);



  private AHRS m_ahrs;
  private DifferentialDriveOdometry m_odometry;

  private Pose2d pose;

  private double inchConversion = Constants.encoderInchConversion;

  /** Creates a new ExampleSubsystem. */
  public DriveTrain(AHRS ahrs, DifferentialDriveOdometry odometry) {

    m_ahrs = ahrs;
    m_odometry = odometry;

    rearLeft.set(ControlMode.Follower, frontLeft.getDeviceID());
    rearRight.set(ControlMode.Follower, frontRight.getDeviceID());
    frontRight.setInverted(TalonFXInvertType.Clockwise);
    rearRight.setInverted(InvertType.FollowMaster);
    frontLeft.setInverted(TalonFXInvertType.CounterClockwise);
    rearLeft.setInverted(InvertType.FollowMaster);
   

    SupplyCurrentLimitConfiguration motorlimit = 
    new SupplyCurrentLimitConfiguration(true,  30, 55,  0.25);

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
    frontLeft.config_kF(0, .06, 30);

    frontRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    frontRight.config_kF(0, .06, 30);
  }
    
  
  public void drive(double moveLeft, double moveRight) {
      frontLeft.set(moveLeft);
      frontRight.set(moveRight);
    } 

    public void velocityDrive(double speed){
      frontLeft.set(ControlMode.Velocity, speed);
      frontRight.set(ControlMode.Velocity, speed);
    }



    public void kinematicsDrive(double vx, double vy, double omega){
      DifferentialDriveKinematics robotWidth = new DifferentialDriveKinematics(Units.inchesToMeters(18));

      ChassisSpeeds chassisSpeed = new ChassisSpeeds(vx, vy, omega);
    
      DifferentialDriveWheelSpeeds wheelSpeed = robotWidth.toWheelSpeeds(chassisSpeed);
      
      frontLeft.set(ControlMode.Velocity, wheelSpeed.leftMetersPerSecond * Constants.encoderMeterConversion);
      frontRight.set(ControlMode.Velocity, wheelSpeed.rightMetersPerSecond * Constants.encoderMeterConversion);

      //Velocity uses ticks/100ms and I think we need to convert meterspersecond to that
   

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
      return m_ahrs.getAngle();
    }

    public double motorVelocityLeft(){
      return frontLeft.getSelectedSensorVelocity();
    }

    public Pose2d getPose(){
      return pose;
    }


     

  @Override
  public void periodic() {
    pose = m_odometry.update(Rotation2d.fromDegrees(-m_ahrs.getAngle()), frontLeft.getSelectedSensorPosition(), frontRight.getSelectedSensorPosition());
 
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
