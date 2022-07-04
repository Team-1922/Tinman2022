// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class goToBall extends CommandBase {
  Trajectory m_trajectorytest;
  Trajectory.State targetPose;
  DriveTrain m_driveTrain;
  Pose2d actualPose;
  Pose2d getBall;

  Translation2d translation;
  Transform2d transform;
  
  RamseteController m_controller;

  double startTime;
  double newTime;

  double m_feet;
  boolean m_reversed;

  /*
   * @param m_driveTrain
   * 
   * @param m_goToBall
   */
  public goToBall(DriveTrain driveTrain, double feet, boolean reversed) {
    m_driveTrain = driveTrain;
    m_feet = feet;
    m_reversed = reversed;

    m_controller = new RamseteController();
 

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();

    Pose2d autoStart = m_driveTrain.getPose();
  // x position
  // y position
  // , heading or angle
  //
/*  getBall = new Pose2d(
      Units.feetToMeters(m_feet),
      Units.feetToMeters(0),
      Rotation2d.fromDegrees(0));
*/
translation = new Translation2d(Units.feetToMeters(m_feet), Units.feetToMeters(0));
transform = new Transform2d(translation, //autoStart.getRotation());
Rotation2d.fromDegrees(.01));

getBall = autoStart.transformBy(transform);

 // var interiorWaypoints = new ArrayList<Translation2d>();
  var waypoints = new ArrayList<Pose2d>();
  waypoints.add(autoStart);
  waypoints.add(getBall);
  //interiorWaypoints.add(new Translation2d(Units.feetToMeters(feet / 2), Units.feetToMeters(1)));

  // start velocity , end velocity , reversed , constraints
  TrajectoryConfig config = new TrajectoryConfig(.75, .75);
  config.setReversed(m_reversed);

  m_trajectorytest = TrajectoryGenerator.generateTrajectory(

      waypoints,
      config

  );
  
  SmartDashboard.putNumber("TargetPoseX", getBall.getX());
  SmartDashboard.putNumber("TargetPoseO", getBall.getRotation().getDegrees());

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
newTime = System.currentTimeMillis() - startTime;

targetPose = m_trajectorytest.sample(newTime / 1000.0);
actualPose = m_driveTrain.getPose();

ChassisSpeeds speeds = m_controller.calculate(actualPose, targetPose);

m_driveTrain.kinematicsDrive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.velocityDrive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(getBall.getX() - actualPose.getX()) <= .1);
  }
}
