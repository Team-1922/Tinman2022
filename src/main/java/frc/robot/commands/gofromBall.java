// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class gofromBall extends CommandBase {
  Trajectory m_trajectorytest;
  Trajectory.State targetPose;
  Pose2d actualPose;
  Pose2d getBall;
  DriveTrain m_driveTrain;
  RamseteController m_controller;

  double startTime;
  double newTime;

  /*
   * @param m_driveTrain
   * 
   * @param m_goToBall
   */
  public gofromBall(DriveTrain driveTrain, double feet) {
    m_driveTrain = driveTrain;

    m_controller = new RamseteController();


    var autoStart = new Pose2d(
        Units.feetToMeters(actualPose.getX()),
        Units.feetToMeters(actualPose.getY()),
        Rotation2d.fromDegrees(0));
    // x position
    // y position
    // , heading or angle
    //
    getBall = new Pose2d(
        Units.feetToMeters(actualPose.getX() -feet),
        Units.feetToMeters(actualPose.getY()),
        Rotation2d.fromDegrees(0));

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(Units.feetToMeters(feet / 2), Units.feetToMeters(0)));

    // start velocity , end velocity , reversed , constraints
    TrajectoryConfig config = new TrajectoryConfig(0.5, 0.5);
   config.setReversed(true);

    m_trajectorytest = TrajectoryGenerator.generateTrajectory(
        autoStart,
        interiorWaypoints,
        getBall,
        config

    );

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();

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
    m_driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // (Math.abs(getBall.getX() - actualPose.getX()) <= 2);
  }
}
