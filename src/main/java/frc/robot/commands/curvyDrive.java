// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class curvyDrive extends CommandBase {
  DriveTrain m_driveTrain;
  boolean m_reversed;

  RamseteController ramseteController;


  double startTime;
  double newTime;

  ArrayList<Pose2d> waypoints;
  TrajectoryConfig config;

  Pose2d startingPose;
  Pose2d midPose;
  Pose2d endPose;


  Translation2d translation1;
  Transform2d transform1;

  Translation2d translation2;
  Transform2d transform2;


  Trajectory generatedTrajectory;

  Trajectory.State targetPose;
  Pose2d currentPose;





  /** Creates a new curvyDrive. */
  public curvyDrive(DriveTrain driveTrain, boolean reversed) {
    m_driveTrain = driveTrain;
    m_reversed = reversed;

    ramseteController = new RamseteController();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.currentTimeMillis();

    startingPose = m_driveTrain.getPose();



    translation1 = new Translation2d(Units.feetToMeters(4), Units.feetToMeters(0));
    transform1 = new Transform2d(translation1, Rotation2d.fromDegrees(.01));
    midPose = startingPose.transformBy(transform1);


    translation2 = new Translation2d(Units.feetToMeters(4), Units.feetToMeters(0));
    transform2 = new Transform2d(translation2, Rotation2d.fromDegrees(.01));
    endPose = midPose.transformBy(transform2);

    waypoints = new ArrayList<Pose2d>();
    waypoints.add(startingPose);
    waypoints.add(midPose);
    waypoints.add(endPose);

    config = new TrajectoryConfig(.5, .5);
    config.setReversed(m_reversed);


    generatedTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    newTime = System.currentTimeMillis() - startTime;


    targetPose = generatedTrajectory.sample(newTime / 1000.0);
    currentPose = m_driveTrain.getPose();

    ChassisSpeeds speeds = ramseteController.calculate(currentPose, targetPose);
    m_driveTrain.kinematicsDrive(speeds.vxMetersPerSecond, 
                                 speeds.vyMetersPerSecond, 
                                 speeds.omegaRadiansPerSecond);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return (Math.abs(endPose.getX() - currentPose.getX()) <= .1);
  }
}
