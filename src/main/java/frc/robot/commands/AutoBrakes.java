// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class AutoBrakes extends CommandBase {
  Elevator m_elevator;
  double posGoal;
  /** Creates a new AutoBrakes. */
  public AutoBrakes(Elevator elevator) {
    m_elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable ozram = NetworkTableInstance.getDefault().getTable("OzRam");
    posGoal = ozram.getEntry("ElevatorRotations").getDouble(0) * 2048;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  /*  if(m_elevator.trajectory() >= posGoal - 50){
      m_elevator.brakeDown();
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.brakeUp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  /*  if(m_elevator.trajectory() <= 50){
      return true;
    }
    */
    return false;
  }
}
