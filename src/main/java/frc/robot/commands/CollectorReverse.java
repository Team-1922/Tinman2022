// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Collector;

public class CollectorReverse extends CommandBase {
Collector m_collector;
  /** Creates a new Collector_out. */
  public CollectorReverse(Collector collector) {
    m_collector = collector;
    addRequirements(m_collector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled hehe i change it to execute
  @Override
  public void execute() {
    m_collector.reverse();
  }

  @Override

  public void end(boolean interrupted) {
    m_collector.collect();
  }
 
}