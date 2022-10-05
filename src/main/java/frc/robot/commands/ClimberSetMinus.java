// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberSetMinus extends InstantCommand {
  Climber m_climber;
  double oldAngle;
  double newAngle = 15;
  public ClimberSetMinus(Climber climber) {
    m_climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oldAngle = m_climber.climberAngle();
  }


  @Override
  public void execute() {
    m_climber.climberSet(oldAngle - newAngle);

  }
}
