// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrain m_subsystem;
  private final Joystick m_leftJoystick;
  private final Joystick m_rightJoystick;

  private double leftEncoder;
  private double rightEncoder;
// it is used just ignore it
// no clue why its like that 

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDrive(DriveTrain subsystem, Joystick leftJoystick, Joystick rightJoystick) {

    m_subsystem = subsystem;
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    m_subsystem.drive(m_leftJoystick.getY()*.25, m_rightJoystick.getY()*.25);

    leftEncoder = m_subsystem.getLeftEncoder();
    rightEncoder = m_subsystem.getRightEncoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
