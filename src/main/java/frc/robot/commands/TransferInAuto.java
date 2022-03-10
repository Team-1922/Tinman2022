// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TOF;
import frc.robot.subsystems.Transfer;

public class TransferInAuto extends CommandBase {

  Transfer m_transfer;

  /** Creates a new TransferOut. */
  public TransferInAuto(Transfer transfer) {
    m_transfer = transfer;
    

    addRequirements(m_transfer);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      m_transfer.sideMotor(6000); 
      m_transfer.frontMotor(6000); 
      m_transfer.rearMotor(6000);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.sideMotor(0); // Later on adjust speed, direction if needed 
    m_transfer.frontMotor(0);
    m_transfer.rearMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
