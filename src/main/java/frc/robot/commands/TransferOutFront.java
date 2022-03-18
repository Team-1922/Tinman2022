// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TOF;
import frc.robot.subsystems.Transfer;

public class TransferOutFront extends CommandBase {

  Transfer m_transfer;
  TOF m_TOF;

  double rearOutput;
  double frontOutput;
  double sideOutput;
  /** Creates a new TransferOut. */
  public TransferOutFront(Transfer transfer, TOF TOF) {
    m_transfer = transfer;
    m_TOF = TOF;

    addRequirements(m_transfer, m_TOF);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTable ozram = NetworkTableInstance.getDefault().getTable("OzRam");

    rearOutput = ozram.getEntry("RearOutput").getDouble(500);
    frontOutput = ozram.getEntry("FrontOutput").getDouble(500);
    sideOutput = ozram.getEntry("SideOutput").getDouble(500);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_transfer.transferVelocity() < 6000){
      m_transfer.sideMotor(sideOutput);
    }
    m_transfer.sideMotor(-sideOutput); // Later on adjust speed, direction if needed 
    m_transfer.frontMotor(-frontOutput);
    m_transfer.rearMotor(rearOutput);


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
