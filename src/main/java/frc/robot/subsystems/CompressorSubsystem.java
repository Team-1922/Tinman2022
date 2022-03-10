// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CompressorSubsystem extends SubsystemBase {
  /** Creates a new air. */
  public CompressorSubsystem() {
  }

  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public void compressortoggle() {
    boolean compressorValue = m_compressor.enabled();
    if (compressorValue) {
      m_compressor.disable();

    } else {
      m_compressor.enableDigital();

    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    boolean compressorValue = m_compressor.enabled();
    SmartDashboard.putBoolean("compressorEnable", compressorValue);
  }
}
