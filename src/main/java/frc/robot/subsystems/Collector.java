// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  private WPI_TalonSRX collectorMotor = new WPI_TalonSRX(Constants.collectorMotor);
  private Solenoid collectorSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, 1);
  /** Creates a new Collector. */
  public Collector() {}
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

 public void collect() {
collectorMotor.set(1);
collectorSolenoid.set(true);
 }

 public void reverse(){
   collectorMotor.set(-1);
   collectorSolenoid.set(true);
 }

  public void stopCollect() {
collectorMotor.set(0);
collectorSolenoid.set(false);
  } 

  public boolean collectorState(){
    return collectorSolenoid.get();
  }
}
