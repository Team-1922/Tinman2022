// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.TOF;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
/*
    XboxController m_xbox = new XboxController(2);
    Joystick joystickLeft = new Joystick(1);
    Joystick joystickRight = new Joystick(0);

    NetworkTable ozram = NetworkTableInstance.getDefault().getTable("OzRam");


    WPI_TalonFX elevator1 = new WPI_TalonFX(Constants.elevator1);
    WPI_TalonFX elevator2 = new WPI_TalonFX(Constants.elevator2);
    Solenoid elevatorSolenoid = new Solenoid(0, PneumaticsModuleType.CTREPCM, 0);
*/
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
/*
    elevator2.set(ControlMode.Follower, elevator1.getDeviceID());
    elevator1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);


    elevator1.configMotionCruiseVelocity(50000);
    elevator1.configMotionAcceleration(25000);
    elevator1.configMotionSCurveStrength(4);
*/

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  //private double posGoal = 0;

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { 

    /*
    TOF TOF = new TOF();
   
    double elevatorRotations = 2048 * ozram.getEntry("ElevatorRotations").getDouble(10);

 
      if(TOF.getDistanceLeft() <= 150 || TOF.getDistanceRight() <= 150){
        
        posGoal = elevatorRotations;
        elevator1.set(ControlMode.MotionMagic, posGoal);

      }
  
      if(TOF.getDistanceLeft() > 150){
        
        posGoal = 0;
        elevator1.set(ControlMode.MotionMagic, posGoal);

    }

    if(elevator1.getActiveTrajectoryPosition() != posGoal && elevator1.getActiveTrajectoryPosition() > -1000){
      elevatorSolenoid.set(true);
    } else {
      elevatorSolenoid.set(false);
    }
  
    if(elevator1.getActiveTrajectoryPosition() == posGoal){
      elevator1.set(ControlMode.PercentOutput, 0);
  
    }
*/

  }
  
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    
      //elevatorSolenoid.set(false);
    }

   // elevator1.set(ControlMode.MotionMagic, 0);

  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
/*
    double leftTrigger = m_xbox.getLeftTriggerAxis();
    double rightTrigger = m_xbox.getRightTriggerAxis();

    // boolean left5 = joystickLeft.getRawButton(5);
    // boolean left6 = joystickLeft.getRawButton(6);

    boolean xbox9 = m_xbox.getRawButton(9);
    boolean xbox10 = m_xbox.getRawButton(10);


    double elevatorRotations = 2048 * ozram.getEntry("ElevatorRotations").getDouble(10);

    // SmartDashboard.putNumber("TrajectoryPosition", elevator1.getActiveTrajectoryPosition());

    if (xbox9 == true && xbox10 == true){
      posGoal = 0;
      elevator1.set(ControlMode.MotionMagic, posGoal);
    } else {

    if(xbox9 == true){
      posGoal = elevatorRotations;
      elevator1.set(ControlMode.MotionMagic, posGoal);
    }

    if(xbox10 == true){
      posGoal = 0;
      elevator1.set(ControlMode.MotionMagic, posGoal);
    }

    if(elevator1.getActiveTrajectoryPosition() != posGoal){
      elevatorSolenoid.set(true);
    } else {
      elevatorSolenoid.set(false);
    }
  }

  if(elevator1.getActiveTrajectoryPosition() == posGoal){
    elevator1.set(ControlMode.PercentOutput, 0);

  }


    SmartDashboard.putNumber("TriggerLeft", leftTrigger);
    SmartDashboard.putNumber("TriggerRight", rightTrigger);

*/


  }


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
