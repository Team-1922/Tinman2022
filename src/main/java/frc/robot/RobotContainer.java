// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//the code will have too many comments at some point
// sounds fun right?
// 1 pasta dish + 1 pasta dish = 1 pasta dish

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoBrakes;
import frc.robot.commands.Auto_forward;
import frc.robot.commands.BallAim;
import frc.robot.commands.BallGet;
import frc.robot.commands.BrakeActivate;
import frc.robot.commands.BrakeDeactivate;
import frc.robot.commands.BrakeDown;
import frc.robot.commands.BrakeUp;
import frc.robot.commands.ClimberDown;
import frc.robot.commands.ClimberUp;
import frc.robot.commands.CollectorOut;
import frc.robot.commands.CollectorReverse;
import frc.robot.commands.DistanceDrive;
import frc.robot.commands.DriveKinematics;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ElevateDown;
import frc.robot.commands.ElevateDownClimb;
import frc.robot.commands.ElevateUp;
import frc.robot.commands.ElevateUpClimb;
import frc.robot.commands.JoystickTankDrive;
import frc.robot.commands.NearLight;
import frc.robot.commands.PlainJoystickTankDrive;
import frc.robot.commands.TOFDistance;
import frc.robot.commands.TOFDrive;
import frc.robot.commands.TOFSeeBall;
import frc.robot.commands.TankDrive;
import frc.robot.commands.TargetGet;
import frc.robot.commands.ToggleFlip;
import frc.robot.commands.TransferIn;
import frc.robot.commands.TransferInAuto;
import frc.robot.commands.TransferOutFront;
import frc.robot.commands.TransferOutRear;
import frc.robot.commands.Turn;
import frc.robot.commands.WeirdTankDrive;
import frc.robot.commands.XboxTankDrive;
import frc.robot.commands.backToHub;
import frc.robot.commands.curvyDrive;
import frc.robot.commands.goToBall;
import frc.robot.commands.gofromBall;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Collector;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.TOF;
import frc.robot.subsystems.Transfer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.SPI;
/**
 * This class is where the hulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
private final XboxController m_XBoxController = new XboxController(2);
private final TOF m_TOF = new TOF();
private final LED m_LED = new LED();
private final Collector m_collector = new Collector();
private final Transfer m_transfer = new Transfer();
private final Elevator m_elevator = new Elevator();
private final Climber m_climber = new Climber();
private final CompressorSubsystem m_compressorSubsystem = new CompressorSubsystem();

private final AHRS m_ahrs = new AHRS(SPI.Port.kMXP);
private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(-m_ahrs.getAngle()));

private final SendableChooser<CommandBase> m_tankChooser = new SendableChooser<CommandBase>();


  private final TOFDistance m_TOFDistance = new TOFDistance(m_TOF);
  private final DriveTrain m_driveTrain = new DriveTrain(m_ahrs, m_odometry);
  private final Joystick m_joystickLeft = new Joystick(1);
  private final Joystick m_joystickRight = new Joystick(0);
  private final TankDrive m_tankDrive = new TankDrive(m_driveTrain, m_joystickLeft, m_joystickRight);
  private final WeirdTankDrive m_weirdTankDrive = new WeirdTankDrive(m_driveTrain, m_joystickLeft, m_joystickRight);
  private final XboxTankDrive m_XboxTankDrive = new XboxTankDrive(m_driveTrain, m_XBoxController);
  private final JoystickTankDrive m_joystickTankDrive = new JoystickTankDrive(m_driveTrain, m_joystickLeft, m_joystickRight);
  private final PlainJoystickTankDrive m_plainJoystickTankDrive = new PlainJoystickTankDrive(m_driveTrain, m_joystickLeft, m_joystickRight);
  private final ToggleFlip m_toggleFlip = new ToggleFlip(m_driveTrain);
  private final DriveStraight m_driveStraight = new DriveStraight(m_driveTrain, m_joystickLeft);
  private final NearLight m_nearLight = new NearLight(m_TOF, m_LED);

  private final CollectorOut m_collectorOut = new CollectorOut(m_collector);
  private final CollectorReverse m_collectorReverse = new CollectorReverse(m_collector);

  private final BrakeActivate m_brakeAct = new BrakeActivate(m_elevator);
  private final BrakeDeactivate m_brakeDeact = new BrakeDeactivate(m_elevator);

  private final ClimberUp m_climberUp = new ClimberUp(m_climber);
  private final ClimberDown m_climberDown = new ClimberDown(m_climber);

  private final DriveKinematics m_driveKinematics = new DriveKinematics(m_driveTrain);

  // Transfer commands
  private final TransferOutFront m_transferOutFront = new TransferOutFront(m_transfer, m_TOF);
  private final TransferOutRear m_transferOutRear = new TransferOutRear(m_transfer, m_TOF);
  private final TransferIn m_transferIn = new TransferIn(m_transfer, m_TOF);
 
  // elevooter
  private final ElevateDown m_elevateDown = new ElevateDown(m_elevator);
  private final ElevateUp m_elevateUp = new ElevateUp(m_elevator);

  private final BrakeUp m_brakeUp = new BrakeUp(m_elevator);
  private final BrakeDown m_brakeDown = new BrakeDown(m_elevator);

  // Auto Commands
  private final BallAim m_ballAim = new BallAim(m_driveTrain);
  private final BallGet m_ballGet = new BallGet(m_driveTrain, m_TOF);
  private final AutoBrakes m_autoBrakes = new AutoBrakes(m_elevator);
  private final Turn m_turn = new Turn(m_driveTrain);
  private final TargetGet m_targetGet = new TargetGet(m_driveTrain);
  private final Auto_forward m_autoForward = new Auto_forward(m_driveTrain);

  private final goToBall m_goToBall = new goToBall(m_driveTrain, 15, false);
  private final curvyDrive m_curvyDrive = new curvyDrive(m_driveTrain, false);




  private final SequentialCommandGroup ElevatorUp(){

    BrakeUp brakeUp = new BrakeUp(m_elevator);
    BrakeDown brakeDown = new BrakeDown(m_elevator);
    ElevateUp elevateUp = new ElevateUp(m_elevator);

    SequentialCommandGroup elevatorUp = new SequentialCommandGroup(brakeDown, elevateUp, brakeUp);

    return elevatorUp;
  }
  
  private final SequentialCommandGroup ElevatorDown(){

    BrakeUp brakeUp = new BrakeUp(m_elevator);
    BrakeDown brakeDown = new BrakeDown(m_elevator);
    ElevateDown elevateDown = new ElevateDown(m_elevator);

    SequentialCommandGroup elevatorDown = new SequentialCommandGroup(brakeDown, elevateDown, brakeUp);

    return elevatorDown;
  }



  private final SequentialCommandGroup ElevatorUpClimb(){

    BrakeUp brakeUp = new BrakeUp(m_elevator);
    BrakeDown brakeDown = new BrakeDown(m_elevator);
    ElevateUpClimb elevateUp = new ElevateUpClimb(m_elevator);
    ClimberUp climberUp = new ClimberUp(m_climber);

    SequentialCommandGroup elevatorUp = new SequentialCommandGroup(brakeDown, elevateUp, brakeUp, climberUp);

    return elevatorUp;
  }

  private final SequentialCommandGroup ElevatorDownClimb(){

    BrakeUp brakeUp = new BrakeUp(m_elevator);
    BrakeDown brakeDown = new BrakeDown(m_elevator);
    ElevateDownClimb elevateDown = new ElevateDownClimb(m_elevator);

    SequentialCommandGroup elevatorDown = new SequentialCommandGroup(brakeDown, elevateDown, brakeUp);

    return elevatorDown;
  }



  private ParallelDeadlineGroup AutoUnload(){
    
    TransferOutFront m_transferOutFront = new TransferOutFront(m_transfer, m_TOF);
    TransferOutRear m_transferOutRear = new TransferOutRear(m_transfer, m_TOF);

    ParallelDeadlineGroup transferTime = new ParallelDeadlineGroup(new WaitCommand(5), m_transferOutFront);

    return transferTime;
  }

  private SequentialCommandGroup Pickup(){

    CollectorOut m_collectorOut = new CollectorOut(m_collector);
    TransferIn m_transferIn = new TransferIn(m_transfer, m_TOF);


    BallAim m_ballAim = new BallAim(m_driveTrain);
    BallGet m_ballGet = new BallGet(m_driveTrain, m_TOF);

    ParallelCommandGroup m_intake = new ParallelCommandGroup(m_collectorOut, m_transferIn);
    ParallelCommandGroup m_goIntake = new ParallelCommandGroup(m_intake, m_ballGet);
    SequentialCommandGroup m_pickup = new SequentialCommandGroup(m_ballAim, m_goIntake);

    return m_pickup;
  }

  private SequentialCommandGroup GoTarget(){

    Turn m_turn = new Turn(m_driveTrain);
    TargetGet m_targetGet = new TargetGet(m_driveTrain);

    SequentialCommandGroup goTarget = new SequentialCommandGroup(m_targetGet, m_turn);

    return goTarget;

  }



public Elevator getElevator() {
  return m_elevator;
}

public DriveTrain getDriveTrain(){
  return m_driveTrain;
}


   Command trajectoryautopartone(){
  //  gofromBall m_goTfromBall = new gofromBall(m_driveTrain, 15);
    goToBall m_goToBall = new goToBall(m_driveTrain, 15, false);
//return m_goTfromBall, 

return m_goToBall;

   }


Command trajectoryautoparttwo(){
   gofromBall m_goTfromBall = new gofromBall(m_driveTrain, 15);
    
return m_goTfromBall; 



  }


 //private SequentialCommandGroup fourthAuto(){
  //CollectorOut m_collectorOut = new CollectorOut(m_collector);
   

 //automover m_driveTrain = new TrajectoryGenerator.generateTrajectory(m_driveTrain);
 // SequentialCommandGroup fourthAuto = new SequentialCommandGroup(trajectoryautopartone() , new WaitCommand(2),trajectoryautoparttwo() 
  //);
  //return fourthAuto; 
//} 

private Command FourthAuto(){

  goToBall moveForward = new goToBall(m_driveTrain, 10, false);
  goToBall moveBack = new goToBall(m_driveTrain, -10, true);

  TransferInAuto transfer = new TransferInAuto(m_transfer);
  TransferOutRear transferOut = new TransferOutRear(m_transfer, m_TOF);
  CollectorOut collector = new CollectorOut(m_collector);
  TOFSeeBall TOF = new TOFSeeBall(m_TOF);
  ElevateUp elevatorUp = new ElevateUp(m_elevator);
  ElevateDown elevatorDown = new ElevateDown(m_elevator);


  ParallelDeadlineGroup getBall = new ParallelDeadlineGroup(TOF, transfer, collector);
  ParallelCommandGroup forward = new ParallelCommandGroup(getBall, moveForward);
  ParallelDeadlineGroup transferDeploy = new ParallelDeadlineGroup(new WaitCommand(2), transferOut);
  SequentialCommandGroup deploy = new SequentialCommandGroup(ElevatorUp(), transferDeploy, ElevatorDown());

  SequentialCommandGroup move = new SequentialCommandGroup(forward, new WaitCommand(1), moveBack, deploy);

 // SequentialCommandGroup move = new SequentialCommandGroup(moveForward, new WaitCommand(1), moveBack);
  return move;
}



  private SequentialCommandGroup Auto(){

    SequentialCommandGroup auto = new SequentialCommandGroup(Pickup(), GoTarget(), AutoUnload());

    return auto;
  }

  private SequentialCommandGroup SecondAuto(){

    TransferOutRear m_transferOut = new TransferOutRear(m_transfer, m_TOF);
    Auto_forward m_autoForward = new Auto_forward(m_driveTrain);
    BallGet m_ballGet = new BallGet(m_driveTrain, m_TOF);
    CollectorOut m_collectorOut = new CollectorOut(m_collector);
    TransferInAuto m_transferIn = new TransferInAuto(m_transfer);

    ParallelDeadlineGroup transferOutput = new ParallelDeadlineGroup(new WaitCommand(2), m_transferOut);
    ParallelDeadlineGroup ballGrab = new ParallelDeadlineGroup(m_ballGet, m_collectorOut, m_transferIn);
    SequentialCommandGroup elevator = new SequentialCommandGroup(ElevatorUp(), transferOutput, ElevatorDown());

    SequentialCommandGroup auto2 = new SequentialCommandGroup(elevator, m_autoForward);
    

    

    return auto2;

  }

private Command ThirdAuto(){
  TransferOutRear m_transferOut = new TransferOutRear(m_transfer, m_TOF);
  ParallelDeadlineGroup transferOutput = new ParallelDeadlineGroup(new WaitCommand(2), m_transferOut);
  SequentialCommandGroup elevator = new SequentialCommandGroup(ElevatorUp(), transferOutput, ElevatorDown());
  CollectorOut m_collectorOut = new CollectorOut(m_collector);
  TransferInAuto m_transferIn = new TransferInAuto(m_transfer);
  DistanceDrive m_distanceDriveBackward = new DistanceDrive(m_driveTrain, -150);

  BallGet m_ballGet = new BallGet(m_driveTrain, m_TOF);
  TOFDrive m_tofDrive = new TOFDrive(m_driveTrain, m_TOF);

  SequentialCommandGroup ballGetDrive = new SequentialCommandGroup(m_ballGet, m_tofDrive);

  ParallelDeadlineGroup ballGrab = new ParallelDeadlineGroup(ballGetDrive, m_collectorOut, m_transferIn);

  //SequentialCommandGroup AutoThree = new SequentialCommandGroup(ballGrab, m_distanceDriveBackward, elevator);


  return ballGrab;
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
   // m_driveTrain.setDefaultCommand(m_XboxTankDrive);
    m_driveTrain.setDefaultCommand(m_plainJoystickTankDrive);
    
    m_TOF.setDefaultCommand(m_TOFDistance);
    
   

    // Configure the button bindings
    configureButtonBindings();
    initNetworkTable();
  }



  private void initNetworkTable(){
    NetworkTable table = NetworkTableInstance.getDefault().getTable("OzRam");

    NetworkTableEntry ballPGain = table.getEntry("BallPGain");
    ballPGain.setNumber(.005);

    NetworkTableEntry ballDGain = table.getEntry("BallDGain");
    ballDGain.setNumber(.0075);

    NetworkTableEntry ballSpeed = table.getEntry("BallSpeed");
    ballSpeed.setNumber(.15);

    NetworkTableEntry ballTimerGoal = table.getEntry("BallTimeGoal");
    ballTimerGoal.setNumber(5);

    NetworkTableEntry ballMinSpeed = table.getEntry("BallMinSpeed");
    ballMinSpeed.setNumber(.175);

    NetworkTableEntry ballAngle = table.getEntry("BallAngle");
    ballAngle.setNumber(1.5);

    NetworkTableEntry acceptWidth = table.getEntry("AcceptableWidth");
    acceptWidth.setNumber(140);

    NetworkTableEntry pipeline = table.getEntry("Pipeline");
    pipeline.setNumber(0);

    NetworkTableEntry elevatorRotations = table.getEntry("ElevatorRotations");
    elevatorRotations.setNumber(85);


    
    NetworkTableEntry tankSpeed = table.getEntry("TankSpeed");
    tankSpeed.setNumber(.15);
    // set to .6



    NetworkTableEntry sideOutput = table.getEntry("SideOutput");
    sideOutput.setNumber(18000);

    NetworkTableEntry rearOutput = table.getEntry("RearOutput");
    rearOutput.setNumber(20000);

    NetworkTableEntry frontOutput = table.getEntry("FrontOutput");
    frontOutput.setNumber(20000);
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   new JoystickButton(m_XBoxController, 1) //A
    .whenPressed(ElevatorUpClimb()); 

    new JoystickButton(m_XBoxController, 2) //B
    .whenPressed(m_climberUp);

    new JoystickButton(m_XBoxController, 3) //X
    .toggleWhenPressed(m_collectorReverse);

    new JoystickButton(m_XBoxController, 4) //Y
    .whenPressed(m_climberDown);


    new JoystickButton(m_XBoxController, 5) //Left bumper 
    .whileHeld(m_transferOutFront);

    new JoystickButton(m_XBoxController, 6) // Right bumper
    .whileHeld(m_transferOutRear);


    new JoystickButton(m_XBoxController, 7) // Left Menu
    .whileHeld(m_transferIn);

    new JoystickButton(m_XBoxController, 8) // Right Menu
      .toggleWhenPressed(m_collectorOut);

    new JoystickButton(m_XBoxController, 9)
      .whenPressed(ElevatorUp());

    new JoystickButton(m_XBoxController, 10)
      .whenPressed(ElevatorDown());



    new JoystickButton(m_joystickLeft, 1)
        .whileHeld(m_driveStraight);

    new JoystickButton(m_joystickLeft, 10) //A
        .toggleWhenPressed(m_driveKinematics);

   // new JoystickButton(m_joystickLeft, 12)
       // .whenPressed(m_curvyDrive);

    

    new JoystickButton(m_joystickLeft, 3)
        .whenPressed(m_toggleFlip);


    new JoystickButton(m_joystickLeft, 4)
        .whenPressed(ElevatorDownClimb());

        new JoystickButton(m_joystickLeft, 5)
        .whenPressed(new InstantCommand
        (m_compressorSubsystem::compressortoggle,m_compressorSubsystem ));

/*
    new JoystickButton(m_joystickLeft, 5)
        .whenPressed(m_climberUp);

    new JoystickButton(m_joystickLeft, 6)
        .whenPressed(m_climberDown);
*/


// testing testing
    new JoystickButton(m_joystickRight, 11)
        .whenPressed(m_ballAim);



  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return FourthAuto();

  }

  public Command getDisabledCommand(){
    return m_brakeAct;
  }
}

