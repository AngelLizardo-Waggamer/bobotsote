// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;

import com.revrobotics.RelativeEncoder;

import javax.lang.model.util.ElementScanner6;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Motores del chasis (spark maxs)
  private CANSparkMax chasis_motorL1;
  private CANSparkMax chasis_motorL2;
  private CANSparkMax chasis_motorR1;
  private CANSparkMax chasis_motorR2;

  // Otros motores (spark normal) 
  private Spark MotorPI; //Metedor Interior num: 0
  private Spark MotorPE; //Metedor Exterior intake num: 1
  private Spark putito ; // num: 2
  private Spark shooter2; // num: 3
  private Spark shooter1; // num: 4
  private Spark elevador1;  //Elevadores  num: 5
  private Spark elevador2;  //Elevadores  num: 6

 
  // poder de los motores
  double leftPower; // lado izquierdo del chasis
  double rightPower; // lado derecho del chasis
  boolean intekOn = false;
  boolean shooterOn = false;


  //Camara config
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  
  //Camara
  //read values periodically
  double xll = tx.getDouble(0.0);
  double yll = ty.getDouble(0.0);
  double all = ta.getDouble(0.0);



// --- ni idea de que es lo que hay en las siguientes lineas
  /*private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
  private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);*/
  private final Joystick drive_Joy = new Joystick(0);
  private final Joystick Shoot_Joy = new Joystick(1);
  //private final Joystick m2s = new Joystick(1);
  private final Timer m_timer = new Timer();
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private RelativeEncoder m_encoder_TEST;
  private double encoder_init;
  private double encoder;
  //private final I2C.Port i2cPort = I2C.Port.kOnboard;
  //private final ColorSensorV3 sensorColor = new ColorSensorV3(i2cPort);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    try{
      chasis_motorR1 = new CANSparkMax(3, MotorType.kBrushless);
      chasis_motorL1 = new CANSparkMax(5, MotorType.kBrushless); //Chasis
      chasis_motorL2 = new CANSparkMax(2, MotorType.kBrushless);
      chasis_motorR2 = new CANSparkMax(4, MotorType.kBrushless);
    } catch (Exception e ){

    }
    

    chasis_motorL1.restoreFactoryDefaults();
    chasis_motorL2.restoreFactoryDefaults();
    chasis_motorR1.restoreFactoryDefaults();
    chasis_motorR2.restoreFactoryDefaults();

    m_encoder_TEST = chasis_motorL1.getEncoder();

    chasis_motorL2.follow(chasis_motorL1);
    chasis_motorR2.follow(chasis_motorR1);

    
    try{
      // demas motores
      MotorPI = new Spark(0);               //Metedor Interior num: 0
      MotorPE = new Spark(1);               //Metedor Exterior intake num: 1
      putito  = new Spark(2);  // el redline del elevador num: 2
      shooter2 = new Spark(3);              // shooter num: 3
      shooter1 = new Spark(4);              // shooter num: 4
      elevador1 = new Spark(5);             //Elevadores  num: 5
      elevador2 = new Spark(6);             //Elevadores  num: 6
    } catch (Exception e ){

    }



    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
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
    //double proximidad = sensorColor.getProximity();
    //SmartDashboard.putNumber("Proximidad", proximidad);

    
    //Camara
    //read values periodically
    xll = tx.getDouble(0.0);
    yll = ty.getDouble(0.0);
    all = ta.getDouble(0.0);
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
    encoder_init = m_encoder_TEST.getPosition();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    encoder = m_encoder_TEST.getPosition();
    if (encoder_init - 60 < encoder){
      RobotMoveAut(-0.5, 0);
    } else {
      RobotMove(0, 0);  
      shooter1.set(-1);
      Timer.delay(1);
      shooter1.set(0);
    }

    /*if (m_timer.get() < 2.0) {
      m_robotDrive.arcadeDrive(0.5, 0.0); // drive forwards half speed
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }*/
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    // motores de el chasis
   
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_robotDrive.arcadeDrive(drive_Joy.getY(), drive_Joy.getX());

    //motorL1.set(drive_Joy.getY());
    RobotMove(drive_Joy.getY()*0.8, drive_Joy.getX()*0.4);
    SmartDashboard.putNumber("Encoder Position", m_encoder_TEST.getPosition());

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", xll);
    SmartDashboard.putNumber("LimelightY", yll);
    SmartDashboard.putNumber("LimelightArea", all);
  

    /*if(Shoot_Joy.getRawButton(1)){
      shooterOn = !shooterOn;
    }*/

    if (Shoot_Joy.getRawButton(4)) {
      // Mover las pelotas para atrás
      MotorPI.set(1);
      MotorPE.set(-0.6);
      shooter2.set(0.4);
      Timer.delay(0.15);

      // Parar el motor de abajo y encender el shooter
      MotorPI.set(0);
      MotorPE.set(0);
      shooter1.set(-0.8);
      shooter2.set(-0.8);

      Timer.delay(1);
      // Mover las pelotas para arriba
      MotorPI.set(-1);

      // Delay más largo por si son 2 pelotas
      Timer.delay(1.5);

      // Parar los 3 motores
      MotorPI.set(0);
      shooter1.set(0);
      shooter2.set(0);
    } 
    if (Shoot_Joy.getRawButton(1)) {
      // Mover las pelotas para atrás
      MotorPI.set(1);
      MotorPE.set(-0.5);
      shooter2.set(0.4);
      Timer.delay(0.15);

      // Parar el motor de abajo y encender el shooter
      MotorPI.set(0);
      MotorPE.set(0);
      shooter1.set(-1);
      shooter2.set(-1);

      Timer.delay(1);
      // Mover las pelotas para arriba
      MotorPI.set(-1);

      // Delay más largo por si son 2 pelotas
      Timer.delay(1.5);

      // Parar los 3 motores
      MotorPI.set(0);
      shooter1.set(0);
      shooter2.set(0);
    }

    if (Shoot_Joy.getRawButton(3)){
      //if (sensorColor.getProximity() < 5.0){
        //MotorPI.set(0);
        
        //MotorPE.set(0.7);
      //}else {
        MotorPE.set(0.5);
        MotorPI.set(-0.7);
      //}
      

    } else {
      MotorPE.set(0);
      MotorPI.set(0);
    }

    /**/

    if (Shoot_Joy.getPOV() == 0){
      elevador1.set(1);
      elevador2.set(1);
    } else if (Shoot_Joy.getPOV() == 180){
      elevador1.set(-1);
      elevador2.set(-1);
    } else {
      elevador1.set(0);
      elevador2.set(0);
    }

    if (Shoot_Joy.getPOV() == 90) {
      putito.set(-0.5);
    }else if (Shoot_Joy.getPOV() == 270){
      putito.set(0.5);
    } else {
      putito.set(0);
    }
    
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }
  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
  
  
  public void RobotMove(double forward, double rotation){
    
    double leftPower = RangoT(-forward + (rotation * 0.5),-1,1);
    double rightPower = -RangoT(-forward - (rotation * 0.5),-1,1);
    
    chasis_motorL1.set(leftPower);
    chasis_motorR1.set(rightPower);
    
  }
  
  public void RobotMoveAut(double forward, double rotation){
    
    double leftPower = RangoT(forward + (rotation * 0.4),-1,1);
    double rightPower = -RangoT(forward - (rotation * 0.4),-1,1);
    
    chasis_motorL1.set(leftPower);
    chasis_motorR1.set(rightPower);
    
  }

  public static double RangoT(double number, double min, double max){
    if (number < min) return min;
    if (number > max) return max;
    return number;
  }
}
