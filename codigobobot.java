// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.ColorSensorV3;

import com.revrobotics.RelativeEncoder;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
  private Spark elevadorSec; // Elevador contrario num: 7

 
  // poder de los motores
  double leftPower; // lado izquierdo del chasis
  double rightPower; // lado derecho del chasis
  boolean intekOn = false;
  boolean shooterOn = false;

  //Autónomo
  int hw;
  double VueltasPorMetro = 10.75/((15.25*Math.PI)/100);
  
  double DyMenos = -1.2;
  double DyMas = 1.2;
  double DxMenos = -1.2;
  double DxMas =   1.2;
  boolean isAligned = false;

  double pyMenos = 1 - 2.5;
  double pyMas = 1 + 2.5;
  double pxMenos = 28 - 2.5;
  double pxMas =   28 + 2.5;
  

  //Potencia
  double PotA = 0.85;
  double PotG = 0.4;

  static final int CAM_W = 320;
  static final int CAM_H = 240;



  //Camara config
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  
  //Camara
  //read values periodically
  double xll;
  double yll;
  double all;
   //Servo Camara
  int alianza = 0;


  private final XboxController drive_Joy = new XboxController(0);
  private final Joystick Shoot_Joy = new Joystick(1);
  
  String direccion = "uy kieto";

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private RelativeEncoder m_encoder_TEST;
  private double encoder_init;
  private double encoder;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 sensorColor = new ColorSensorV3(i2cPort);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    isAligned = false;
    try{
      chasis_motorR1 = new CANSparkMax(3, MotorType.kBrushless);
      chasis_motorL1 = new CANSparkMax(5, MotorType.kBrushless); //Chasis
      chasis_motorL2 = new CANSparkMax(2, MotorType.kBrushless);
      chasis_motorR2 = new CANSparkMax(4, MotorType.kBrushless);
      // demas motores
      MotorPI = new Spark(0);               //Metedor Interior num: 0
      MotorPE = new Spark(1);               //Metedor Exterior intake num: 1
      putito  = new Spark(2);               // el redline del elevador num: 2
      shooter2 = new Spark(3);              // shooter num: 3
      shooter1 = new Spark(4);              // shooter num: 4
      elevador1 = new Spark(5);             //Elevadores  num: 5
      elevador2 = new Spark(6);             //Elevadores  num: 6
      elevadorSec = new Spark(7);           // Elevador motor secundario num: 7

    } catch (Exception e ){

    }
    

    chasis_motorL1.restoreFactoryDefaults();
    chasis_motorL2.restoreFactoryDefaults();
    chasis_motorR1.restoreFactoryDefaults();
    chasis_motorR2.restoreFactoryDefaults();

    m_encoder_TEST = chasis_motorL1.getEncoder();

    chasis_motorL2.follow(chasis_motorL1);
    chasis_motorR2.follow(chasis_motorR1);

    
   



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
   

    /*SmartDashboard.putNumber("distancia", sensorColor.getProximity());
    SmartDashboard.putNumber("Red", sensorColor.getRed());
    SmartDashboard.putNumber("Green", sensorColor.getGreen());
    SmartDashboard.putNumber("Blue", sensorColor.getBlue());*/
    SmartDashboard.putNumber("LimelightX", xll);
    SmartDashboard.putNumber("LimelightY", yll);
    SmartDashboard.putNumber("LimelightArea", all);
    SmartDashboard.putString("direccion", direccion);
    SmartDashboard.putBoolean("al menos si se esta subiendo esto", isAligned);
    switch (DriverStation.getAlliance()){
      case Red:
        alianza = 1;
        break;
      case Blue:
        alianza = 2;
        break;
      default:
        alianza = 0;
    }
    SmartDashboard.putNumber("Alianza", alianza);
    // http://10.56.96.11:5801/
    // Y = -1.8 | -2.3
    if (xll > DxMenos && xll < DxMas && yll > DyMenos && yll < DyMas){
      isAligned = true;
    }else{
      isAligned = false;
    }
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
    hw = 1;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {



    encoder = m_encoder_TEST.getPosition();

    switch (hw){
      case 1:
        if (encoder_init - (1.9*VueltasPorMetro) < encoder){
          RobotMoveAut(-0.3, 0);
        } else if (encoder_init - (2.5*VueltasPorMetro) < encoder){
          RobotMoveAut(-0.1, 0);
        }else{
          RobotMoveAut(0, 0);
          
          //x=0.7, y = 1.7
          table.getEntry("pipeline").setValue(0);
          hw++;
          //12345678
        }
        break;

      case 2:
        
        // Apuntar 
        if(!isAligned && xll != 0 && yll != 0){
          if (xll < DxMenos) {
              direccion = "adelante";
            //while (xll < DxMenos) {
              // ADELANTE 
              RobotMoveAut(-0.1, 0);
            //} 
            
          }

          if (xll > DxMas) {
            direccion = "atras";
            //while(xll > DxMas) {
              // ATRAS
              RobotMoveAut(0.1, 0);
            //}
            
          }
         if(yll < DyMenos){
          direccion = "izquierda";
          //while (yll < DyMas) {
            // IZQUIERDA
            RobotMoveAut(0,-0.1);
          //}
        }
        if (yll > DyMas) {
          direccion = "derecha";
          //while(yll > DyMenos) {
            //DERECHA
            RobotMoveAut(0,0.1);
          //}
          }
            
        }else if (xll == 0 && yll == 0){
          RobotMoveAut(0, -0.1);
        }else{
          RobotMove(0,0);
          autShoot();
          if (alianza == 1){
            table.getEntry("pipeline").setValue(1);
          }else{
            table.getEntry("pipeline").setValue(2);
          }
          hw++;
        }
        // Cuando se tengan dos pelotas, pasar a tarea 3
        break;
      case 3:
      
      // Reutilizar el codigo para seguir la pelota
      if(xll == 0 && yll == 0){
        RobotMoveAut(0, 0.2);
      }else{
          
          if(yll < pyMenos){
            direccion = "izquierda";
            RobotMoveAut(0,-0.15);
          }
          if (yll > pyMas) {
            direccion = "derecha";
            RobotMoveAut(0,0.15);
          }

          if (yll > DyMenos && yll < DyMas && all > 0.1) {
            // soltar intake
            putito.set(0.5);
            Timer.delay(3);
            MotorPE.set(0.5);
            MotorPI.set(-1);
            RobotMoveAut(0.2, 0);
            Timer.delay(3);
            RobotMoveAut(0, 0);
            MotorPE.set(0);
            MotorPI.set(0);
            putito.set(-0.5);
            Timer.delay(2);
            putito.set(0);
            table.getEntry("pipeline").setValue(0);
            hw++;
          }
        }
        
        break;

      case 4:
        // apuntar a la "canasta"
        isAligned = false;
        if(!isAligned && xll != 0 && yll != 0){
          if (xll < DxMenos) {
              direccion = "adelante";
            //while (xll < DxMenos) {
              // ADELANTE 
              RobotMoveAut(-0.1, 0);
            //} 
            
          }

          if (xll > DxMas) {
            direccion = "atras";
            //while(xll > DxMas) {
              // ATRAS
              RobotMoveAut(0.1, 0);
            //}
            
          }
         if(yll < DyMenos){
          direccion = "izquierda";
          //while (yll < DyMas) {
            // IZQUIERDA
            RobotMoveAut(0,-0.1);
          //}
        }
        if (yll > DyMas) {
          direccion = "derecha";
          //while(yll > DyMenos) {
            //DERECHA
            RobotMoveAut(0,0.1);
          //}
          }
            
        }else if (xll == 0 && yll == 0){
          RobotMoveAut(0, -0.1);
        }else{
          RobotMove(0,0);
          autShoot();
        }
        break;
      default: 

        break;
    }
    
  }

  @Override
  public void teleopInit() {
   
    table.getEntry("pipeline").setValue(3);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_robotDrive.arcadeDrive(drive_Joy.getY(), drive_Joy.getX());
    
   
    if (drive_Joy.getLeftTriggerAxis() > 0){
      PotA = 0.2;
    }else{
      PotA = 0.85;
    }
    //motorL1.set(drive_Joy.getY());
    if (drive_Joy.getRightTriggerAxis() > 0){
      PotG = 0.1;
    }else {
      PotG = 0.4;
    }

    RobotMove(drive_Joy.getLeftY()*PotA, drive_Joy.getRightX()*PotG);
    

    
    if (Shoot_Joy.getRawButton(4)) {
      lowShoot();
    } 
    if (Shoot_Joy.getRawButton(1)) {
      strongshoot();
    }

    if (Shoot_Joy.getRawButton(3)){
      if (sensorColor.getProximity() > 400){
        MotorPI.set(0);
        MotorPE.set(0.7);
      }else {
        MotorPE.set(0.5);
        MotorPI.set(-1);
      }
      

    } else {
      MotorPE.set(0);
      MotorPI.set(0);
    }

    /**/

    if (Shoot_Joy.getPOV() == 180){
      elevador1.set(1);
      elevador2.set(1);
      elevadorSec.set(1);
    } else if (Shoot_Joy.getPOV() == 0){
      elevador1.set(-0.3);
      elevador2.set(-0.3);
      elevadorSec.set(-1);
    } else {
      elevador1.set(0);
      elevador2.set(0);
      elevadorSec.set(0);
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
  public void testPeriodic() {

  }
  
  
  
  public void RobotMove(double forward, double rotation){
    
    double leftPower = RangoT(-forward + (rotation * 0.35),-1,1);
    double rightPower = -RangoT(-forward - (rotation * 0.35),-1,1);
    
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

  public void strongshoot(){
    // Mover las pelotas para atrás
    RobotMove(0, 0);
    MotorPI.set(1);
    MotorPE.set(-0.5);
    shooter2.set(0.4);
    Timer.delay(0.16);

    // Parar el motor de abajo y encender el shooter
    MotorPI.set(0);
    MotorPE.set(0);
    shooter1.set(-1);
    shooter2.set(-1);

    Timer.delay(1);
    // Mover las pelotas para arriba
    MotorPI.set(-0.7);

    // Delay más largo por si son 2 pelotas
    Timer.delay(0.12);
    MotorPI.set(0);
    Timer.delay(0.45);
    MotorPI.set(-0.7);
    Timer.delay(0.5);

    // Parar los 3 motores
    MotorPI.set(0);
    shooter1.set(0);
    shooter2.set(0);
  }

  public void autShoot(){

    // Parar el motor de abajo y encender el shooter
    MotorPI.set(0);
    MotorPE.set(0);
    shooter1.set(-1);
    shooter2.set(-1);

    Timer.delay(1);
    // Mover las pelotas para arriba
    MotorPI.set(-0.7);

    // Delay más largo por si son 2 pelotas
    Timer.delay(0.12);
    MotorPI.set(0);
    Timer.delay(0.45);
    MotorPI.set(-0.7);
    Timer.delay(0.5);

    // Parar los 3 motores
    MotorPI.set(0);
    shooter1.set(0);
    shooter2.set(0);
  }


  public void lowShoot(){
    // Mover las pelotas para atrás
    RobotMove(0, 0);
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
  
}
