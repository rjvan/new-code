// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;





import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.Timer;
//Added Imports Below
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String auto = "Defat";
  private static final String kSpeakerMiddleAuto = "Speaker Middle and Backup";
  private static final String kRedLongAuto = "Red Long Speaker and Backup";
  private static final String bad = "bad";
  private static final String leftFrontm = "Left Front Motor";
  private static final String leftRearm = "Left Rear Motor";
  private static final String rightFrontm = "Right Front Motor";
  private static final String rightRearm = "Right Rear Motor";
  private static final String n1ewd = "Top Dropper";
  private static final String n2ewd = "Top Dropper Backup";
  private static final String led1l = "climb";
  private static final String speed = "Drive Speed Foreward/back";
  private static final String leftFrontmt1 = "Left Front Motor";
  private static final String leftRearmt = "Left Rear Motor";
  private static final String rightFrontmt = "Right Front Motor";
  private static final String rightRearmt = "Right Rear Motor";
  private static final String n1ewdt = "Top Dropper";
  private static final String n2ewdt = "Top Dropper Backup";
  private static final String led1lt = "climb";
private static final String motoron = "Motor On?";
private static final String Heading = "heading";
private static final String leftdd = "leftDistance";
private static final String rightdd = "rightDistance";
  private static final String good = "good";
  private static final String climeee = "climbAngle";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private final PWMSparkMax ledrr = new PWMSparkMax(1);
  private final PWMSparkMax lock = new PWMSparkMax(2);
  private final SparkMax leftFront = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax leftRear =  new SparkMax(1, MotorType.kBrushless);
  private final SparkMax rightFront = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax rightRear =   new SparkMax(4, MotorType.kBrushless);
  private final SparkMax n1ew =   new SparkMax(7, MotorType.kBrushless);
  private final SparkMax n2ew =   new SparkMax(8, MotorType.kBrushless);
  private final SparkMax clime =   new SparkMax(9, MotorType.kBrushless);
  private final SparkMax led1 =   new SparkMax(6, MotorType.kBrushed);
  private final MotorControllerGroup leftMotors = new MotorControllerGroup(leftFront, leftRear);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightFront, rightRear);
  private final DifferentialDrive myDrive = new DifferentialDrive(leftMotors, rightMotors);
  public static AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final Timer timer1 = new Timer();
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);
 private final XboxController rum = new XboxController(0);
  private UsbCamera usbcam1 = CameraServer.startAutomaticCapture(0);
  private UsbCamera usbcam2 = CameraServer.startAutomaticCapture(1);
  private UsbCamera usbcam3 = CameraServer.startAutomaticCapture(2);
  double driveLimit = 1;
  double launchPower = 0;
  double feedPower = 0;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Move Forward ", kDefaultAuto);
    m_chooser.addOption("Do Nothing", kSpeakerMiddleAuto );
    m_chooser.addOption("Default Auto drive and score", kRedLongAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_chooser.addOption("long back up and score", auto);
    SmartDashboard.putBoolean(motoron, false);
    
   
  }



  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    usbcam1.setResolution(20, 50);
    usbcam1.setFPS(10);
    usbcam2.setResolution(20, 50);
    usbcam2.setFPS(10);
    usbcam3.setResolution(20, 50);
    usbcam3.setFPS(10);
   SmartDashboard.putNumber(rightFrontm, rightFront.getAppliedOutput());
    SmartDashboard.putNumber(rightRearm, rightRear.getAppliedOutput());
    SmartDashboard.putNumber(leftFrontm, leftFront.getAppliedOutput());
    SmartDashboard.putNumber(leftRearm, leftRear.getAppliedOutput());
    SmartDashboard.putNumber(n1ewd, n1ew.getAppliedOutput());
    SmartDashboard.putNumber(led1l, clime.getAppliedOutput());
    SmartDashboard.putNumber(n2ewd, n2ew.getAppliedOutput());
    double hedr = gyro.getAngle();
    SmartDashboard.putNumber(Heading, hedr);
    
    SmartDashboard.putNumber(leftdd, leftFront.getEncoder().getPosition()/5);
    SmartDashboard.putNumber(rightdd,rightFront.getEncoder().getPosition()/5);
    SmartDashboard.putNumber(climeee,clime.getEncoder().getPosition());
  }

 
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    timer1.reset();
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kRedLongAuto:
        if(timer1.get() < 2.6) { //spool up the launch wheel
         myDrive.tankDrive(.505,.5);
         timer1.start();
        }
        else if(timer1.get() < 4){
myDrive.tankDrive(0,0);
         n1ew.set(.2);
         SmartDashboard.putBoolean(motoron, true);
        }
        else {
          n1ew.set(0);
        }
        break;
        case auto:
        if(timer1.get() < 11 ){
myDrive.tankDrive(.3,.31);
timer1.start();
         }
         else if(timer1.get() < 13){
          myDrive.tankDrive(.6, .6);
         }
        else if(timer1.get() < 15){
          myDrive.tankDrive(0, 0);
          n1ew.set(.2);
          n2ew.set(.2);
          SmartDashboard.putBoolean(motoron, true);
        }
       

        break;
      case kSpeakerMiddleAuto: //start middle speaker launch then backup
        // Put custom auto code here
        if(leftFront.getEncoder().getPosition()/5 < 5) { //spool up the launch wheel
          leftMotors.set(.15);
          rightMotors.set(.15);
          
        }
        else if(gyro.getAngle() > 1) { //turn on feed wheel to launch the note
        }
        break;
      case kDefaultAuto:
      default: //just cross the line backwards for points
        // Put default auto code here 
   
        if (timer1.get() < 2) {
         myDrive.tankDrive(.3, .33);  
         timer1.start();
        }
        else{
          myDrive.tankDrive(0, 0);
    
        }
             break;
    }

  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber(speed, driverController.getRawAxis(1)*driverController.getRawAxis(3)*driverController.getRawAxis(3));
    //drive train code here
    if(operatorController.getPOV() == 270)  {
      led1.set(0);
      ledrr.set(0);
      //rum.setRumble(RumbleType.kLeftRumble, 1.0);
    }
    else {
      led1.set(1);
     ledrr.set(-0.95);
    }
    

    if(operatorController.getRawButton(1)) {
      n1ew.set(.1);
      n2ew.set(.1);
      SmartDashboard.putBoolean(motoron, true);
      //rum.setRumble(RumbleType.kLeftRumble, 1.0);

    }
    
    else if(operatorController.getRawButton(2)) {
      n1ew.set(-.1);
      n2ew.set(-.1);
      SmartDashboard.putBoolean(motoron, true);
      //SmartDashboard.putNumber(good, 1);
    }
   
    
    else if(operatorController.getRawButton(3)) {
      n1ew.set(.4);
      n2ew.set(.4);
      SmartDashboard.putBoolean(motoron, true);
      //SmartDashboard.putNumber(good, 1);
    }
    else if (operatorController.getRawButton(2) == true) {
      n1ew.set(0);
      n2ew.set(0);
      SmartDashboard.putBoolean(motoron, false);
      //SmartDashboard.putNumber(good, 10);
      //rum.setRumble(RumbleType.kBothRumble, 1.0);
    }
    else if(operatorController.getRawButton(4)) {
      n1ew.set(-.4);
      n2ew.set(-.4);
      SmartDashboard.putBoolean(motoron, true);
      //SmartDashboard.putNumber(good, 1);
    }
    else  {
      n1ew.set(0);
      n2ew.set(0);
      SmartDashboard.putBoolean(motoron, false);
      //SmartDashboard.putNumber(good, 10);
      //rum.setRumble(RumbleType.kBothRumble, 1.0);
    }
    //if(operatorController.getRawButton(14) == true) {
      
    //}
    //else if(operatorController.getRawButton(14) == false) {
      
    //}
    if(operatorController.getRawButton(8)) {
      clime.set(-.6);
      lock.set(-1);
    }
    else if(operatorController.getRawButton(7)) {
      clime.set(1);
lock.set(-1);
    }
    else {
      clime.set(0);
      lock.set(0);
    }
    if(driverController.getRawButton(6)) {
      driveLimit = 1;
    }
    else if(driverController.getRawButton(5)) {
      driveLimit = .4;
    }
    myDrive.arcadeDrive(-driverController.getRawAxis(1)*driveLimit, -driverController.getRawAxis(2)*driverController.getRawAxis(3)*driverController.getRawAxis(3));

    //launcher code here
    if(operatorController.getRawButton(1)) {
      launchPower = -1;
      feedPower = -.2;
    }
    else {
      if(operatorController.getRawButtonPressed(2)) {
        timer1.reset();
      }
      if(timer1.get() < 1.0) {
        launchPower = 1;
        feedPower = 0;
      }
      else if(timer1.get() < 2.0) {
        launchPower = 1;
        feedPower = 1;
      }
      else {
      launchPower = 0;
      feedPower = 0;
      }
    }
 
 
  
  }
  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
