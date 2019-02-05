package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.ColorSensor;
import jdk.jfr.Threshold;

import com.ctre.phoenix.motorcontrol.can.*;


public class Robot extends TimedRobot {
    //private static final String kDefaultAuto = "Default";
    //private static final String kCustomAuto = "My Auto";
    //private String m_autoSelected;
    //private final SendableChooser<String> m_chooser = new SendableChooser<>();

    /*************************************************************
                        Motor Controllers
    *************************************************************/
    WPI_TalonSRX frontRightDrive = new WPI_TalonSRX(1);
    WPI_TalonSRX frontLeftDrive = new WPI_TalonSRX(2);
    WPI_TalonSRX rearRightDrive = new WPI_TalonSRX(3);
    WPI_TalonSRX rearLeftDrive = new WPI_TalonSRX(4);
    WPI_TalonSRX crossSlide = new WPI_TalonSRX(5);
    WPI_TalonSRX lLift = new WPI_TalonSRX(6);
    WPI_TalonSRX rLift = new WPI_TalonSRX(7);
    WPI_TalonSRX lIntake = new WPI_TalonSRX(8);
    WPI_TalonSRX rIntake = new WPI_TalonSRX(9);
    WPI_TalonSRX rearClimb = new WPI_TalonSRX(10);
    WPI_TalonSRX frontClimb = new WPI_TalonSRX(11);
    
    //WPI_VictorSPX victor1 = new WPI_VictorSPX(12);
    //WPI_VictorSPX victor2 = new WPI_VictorSPX(13);

    //Spark spark0 = new Spark(0);
    //Spark spark1 = new Spark(1);



    Encoder armEnc = new Encoder(6,7, false, Encoder.EncodingType.k4X);
    ColorSensor colorSensor = new ColorSensor(I2C.Port.kOnboard);
    
    

    /***********************************************************
                         Limit Switches
    ************************************************************/
    DigitalInput rSlideSensor = new DigitalInput(0);
    DigitalInput lSlideSensor = new DigitalInput(1);
    DigitalInput bLiftSwitch = new DigitalInput(2);
    DigitalInput tLiftSwitch = new DigitalInput(3);
    DigitalInput proxySensor = new DigitalInput(4);

    /*************************************************************
                            Servo Motors
    ***************************************************************/
    Servo rDriveServo = new Servo(8);
    Servo lDriveServo = new Servo(9);

  
    /***********************************************************
                            Drive Code
    ************************************************************/
    SpeedControllerGroup rightDrive = new SpeedControllerGroup(frontRightDrive, rearRightDrive);
    SpeedControllerGroup leftDrive = new SpeedControllerGroup(frontLeftDrive, rearLeftDrive);
    DifferentialDrive diffDrive = new DifferentialDrive(leftDrive, rightDrive);
    

    Joystick right = new Joystick(1);
    /***********************************************************
                      ADDITIONAL VARIABLES
    ***********************************************************/

    boolean tog = false;
    boolean flag = false;
    static double threshold = 10;
    int slideTarget = 0;
    int colorThreshhold = 20;
    double encoderCount;
    double encoderDegree;
    boolean lowGear = false;

 
  
    /**********************************************************
     ----------------------------------------------------------
                        ROBOT FUNCTIONS
     ----------------------------------------------------------
    **********************************************************/

    @Override
    public void robotInit() {
        //limitSwitch = new DigitalInput(0);
        //m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        //m_chooser.addOption("My Auto", kCustomAuto);
        //SmartDashboard.putData("Auto choices", m_chooser);
        armEnc.reset();
        CameraServer.getInstance().startAutomaticCapture();
    }
 
    @Override
    public void robotPeriodic() {
        //testDrive.arcadeDrive(right.getY(), right.getX());
        //SmartDashboard.putBoolean("TestLS", limitSwitch.get());
    }

    @Override
    public void autonomousInit() {
        //m_autoSelected = m_chooser.getSelected();
        //m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
        // system.out.println("Auto selected: " + m_autoselected);
    }

    @Override
    public void autonomousPeriodic() {
        /*switch (m_autoSelected){
        case kCustomAuto:
            break;
        case kDefaultAuto:
        default:
            break;
        }*/
        teleopPeriodic();
    }

/**************************************************************
---------------------------------------------------------------
TELEOP TELEOP TELEOP TELEOP TELEOP TELEOP TELEOP TELEOP TELOP
---------------------------------------------------------------
***************************************************************/
  @Override
  public void teleopPeriodic() {
    
    diffDrive.arcadeDrive(right.getRawAxis(1), right.getRawAxis(0) * -1);

    boolean trigger = right.getRawButton(1);
    SmartDashboard.putBoolean("trigger", trigger);
    boolean bottomFace = right.getRawButton(2);
    SmartDashboard.putBoolean("bottomFace", bottomFace);
    boolean leftFace = right.getRawButton(3);
    SmartDashboard.putBoolean("leftFace", leftFace);
    boolean rightFace = right.getRawButton(4);
    SmartDashboard.putBoolean("rightFace", rightFace);
    boolean L1 = right.getRawButton(5);
    SmartDashboard.putBoolean("L1", L1);
    boolean R2 = right.getRawButton(12);
    SmartDashboard.putBoolean("R2", R2);    
    boolean R3 = right.getRawButton(11);
    SmartDashboard.putBoolean("R3", R3);
    boolean R6 = right.getRawButton(14);
    SmartDashboard.putBoolean("R6", R6);
    boolean R5 = right.getRawButton(15);
    SmartDashboard.putBoolean("rR5", R5);    
    boolean R4 = right.getRawButton(16);
    SmartDashboard.putBoolean("rR4", R4);
    boolean L3 = right.getRawButton(7);
    SmartDashboard.putBoolean("L3", L3);
    boolean L2 = right.getRawButton(6);
    SmartDashboard.putBoolean("L2", L2);
    boolean R1 = right.getRawButton(13);
    SmartDashboard.putBoolean("R1", R1);
    boolean L4 = right.getRawButton(10);
    SmartDashboard.putBoolean("L4", L4);
    boolean L5 = right.getRawButton(9);
    SmartDashboard.putBoolean("L5", L5);
    boolean L6 = right.getRawButton(8);
    SmartDashboard.putBoolean("L6", L6);
    int rPOV = right.getPOV();
    SmartDashboard.putNumber("rPOV", rPOV);
    int colorTotal = colorSensor.red + colorSensor.green + colorSensor.blue;

    if(trigger){
      lowGear = true;
    } else{
      lowGear = false;
    }
    
    
    if(!lowGear) { //servo code changes servo position based on boolean input
      rDriveServo.setAngle(120);
      lDriveServo.setAngle(30);
    } else {
      rDriveServo.setAngle(30);
      lDriveServo.setAngle(100);
    }
  
   /* if() {   //Cayden wrote some code for a toggle as a practice exercise, it works so use it if you want to
      if(flag) {
        if(tog) {
          tog = false;
          } else {
            tog = true;
            }
      flag = false;
    }
  } else {
    flag = true;
  }

    if(tog) {
      talon5.set(1);
    } else {
      talon5.set(0);
    }*/

    /*
    
    *********************************************************
      CROSS SLIDE CODE
    ********************************************************/

    if (rightFace && !rSlideSensor.get()) {
      slideTarget = 1;
    } else {
       if (leftFace && !lSlideSensor.get()) {
         slideTarget = -1;
      } else {
          if ((rSlideSensor.get() && slideTarget == 1) || (lSlideSensor.get() && slideTarget == -1) || colorTotal > colorThreshhold || right.getPOV() == 90 || right.getPOV() == 270) {
            slideTarget = 0;
          }
        }
     }


    if(right.getPOV() == 90 || slideTarget == 1 || right.getPOV() == 45 || right.getPOV() == 135) {
        if(!rSlideSensor.get()){
            crossSlide.set(1);
        }
    }
    else{
        if(right.getPOV() == 270 || slideTarget == -1 || right.getPOV() == 315 || right.getPOV() == 225){
            if(!lSlideSensor.get()){
              crossSlide.set(-1);
            }
        }
        else{
            crossSlide.set(0.0); 
        }
    }
   /**********************************************************
                            LIFT CODE
    *********************************************************/

    if(right.getPOV() == 0){
        if(!tLiftSwitch.get()){
            lLift.set(0.5);
            rLift.set(0.5);
        }
    }
      else{
          if(right.getPOV() == 180){
              if(!bLiftSwitch.get()){
                  lLift.set(-0.5);
                  rLift.set(-0.5);
              }
          }
          else{
             lLift.set(0.0);
             rLift.set(0.0);
           
         }
       }
       //run talon 10 until no buttons are pressed or a limit switch(DIO4)is pressed
       //two buttons are used R3 and L3, R3 moves it forward, L3 moves it backward
      
    
  
       
      /**************************************************************
                Encoder Code
      ***************************************************************/

      encoderCount = armEnc.getRaw();
      

    //4063counts per revolution/18.5 gear down = 1 revolution
    //208.79 counts = 1 degree

    // OUTPUTS TO SHUFFLEBOARD/SMARTDASHBOARD, USE FOR TRACKING VALUES
    SmartDashboard.putNumber("JoyStickXVal", right.getX());
    SmartDashboard.putNumber("JoyStickYVal", right.getY());
    SmartDashboard.putBoolean("TestShoot", right.getRawButton(1));
    SmartDashboard.putNumber("Left Servo", rDriveServo.getAngle());
    SmartDashboard.putNumber("Right Servo", lDriveServo.getAngle());
    SmartDashboard.putNumber("Slide", crossSlide.get());
    SmartDashboard.putBoolean("left", lSlideSensor.get());
    SmartDashboard.putBoolean("right", rSlideSensor.get());
    SmartDashboard.putNumber("target", slideTarget);
    SmartDashboard.putNumber("color total", colorTotal);
    SmartDashboard.putNumber("Arm Encoder", encoderCount);
    SmartDashboard.putBoolean("Proximity Sensor", proxySensor.get());
    SmartDashboard.putBoolean("Low Gear", lowGear);

    colorSensor.read();
    SmartDashboard.putNumber("test", colorSensor.status());
    SmartDashboard.putNumber("red", colorSensor.red);
    SmartDashboard.putNumber("green", colorSensor.green);
    SmartDashboard.putNumber("blue", colorSensor.blue);
    SmartDashboard.putNumber("prox", colorSensor.prox);
  }
  

  /***********************************************************
                      Test Code
  ***********************************************************/

  //---------------------------------------------------------\\
  @Override
  public void testPeriodic() {

  }

  
  /**********************************************************
                      User Functions
  ***********************************************************/
  boolean checkColor(double threshold){ //returns if white tape if found, recommended threshold is 10
    if(colorSensor.red > threshold && colorSensor.green > threshold && colorSensor.blue > threshold){
      return true;
    }
    else{
      return false;
    }
  }

}
