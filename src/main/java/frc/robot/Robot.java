package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
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
    WPI_TalonSRX bottomRightDrive = new WPI_TalonSRX(1);
    WPI_VictorSPX bottomLeftDrive = new WPI_VictorSPX(2);
    WPI_VictorSPX topRightDrive = new WPI_VictorSPX(3);
    WPI_TalonSRX topLeftDrive = new WPI_TalonSRX(4);
    WPI_TalonSRX lLift = new WPI_TalonSRX(5);
    WPI_TalonSRX crossSlide = new WPI_TalonSRX(6);
    WPI_TalonSRX rLift = new WPI_TalonSRX(7);
    WPI_TalonSRX lIntake = new WPI_TalonSRX(8);
    WPI_TalonSRX rIntake = new WPI_TalonSRX(9);
    WPI_TalonSRX frontClimb = new WPI_TalonSRX(10);
    WPI_TalonSRX rearClimb = new WPI_TalonSRX(11);
    WPI_TalonSRX intakeArm = new WPI_TalonSRX(14);

    Spark lLift2 = new Spark(5);
    
    Spark rLift2 = new Spark(4);

    WPI_TalonSRX rRearArmDrive = new WPI_TalonSRX(12);
    WPI_TalonSRX lRearArmDrive = new WPI_TalonSRX(13);

    // Spark rLiftTest = new Spark(0);
    // Spark lLiftTest = new Spark(1);

    ColorSensor colorSensor = new ColorSensor(I2C.Port.kOnboard);

    /***********************************************************
                         Limit Switches
    ************************************************************/
    DigitalInput rSlideSensor = new DigitalInput(0);
    DigitalInput lSlideSensor = new DigitalInput(1);
    DigitalInput cSlideSensor = new DigitalInput(2);
    DigitalInput rearClimbStop = new DigitalInput(3);
    DigitalInput frontClimbStop = new DigitalInput(4);
    DigitalInput zeroArm = new DigitalInput(5);
    Encoder armEnc = new Encoder(6,7,false, Encoder.EncodingType.k4X);
    DigitalInput ballSwitch = new DigitalInput(8);
    

    
  
    /***********************************************************
                            Drive Code
    ************************************************************/
    SpeedControllerGroup rightDrive = new SpeedControllerGroup(bottomRightDrive, topRightDrive, rRearArmDrive);
    SpeedControllerGroup leftDrive = new SpeedControllerGroup(bottomLeftDrive, topLeftDrive, lRearArmDrive);
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
    int lastKnownSlidePosition = 0;
    int encoderCount;
    double encoderDegree;
    boolean highGear = false;
    int armtarget = 1;
    Timer timer = new Timer();
    int Armarray[] = {10,20,30,40}; 

    boolean rearHold;
    boolean frontHold;

    int armPos = 0;

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
        highGear = true;
        timer.start();
        CameraServer.getInstance().startAutomaticCapture();

        

        topLeftDrive.setInverted(true);
        bottomRightDrive.setInverted(true);
        lRearArmDrive.setInverted(true);
        
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
    public void teleopPeriodic()
    {
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
        boolean L2 = right.getRawButton(6);
        boolean L3 = right.getRawButton(7);
        boolean L6 = right.getRawButton(8);
        boolean L5 = right.getRawButton(9);
        boolean L4 = right.getRawButton(10);
        
        boolean R1 = right.getRawButton(13);
        boolean R2 = right.getRawButton(12);
        boolean R3 = right.getRawButton(11);
        boolean R4 = right.getRawButton(14);
        boolean R5 = right.getRawButton(15);
        boolean R6 = right.getRawButton(16);
        SmartDashboard.putBoolean("L1", L1);
        SmartDashboard.putBoolean("L3", L3);
        SmartDashboard.putBoolean("L2", L2);
        SmartDashboard.putBoolean("L4", L4);
        SmartDashboard.putBoolean("L5", L5);
        SmartDashboard.putBoolean("L6", L6);
        SmartDashboard.putBoolean("R1", R1);
        SmartDashboard.putBoolean("R2", R2);
        SmartDashboard.putBoolean("R3", R3);
        SmartDashboard.putBoolean("R4", R4);
        SmartDashboard.putBoolean("R5", R5);
        SmartDashboard.putBoolean("R6", R6);
        SmartDashboard.putBoolean("Ball Switch", ballSwitch.get());

        SmartDashboard.putBoolean("frontClimbStop", frontClimbStop.get());
        SmartDashboard.putBoolean("rearClimbStop", rearClimbStop.get());

        int rPOV = right.getPOV();
        SmartDashboard.putNumber("rPOV", rPOV);
        int colorTotal = colorSensor.red + colorSensor.green + colorSensor.blue;

        if(L1 && !highGear && timer.get() > .5 ){
            timer.reset();
            highGear = true;
            timer.start();
        } else {
            if(highGear && L1 && timer.get() > .5) {
                timer.reset();
                highGear = false;
                timer.start();
            }
        }
        
        
       
        
    
        /*********************************************************
         CROSS SLIDE CODE
        ********************************************************/
        // slideTargets
        // 0 - center
        // 1 - right OR TAPE
        // 2 - left OR TAPE
        // 3 - Manual override right
        // 4 - Manual override left

        double CROSS_SLIDE_SPEED = 1.0;

        if (rightFace)
            slideTarget = 1;
        else if (leftFace)
            slideTarget = 2;
        else if (right.getPOV() == 90) 
            slideTarget = 3;
        else if (right.getPOV() == 270) 
            slideTarget = 4;
        else
            slideTarget = 0;

        // So we can find center faster, remember if we hit left or right last
        if(lSlideSensor.get())
            lastKnownSlidePosition = 2;
        else if (rSlideSensor.get())
            lastKnownSlidePosition = 1;

        if(slideTarget == 1) {
            // Target is right
            if(!rSlideSensor.get() || colorTotal < colorThreshhold)
                crossSlide.set(CROSS_SLIDE_SPEED);
            else
                crossSlide.set(0.0); // At right, stop
        } else if(slideTarget == 2) {
            // Target is left
            if(!lSlideSensor.get() || colorTotal < colorThreshhold)
                crossSlide.set(-CROSS_SLIDE_SPEED);
            else
                crossSlide.set(0.0); // At left, stop
        } else if(slideTarget == 0) {
            // Stay center
            if(!cSlideSensor.get()) {
                if(lastKnownSlidePosition == 1)
                    crossSlide.set(CROSS_SLIDE_SPEED);
                else if (lastKnownSlidePosition == 2)
                    crossSlide.set(-CROSS_SLIDE_SPEED);
            } else
                crossSlide.set(0.0); // At center, stop
        } else if (slideTarget == 3) {
            // Manual override right
            if(right.getPOV() == 90 && !rSlideSensor.get())
                crossSlide.set(CROSS_SLIDE_SPEED);
            else
                crossSlide.set(0.0);
        } else if (slideTarget == 4) {
            // Manual override left
            if(right.getPOV() == 270 && !lSlideSensor.get())
                crossSlide.set(-CROSS_SLIDE_SPEED);
            else
                crossSlide.set(0.0);
        }

        /*********************************************************
                                Climb                 
        *********************************************************/
        double downPressure = .2;
        
        frontClimb.setInverted(true);
        rearClimb.setInverted(true);

        if(L2){
            frontClimb.set(-.5);
            frontHold = false;
          }       
        else {
            if(L5 && !frontClimbStop.get()){
                frontClimb.set(.8);
                frontHold = true;
            }
            else{
                if(frontHold){
                    frontClimb.set(downPressure);
                } else{
                    if(frontClimbStop.get()){
                        frontClimb.set(downPressure);
                    } else
                    frontClimb.set(0);
                }
            }
        }
        
        if(L3){
            rearClimb.set(.5);
            frontHold = false;
          }       
        else {
            if(L6 && !rearClimbStop.get()){
                rearClimb.set(-.8);
                frontHold = true;
            }
            else{
                if(frontHold){
                    rearClimb.set(-downPressure);
                } else{
                    if(rearClimbStop.get()){
                        rearClimb.set(-downPressure);
                    } else
                    rearClimb.set(0);
                }
            }
        }

        


        //window

        // if(R11) {
        //     rRearArmDrive.set(1);
        //     lRearArmDrive.set(1);
        // } else {
        //     if(R16) {
        //         rRearArmDrive.set(-1);
        //         lRearArmDrive.set(-1);
        //     } else {
        //         rRearArmDrive.set(0);
        //         lRearArmDrive.set(0);
        //     }
        // }



        /**********************************************************
                                    LIFT CODE
            *********************************************************/
        double LIFT_POWER = 0.6;
        //lLift.setInverted(true);
        if(right.getPOV() == 0) {
            lLift.set(LIFT_POWER);
            rLift.set(LIFT_POWER);
            lLift2.set(LIFT_POWER);
            rLift2.set(LIFT_POWER);
        } else {
            if(right.getPOV() == 180) {
                lLift.set(-LIFT_POWER);
                rLift.set(-LIFT_POWER);
                lLift2.set(-LIFT_POWER);
                rLift2.set(-LIFT_POWER);
            } else {
                lLift.set(0.0);
                rLift.set(0.0);
                lLift2.set(0.0);
                rLift2.set(0.0);
            }
        }

       ////////Intake////////////////

        if(trigger) {
            lIntake.set(1);
            rIntake.set(1);
        } else {
            if(bottomFace) {
                lIntake.set(-1);
                rIntake.set(-1);
            } else {
                lIntake.set(0);
                rIntake.set(0);
            }
        }

        double ARM_UP_SPEED = 0.8;
        double ARM_DOWN_SPEED = -0.5;
        double ARM_HOLD_POWER = 0.1;
        int DEAD_ZONE = 25;

        int SLOWER_TO_HOME = -560;
        int WALL_POS = -560;
        int BALL_PICKUP_POS = -1220;
        int FLOOR_POS = -1950;

        // Home Position
        if(R1)
            armPos = 0;
        
        // Ball Pickup Position
        if(R4)
            armPos = 2;
        
        // Wall Position
        if(R2)
            armPos = 1;

        // Floor Position
        if(R5)
            armPos = 3;
        
        // Manual Override Up
        if(R3)
            armPos = 4;
    
        // Manual Override Down
        if(R6)
            armPos = 5;

        // Automatically bring ball in
        if(ballSwitch.get() && armPos == 2)
            armPos = 0;
    
        encoderCount = armEnc.getRaw();
        // double PERC_TO_TOP = (encoderCount / FLOOR_POS);

        switch(armPos) {
            case 0:
                if(!zeroArm.get()) {
                    if (encoderCount > SLOWER_TO_HOME) {
                        intakeArm.set(ARM_UP_SPEED * 0.5);
                    } else {
                        intakeArm.set(ARM_UP_SPEED);
                    }
                } else {
                    armEnc.reset();
                    intakeArm.set(0);
                }
                break;
            case 1:
                if(encoderCount < (WALL_POS - DEAD_ZONE))
                    intakeArm.set(ARM_UP_SPEED);
                else {
                    if(encoderCount > (WALL_POS + DEAD_ZONE)) {
                        intakeArm.set(ARM_DOWN_SPEED);
                    } else{
                        intakeArm.set(ARM_HOLD_POWER);
                    }
                }
                break;
            case 2:
                if(encoderCount < (BALL_PICKUP_POS - DEAD_ZONE))
                    intakeArm.set(ARM_UP_SPEED);
                else {
                    if(encoderCount > (BALL_PICKUP_POS + DEAD_ZONE)) {
                        intakeArm.set(ARM_DOWN_SPEED);
                        lIntake.set(1);
                        rIntake.set(1);
                    } else{
                        intakeArm.set(ARM_HOLD_POWER);
                    }
                }
                break;
            case 3:
                if(encoderCount < (FLOOR_POS - DEAD_ZONE))
                    intakeArm.set(ARM_UP_SPEED);
                else {
                    if(encoderCount > (FLOOR_POS + DEAD_ZONE)) {
                        intakeArm.set(ARM_DOWN_SPEED);
                    } else{
                        intakeArm.set(ARM_HOLD_POWER);
                    }
                }
                break;
            case 4:
                if(R3) {
                    if(zeroArm.get()) {
                        intakeArm.set(0.0);
                    } else {
                        intakeArm.set(ARM_UP_SPEED);
                    }
                } else {
                    intakeArm.set(0.0);
                }
                break;
            case 5:
                if(R6) {
                    if(encoderCount < (FLOOR_POS - DEAD_ZONE)) {
                        intakeArm.set(0.0);
                    } else {
                        intakeArm.set(ARM_DOWN_SPEED);
                    }
                } else {
                    intakeArm.set(0.0);
                }
                break;
            }
      
    
        //run talon 10 until no buttons are pressed or a limit switch(DIO4)is pressed
        //two buttons are used R3 and L3, R3 moves it forward, L3 moves it backward

        //4063counts per revolution/18.5 gear down = 1 revolution
        //208.79 counts = 1 degree

        if(right.getPOV() == 90)
            crossSlide.set(.5);
        else if(right.getPOV() == 270)
            crossSlide.set(-.5);
        else
            crossSlide.set(0);





        
    

        // OUTPUTS TO SHUFFLEBOARD/SMARTDASHBOARD, USE FOR TRACKING VALUES
        SmartDashboard.putNumber("JoyStickXVal", right.getX());
        SmartDashboard.putNumber("JoyStickYVal", right.getY());
        SmartDashboard.putBoolean("TestShoot", right.getRawButton(1));
        //SmartDashboard.putNumber("Left Servo", rDriveServo.getAngle());
        //SmartDashboard.putNumber("Right Servo", lDriveServo.getAngle());
        SmartDashboard.putNumber("Slide", crossSlide.get());
        SmartDashboard.putBoolean("left", lSlideSensor.get());
        SmartDashboard.putBoolean("right", rSlideSensor.get());
        SmartDashboard.putNumber("target", slideTarget);
        SmartDashboard.putNumber("color total", colorTotal);
        SmartDashboard.putNumber("Arm Encoder", encoderCount);
        SmartDashboard.putBoolean("Proximity Sensor", cSlideSensor.get());
        SmartDashboard.putBoolean("High Gear", highGear);

        colorSensor.read();
        //SmartDashboard.putNumber("test", colorSensor.status());
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
