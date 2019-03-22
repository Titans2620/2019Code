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
import edu.wpi.first.wpilibj.SendableBase;

import com.ctre.phoenix.motorcontrol.can.*;


public class Robot extends TimedRobot {
    /*************************************************************
                        Motor Controllers
    *************************************************************/
    WPI_VictorSPX bottomLeftDrive = new WPI_VictorSPX(2);
    WPI_TalonSRX topLeftDrive = new WPI_TalonSRX(1);
    WPI_VictorSPX topRightDrive = new WPI_VictorSPX(3);
<<<<<<< HEAD
    WPI_TalonSRX bottomRightDrive = new WPI_TalonSRX(4);
    WPI_TalonSRX frontClimb2 = new WPI_TalonSRX(5);
    WPI_TalonSRX frontClimb = new WPI_TalonSRX(6);
    WPI_TalonSRX rRearArmDrive = new WPI_TalonSRX(7);
    WPI_TalonSRX lRearArmDrive = new WPI_TalonSRX(8);
    WPI_TalonSRX rearClimb = new WPI_TalonSRX(9);
    WPI_TalonSRX rearClimb2 = new WPI_TalonSRX(10);
    //WPI_TalonSRX lift = new WPI_TalonSRX(11);
    // WPI_TalonSRX rLift = new WPI_TalonSRX(12);
    WPI_TalonSRX crossSlide = new WPI_TalonSRX(13);
    // WPI_VictorSPX intakeArm = new WPI_VictorSPX(14);
   // WPI_VictorSPX intake = new WPI_VictorSPX(15);

    Spark tLeftDrive = new Spark(0);
    Spark tRightDrive = new Spark(1);
    Spark intake = new Spark(2);
    Spark lift = new Spark(3);
    WPI_TalonSRX intakeArm = new WPI_TalonSRX(14);
=======
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
>>>>>>> parent of a6ad3dd... ws

    ColorSensor colorSensor = new ColorSensor(I2C.Port.kOnboard);

    /***********************************************************
                         Limit Switches
    ************************************************************/
    DigitalInput rSlideSensor = new DigitalInput(0);
    DigitalInput lSlideSensor = new DigitalInput(1);
    //DigitalInput cSlideSensor = new DigitalInput(2);
    DigitalInput bottomLift = new DigitalInput(2);
    DigitalInput frontClimbStop = new DigitalInput(3);
    DigitalInput rearClimbStop = new DigitalInput(4);
    DigitalInput zeroArm = new DigitalInput(5);
    Encoder armEnc = new Encoder(6,7,false, Encoder.EncodingType.k4X);
    DigitalInput ballSwitch = new DigitalInput(8);
    DigitalInput topLift = new DigitalInput(9);
    
    

    
    
  
    /***********************************************************
                            Drive Code
    ************************************************************/
    
    
    
    SpeedControllerGroup rightDrive = new SpeedControllerGroup(bottomRightDrive, topRightDrive, rRearArmDrive);
    SpeedControllerGroup leftDrive = new SpeedControllerGroup(bottomLeftDrive, topLeftDrive, lRearArmDrive);
    DifferentialDrive diffDrive = new DifferentialDrive(leftDrive, rightDrive);
    DifferentialDrive tDiffDrive = new DifferentialDrive(tLeftDrive, tRightDrive);
    Joystick left = new Joystick(0);
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
    Timer timerPID = new Timer();
    int Armarray[] = {10,20,30,40}; 

    double PID;

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
        
        timer.start();
        CameraServer.getInstance().startAutomaticCapture();

        


        
        topRightDrive.setInverted(true);
        bottomRightDrive.setInverted(true);
        lRearArmDrive.setInverted(true);
        intake.setInverted(true);
        frontClimb.setInverted(true);
        rearClimb.setInverted(true);
        frontClimb2.setInverted(true);
        rearClimb2.setInverted(true);
        lift.setInverted(true);
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
        
        

        diffDrive.arcadeDrive(right.getRawAxis(1), right.getRawAxis(0));
        tDiffDrive.arcadeDrive(right.getRawAxis(1) * -1, right.getRawAxis(0) * .80 );
        
        boolean lTrigger = left.getRawButton(1);
        boolean rTrigger = right.getRawButton(1);
        SmartDashboard.putBoolean("trigger", lTrigger);
        boolean rBottomFace = right.getRawButton(2);
        boolean lBottomFace = left.getRawButton(2);
        SmartDashboard.putBoolean("bottomFace", lBottomFace);
        boolean rLeftFace = right.getRawButton(3);
        boolean lLeftFace = left.getRawButton(3);
        SmartDashboard.putBoolean("leftFace", lLeftFace);
        boolean rRightFace = right.getRawButton(4);
        boolean lRightFace = left.getRawButton(4);
        SmartDashboard.putBoolean("rightFace", lRightFace);
        //RIGHT(LEFT SIDE)
        boolean RL1 = right.getRawButton(5);
        boolean RL2 = right.getRawButton(6);
        boolean RL3 = right.getRawButton(7);
        boolean RL6 = right.getRawButton(8);
        boolean RL5 = right.getRawButton(9);
        boolean RL4 = right.getRawButton(10);
        //RIGHT(RIGHT SIDE)
        boolean RR1 = right.getRawButton(13);
        boolean RR2 = right.getRawButton(12);
        boolean RR3 = right.getRawButton(11);
        boolean RR4 = right.getRawButton(14);
        boolean RR5 = right.getRawButton(15);
        boolean RR6 = right.getRawButton(16);

        //LEFT(LEFT SIDE)
        boolean LL1 = left.getRawButton(5);
        boolean LL2 = left.getRawButton(6);
        boolean LL3 = left.getRawButton(7);
        boolean LL6 = left.getRawButton(8);
        boolean LL5 = left.getRawButton(9);
        boolean LL4 = left.getRawButton(10);
        //LEFT(RIGHT SIDE)
        boolean LR1 = left.getRawButton(13);
        boolean LR2 = left.getRawButton(12);
        boolean LR3 = left.getRawButton(11);
        boolean LR4 = left.getRawButton(14);
        boolean LR5 = left.getRawButton(15);
        boolean LR6 = left.getRawButton(16);

        SmartDashboard.putBoolean("ArmZero", zeroArm.get());
        SmartDashboard.putBoolean("Ball Switch", ballSwitch.get());
        SmartDashboard.putNumber("Ender Coder", encoderCount);
        SmartDashboard.putBoolean("frontClimbStop", frontClimbStop.get());
        SmartDashboard.putBoolean("bottomstopp", bottomLift.get());

        int rPOV = right.getPOV();
        SmartDashboard.putNumber("rPOV", rPOV);
        int colorTotal = colorSensor.red + colorSensor.green + colorSensor.blue;

        
    
        /*********************************************************
         CROSS SLIDE CODE
        ********************************************************/
        // slideTargets
        // 0 - center
        // 1 - right OR TAPE
        // 2 - left OR TAPE
        // 3 - Manual override right
        // 4 - Manual override left

        // double CROSS_SLIDE_SPEED = 1.0;

        // if (rightFace)
        //     slideTarget = 1;
        // else if (leftFace)
        //     slideTarget = 2;
        // else if (right.getPOV() == 90) 
        //     slideTarget = 3;
        // else if (right.getPOV() == 270) 
        //     slideTarget = 4;
        // else
        //     slideTarget = 0;

        // // So we can find center faster, remember if we hit left or right last
        // if(lSlideSensor.get())
        //     lastKnownSlidePosition = 2;
        // else if (rSlideSensor.get())
        //     lastKnownSlidePosition = 1;

        // if(slideTarget == 1) {
        //     // Target is right
        //     if(!rSlideSensor.get() || colorTotal < colorThreshhold)
        //         crossSlide.set(CROSS_SLIDE_SPEED);
        //     else
        //         crossSlide.set(0.0); // At right, stop
        // } else if(slideTarget == 2) {
        //     // Target is left
        //     if(!lSlideSensor.get() || colorTotal < colorThreshhold)
        //         crossSlide.set(-CROSS_SLIDE_SPEED);
        //     else
        //         crossSlide.set(0.0); // At left, stop
        // } else if(slideTarget == 0) {
        //     // Stay center
        //     if(!cSlideSensor.get()) {
        //         if(lastKnownSlidePosition == 1)
        //             crossSlide.set(CROSS_SLIDE_SPEED);
        //         else if (lastKnownSlidePosition == 2)
        //             crossSlide.set(-CROSS_SLIDE_SPEED);
        //     } else
        //         crossSlide.set(0.0); // At center, stop
        // } else if (slideTarget == 3) {
        //     // Manual override right
        //     if(right.getPOV() == 90 && !rSlideSensor.get())
        //         crossSlide.set(CROSS_SLIDE_SPEED);
        //     else
        //         crossSlide.set(0.0);
        // } else if (slideTarget == 4) {
        //     // Manual override left
        //     if(right.getPOV() == 270 && !lSlideSensor.get())
        //         crossSlide.set(-CROSS_SLIDE_SPEED);
        //     else
        //         crossSlide.set(0.0);
        // }

        /*********************************************************
                                Climb                 
        *********************************************************/
        double downPressure = .2;


        //Climb (Front)

        if(RL2){
            frontClimb.set(-.5);
            frontClimb2.set(-.5);
            frontHold = false;
          }       
        else {
            if(RL5 && !frontClimbStop.get()){
                frontClimb.set(.8);
                frontClimb2.set(.8);
                frontHold = true;
            }
            else{
                if(frontHold){
                    frontClimb.set(downPressure);
                    frontClimb2.set(downPressure);
                } else{
                    if(frontClimbStop.get()){
                        frontClimb.set(downPressure);
                        frontClimb2.set(downPressure);
                    } else
                    frontClimb.set(0);
                    frontClimb2.set(0);
                }
            }
        }
        
        //Climb (Rear)

        if(RL3){
            rearClimb.set(.5);
            rearClimb2.set(.5);
            rearHold = false;
          }       
        else {
            if(RL6 && !rearClimbStop.get()){
                rearClimb.set(-.8);
                rearClimb2.set(-.8);
                rearHold = true;
            }
            else{
                if(rearHold){
                    rearClimb.set(-downPressure);
                    rearClimb2.set(-downPressure);
                } else{
                    if(rearClimbStop.get()){
                        rearClimb.set(-downPressure);
                        rearClimb2.set(-downPressure);
                    } else
                    rearClimb.set(0);
                    rearClimb2.set(0);
                }
            }
        }

        /**********************************************************
                                    LIFT CODE
            *********************************************************/
<<<<<<< HEAD
            
        double LIFT_POWER = 0.85;
        //(UP)
        if(left.getPOV() == 0)// && !topLift.get())
            lift.set(LIFT_POWER);
        else {
        //(Down)
            if(left.getPOV() == 180 && !bottomLift.get())
                lift.set(-.35);
            else 
                lift.set(0.0);
=======
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
>>>>>>> parent of a6ad3dd... ws
        }

       ////////Intake////////////////

        if(lTrigger)//(IN)
           // if(!ballSwitch.get())
                intake.set(0.7);
            //else
               // intake.set(0);
        else {
            if(lBottomFace)//(OUT)
                intake.set(-1);
           // else if (ballSwitch.get())
                //intake.set(0.0);
            else
                intake.set(0);
        }


        //Speed
        double ARM_UP_SPEED = 0.8;
        double ARM_DOWN_SPEED = -0.5;
        double ARM_HOLD_POWER = 0.0;
        int DEAD_ZONE = 25;
        //Positions
        int SLOWER_TO_HOME = -560;
        int WALL_POS = -150;
        int BALL_PICKUP_POS = -1220;
        int FLOOR_POS = -2100;

        //Home Position
        if(LR1)
            armPos = 0;        
        // Wall Position
        if(LR2)
            armPos = 1;
        // Ball Pickup Position
        if(LR4)
            armPos = 2;
        // Floor Position
        if(LR5)
            armPos = 3;
        // Manual Override Up
        if(LR3)
            armPos = 4;
        // Manual Override Down
        if(LR6)
            armPos = 5;

        // Automatically bring ball in
        if(ballSwitch.get() && armPos == 2)
            armPos = 0;
    
        encoderCount = armEnc.getRaw() * -1;
        // double PERC_TO_TOP = (encoderCount / FLOOR_POS);

        if(zeroArm.get())
            armEnc.reset();
        

        switch(armPos) {
            case 0://(HOME)
                if(!zeroArm.get()) {
                    if (encoderCount > SLOWER_TO_HOME) 
                        intakeArm.set(ARM_UP_SPEED * 0.5);
                    else 
                        intakeArm.set(ARM_UP_SPEED);
                } else 
                    intakeArm.set(0);
                break;
            case 1://(WALL)
                if(encoderCount < (WALL_POS - DEAD_ZONE))
                    intakeArm.set(ARM_UP_SPEED);
                else {
                    if(encoderCount > (WALL_POS + DEAD_ZONE))
                        intakeArm.set(ARM_DOWN_SPEED);
                    else
                        intakeArm.set(ARM_HOLD_POWER);
                }
                break;
            case 2://(BALL)
                if(encoderCount < (BALL_PICKUP_POS - DEAD_ZONE))
                    intakeArm.set(ARM_UP_SPEED);
                else {
                    if(encoderCount > (BALL_PICKUP_POS + DEAD_ZONE)) {
                        intakeArm.set(ARM_DOWN_SPEED);
                        intake.set(1);
                    } else
                        intakeArm.set(ARM_HOLD_POWER);
                }
                break;
            case 3://(FLOOR)
                if(encoderCount < (FLOOR_POS - DEAD_ZONE))
                    intakeArm.set(ARM_UP_SPEED);
                else {
                    if(encoderCount > (FLOOR_POS + DEAD_ZONE))
                        intakeArm.set(ARM_DOWN_SPEED);
                    else
                        intakeArm.set(ARM_HOLD_POWER);
                }
                break;
            case 4://MANUAL(UP)
                if(zeroArm.get())
                    intakeArm.set(0.0);
                else {
                    if(LR3)
                        intakeArm.set(ARM_UP_SPEED);
                    else
                        intakeArm.set(ARM_HOLD_POWER);
                
                }
                break;
            case 5://MANUAL(DOWN)
                if(encoderCount < (FLOOR_POS - DEAD_ZONE))
                    intakeArm.set(0.0);
                else {
                    if(LR6) 
                        intakeArm.set(ARM_DOWN_SPEED);
                    else
                        intakeArm.set(ARM_HOLD_POWER);
                }
                break;
            }
      
    
        //run talon 10 until no buttons are pressed or a limit switch(DIO4)is pressed
        //two buttons are used RR3 and RL3, RR3 moves it forward, RL3 moves it backward

        //4063counts per revolution/18.5 gear down = 1 revolution
        //208.79 counts = 1 degree
    

        // OUTPUTS TO SHUFFLEBOARD/SMARTDASHBOARD, USE FOR TRACKING VALUES
        // SmartDashboard.putNumber("JoyStickXVal", right.getX());
        // SmartDashboard.putNumber("JoyStickYVal", right.getY());
        //SmartDashboard.putNumber("Left Servo", rDriveServo.getAngle());
        //SmartDashboard.putNumber("Right Servo", lDriveServo.getAngle());
        // SmartDashboard.putNumber("Slide", crossSlide.get());
        // SmartDashboard.putBoolean("left", lSlideSensor.get());
        // SmartDashboard.putBoolean("right", rSlideSensor.get());
        // SmartDashboard.putNumber("target", slideTarget);
        // SmartDashboard.putNumber("color total", colorTotal);
        // SmartDashboard.putNumber("Arm Encoder", encoderCount);
        //SmartDashboard.putBoolean("Proximity Sensor", cSlideSensor.get());

        colorSensor.read();
        //SmartDashboard.putNumber("test", colorSensor.status());
        // SmartDashboard.putNumber("red", colorSensor.red);
        // SmartDashboard.putNumber("green", colorSensor.green);
        // SmartDashboard.putNumber("blue", colorSensor.blue);
        // SmartDashboard.putNumber("prox", colorSensor.prox);
  }
  
  /**********************************************************
                      User Functions
  ***********************************************************/
  boolean checkColor(double threshold){ //returns if white tape if found, recommended threshold is 10
    if(colorSensor.red > threshold && colorSensor.green > threshold && colorSensor.blue > threshold)
      return true;   
    else
      return false;
  }

}