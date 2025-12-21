package org.firstinspires.ftc.teamcode.Robot;

import org.firstinspires.ftc.teamcode.Roadrunner.Drive.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public class BaseRobot14471 {

    private double INDEX_FIRE_L = 0;
    private double INDEX_FIRE_R =1;
    private double INDEX_REVERSE_L = 0.6;
    private double INDEX_REVERSE_R = 0.4;
    private Pose2d start;
    private boolean isAuto;
    public MecanumDrive drive;
    /* ACCURATELY (Self documenting) NAMED MOTORS/SERVOS */
    public DcMotor leftFront   = null;
    public DcMotor rightFront  = null;
    public DcMotor leftRear    = null;
    public DcMotor rightRear   = null;
    
    public DcMotorEx rightShooter    = null;
    public DcMotorEx leftShooter     = null;
    public Servo rightIndex    = null;
    public Servo leftIndex   = null;
    public DcMotor intake = null;
    public DcMotor belt = null;

    public IMU imu;

    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Constructor (tele)
    public BaseRobot14471(boolean auto) {
        isAuto = auto;
    }
  // overloaded constructor (auto)
   public BaseRobot14471(boolean auto, Pose2d startPose) {
      start = startPose;
      isAuto = auto;
   }

    
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        if (!isAuto) { //tele
        /**
        * I Don't like ahwMap
        * TODO figure out if its needed
        * Mecanum drive does this automatically by itself :)
        */
    
        leftFront   = hwMap.dcMotor.get("leftfront");
        rightFront  = hwMap.dcMotor.get("rightfront");
        leftRear     = hwMap.dcMotor.get("leftrear");
        rightRear    = hwMap.dcMotor.get("rightrear");
        leftFront.setDirection(DcMotor.Direction.FORWARD); 
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD); 
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
      
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

         // IMU  
        imu = hwMap.get(IMU.class, "imu");

        //TODO research inconsistency btwn predone logo facing direction and actual  
        //LEFT vs RIGHT (RIGHT should be right)
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // Initialize  IMU
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        }
        else { //auto
            drive = new MecanumDrive(hwMap, start);
            //:) easy as that
        }

        //other motors as usual
        leftShooter      = (DcMotorEx) hwMap.get(DcMotor.class, "leftarm");
        rightShooter     = (DcMotorEx) hwMap.get(DcMotor.class, "rightarm");
        intake       = hwMap.dcMotor.get("intake");
        belt       = hwMap.dcMotor.get("uptake");
        // Define and initialize ALL installed servos.
        //TODO try CR Servo controls
        leftIndex = hwMap.servo.get("lefthand");
        rightIndex = hwMap.servo.get("righthand");
    

    }
    public void initPIDF() {
        PIDFCoefficients pidSettings = new PIDFCoefficients(120, 0, 0.8, 11.5); //still need to tune F :(
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
    }

    /**
     * This method is intended for testing/tuning pidf with variable values.
     *
     * @param p
     * @param i
     * @param d
     * @param f
     */
    public void testPIDF(double p, double i, double d, double f) {
        PIDFCoefficients pidSettings = new PIDFCoefficients(p, i, d, f); //still need to tune F :(
        leftShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
        rightShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
    }

    /** 
    * field centric, only tele
    */
    public void driveFieldCentric(double gamepadXPow, double gamepadYPow, double gamepadRotPow, double robotHeading){

        double gamepadTheta = Math.atan2(gamepadYPow, gamepadXPow) - Math.PI/2;
        double velocity = Math.sqrt(Math.pow(gamepadXPow, 2) + Math.pow(gamepadYPow, 2));
        double diffTheta = gamepadTheta + Math.toRadians(robotHeading);
        double rotpow = gamepadRotPow;
        
        double v1= velocity * Math.sin(diffTheta + Math.PI/4) - rotpow; // leftfront
        double v2 = velocity * Math.cos(diffTheta + Math.PI/4) + rotpow; // rightfront
        double v3 = velocity * Math.cos(diffTheta + Math.PI/4) - rotpow; // leftrear
        double v4 = velocity * Math.sin(diffTheta + Math.PI/4) + rotpow; // rightrear
            
        leftFront.setPower(v1);
        leftRear.setPower(v3);
        rightFront.setPower(v2);
        rightRear.setPower(v4);
        
    }
    /* -------- Shooter / Intake Control -------- */

    public void spinUpShooter(double velocity) {
        leftShooter.setVelocity(velocity);
        rightShooter.setVelocity(-velocity);
    }

    public void reverseEverything() {
        leftIndex.setPosition(1);
        rightIndex.setPosition(0);
        intake.setPower(0.5);
        belt.setPower(0.5);
    }

    public void reverseIndexers() {
        leftIndex.setPosition(INDEX_REVERSE_L);
        rightIndex.setPosition(INDEX_REVERSE_R);
    }

    public void shoot() {
        intake.setPower(-1);
        belt.setPower(-1);
        leftIndex.setPosition(INDEX_FIRE_L);
        rightIndex.setPosition(INDEX_FIRE_R);
    }

    public void stopShooting() {
        intake.setPower(-0.1);
        belt.setPower(-0.5);
        leftShooter.setVelocity(0);
        rightShooter.setVelocity(0);

    }


}

    



