package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.Robot.BaseRobot14471;

import java.util.List;

@TeleOp(name="TestPIDF", group="Training")
public class TestPIDF extends OpMode {

    private double P =5;
    private double I = 0;
    private double D = 0;
    private double F = 12;

    BaseRobot14471 robot = new BaseRobot14471(false);  // TeleOp robot

    private double SPEED_CONTROL = 1;
    private boolean comboUpPrev = false;
    private boolean comboDownPrev = false;


    private boolean shooterReadyPrev = false;




    private boolean bPrev = false; // class-level variable
    private boolean shooting;
    private double targetVel = 0;


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.testPIDF(P, I , D, F);  // use PIDF, not PID

        // Set drive motors to run without encoders
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Shooter motors run using encoders
        robot.leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Robot Ready");
        telemetry.update();
    }

    @Override
    public void loop() {



            // Manual field-centric control
            double leftY = gamepad1.left_stick_y * SPEED_CONTROL;
            double leftX = gamepad1.left_stick_x * SPEED_CONTROL;
            double rightX = gamepad1.right_stick_x * SPEED_CONTROL;

            double yaw = robot.imu.getRobotYawPitchRollAngles().getYaw();
            robot.driveFieldCentric(leftX, leftY, rightX, yaw);


        double leftVel = robot.leftShooter.getVelocity();
        double rightVel = robot.rightShooter.getVelocity();
        if (gamepad1.left_bumper) {
            targetVel = 1150;
            robot.spinUpShooter(targetVel);
        }
        else if (gamepad1.right_bumper) {
            targetVel = 1350;
            robot.spinUpShooter(targetVel);
        }
        double intakePower = -(gamepad1.right_trigger-gamepad1.left_trigger);
        //added limits to intake, belt powers
        robot.belt.setPower(intakePower *0.85);
        robot.intake.setPower(intakePower * 0.85);

// ----------- Toggle shooting -----------
        boolean bNow = gamepad1.b;
        boolean shooterReady = targetVel > 0 &&
                rightVel  >= targetVel - 50 &&
                leftVel >= targetVel - 50 && Math.abs(intakePower)< 0.05;

        if (shooterReady && !shooterReadyPrev) {
            gamepad1.rumble(1000); // 200 ms short buzz
        }

        shooterReadyPrev = shooterReady;

        if (shooterReady && bNow && !bPrev) {
            shooting = true;

        }
        //intake w/ bumpers


// Stop shooting if x or triggers  being used
        if ( gamepad1.x || Math.abs(intakePower)>0.05) {
            shooting = false;
        }

        bPrev = bNow;



// ----------- Execute state -----------

        if (shooting) {

            robot.shoot();
        }
        else if (intakePower > 0.05) {
            robot.reverseEverything();

        }
        else if (intakePower < -0.05) {
            robot.reverseIndexers();
        }


        // ---------------- PIDF live tuning with D-pad ----------------
        boolean comboUpNow = gamepad2.dpad_up;
        boolean comboDownNow = gamepad2.dpad_down;

        boolean pButtonNow = gamepad2.y;
        boolean iButtonNow = gamepad2.x;
        boolean dButtonNow = gamepad2.a;
        boolean fButtonNow = gamepad2.b;

// Edge detection: only trigger on first press
        if (comboUpNow && !comboUpPrev) {
            if (pButtonNow) {
                P += 5;
                robot.testPIDF(P, I, D, F);
            }
            if (iButtonNow) {
                I += 0.3;
                robot.testPIDF(P, I, D, F);
            }
            if (dButtonNow) {
                D += 0.3;
                robot.testPIDF(P, I, D, F);
            }
            if (fButtonNow) {
                F += 0.1;
                robot.testPIDF(P, I, D, F);
            }
        } else if (comboDownNow && !comboDownPrev) {
            if (pButtonNow) {
                P -= 5;
                robot.testPIDF(P, I, D, F);
            }
            if (iButtonNow) {
                I -= 0.3;
                robot.testPIDF(P, I, D, F);
            }
            if (dButtonNow) {
                D -= 0.3;
                robot.testPIDF(P, I, D, F);
            }
            if (fButtonNow) {
                F -= 0.1;
                robot.testPIDF(P, I, D, F);
            }
        }

// Save previous states for next loop
        comboUpPrev = comboUpNow;
        comboDownPrev = comboDownNow;



        // Resety yaw
        if (gamepad1.y) robot.imu.resetYaw();

        // Telemetry
        telemetry.addData("intakePower", intakePower);
        telemetry.addData("left shooter Velocity", leftVel);
        telemetry.addData("right shooter Velocity", rightVel);
        telemetry.addData("target velocity", targetVel);
        telemetry.addData("Current P: (gamepad2.Y) ", P);
        telemetry.addData("Current I: (gamepad2.X) ", I);
        telemetry.addData("Current D: (gamepad2.A) ", D);
        telemetry.addData("Current F: (gamepad2.B) ", F);

        telemetry.update();
    }


}
