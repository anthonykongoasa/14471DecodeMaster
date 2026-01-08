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

@TeleOp(name="Teleop", group="Training")
public class MainTeleop extends OpMode {

    BaseRobot14471 robot = new BaseRobot14471(false);  // TeleOp robot

    private double SPEED_CONTROL = 1;

    // Camera & vision
    private boolean shooterReadyPrev = false;
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;



    private int tagID = -1;
    private double range = -1;


    private boolean bPrev = false; // class-level variable
    private boolean shooting;
    private double targetVel = 0;


    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initPIDF();  // use PIDF, not PID

        // Set drive motors to run without encoders
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Shooter motors run using encoders
        robot.leftShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        initAprilTag();

        telemetry.addData("Status", "Robot Ready");
        telemetry.update();
    }

    @Override
    public void loop() {


        // Read AprilTag detections
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        desiredTag = null;
        tagID = -1;
        range = -1;

        for (AprilTagDetection detection : currentDetections) {
            if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                desiredTag = detection;
                tagID = desiredTag.id;
                range = desiredTag.ftcPose.range;
                break;
            }
        }

        // ----------- Drive logic ----------- NEED TO FIX
        if (gamepad1.dpad_left && desiredTag != null) {
            // Auto-drive to tag
            double rangeError = desiredTag.ftcPose.range - 75; // target distance
            double headingError = desiredTag.ftcPose.bearing - 0; // target heading
            double yawError = desiredTag.ftcPose.yaw;

            // Gains for proportional control
            double SPEED_GAIN = 0.02;
            double TURN_GAIN = 0.01;
            double STRAFE_GAIN = 0.015;

            // Compute motor powers
            double forward = Range.clip(rangeError * SPEED_GAIN, -0.5, 0.5);
            double strafe = Range.clip(-yawError * STRAFE_GAIN, -0.5, 0.5);
            double turn = Range.clip(-headingError * TURN_GAIN, -0.3, 0.3);

            robot.driveFieldCentric(strafe, forward, turn, robot.imu.getRobotYawPitchRollAngles().getYaw());

        } else {
            // Manual field-centric control
            double leftY = gamepad1.left_stick_y * SPEED_CONTROL;
            double leftX = gamepad1.left_stick_x * SPEED_CONTROL;
            double rightX = gamepad1.right_stick_x * SPEED_CONTROL;

            double yaw = robot.imu.getRobotYawPitchRollAngles().getYaw();
            robot.driveFieldCentric(leftX, leftY, rightX, yaw);
        }

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
                Math.abs(leftVel) >= targetVel - 50 && Math.abs(intakePower)< 0.05;

        if (shooterReady && !shooterReadyPrev) {
            gamepad1.rumble(1000); // 200 ms short buzz
        }

        shooterReadyPrev = shooterReady;

        if (shooterReady && bNow && !bPrev) {
            shooting = true;

        }
        //intake w/ bumpers    


// Stop shooting if x or triggers  being used
        if (gamepad1.x || Math.abs(intakePower)>0.05) {
            shooting = false;
        }

        bPrev = bNow;

        

// ----------- Execute state -----------
        // may need to swap > / < logic
        if (shooting) {

            robot.shoot();
        }
        else if (intakePower > 0.05) {
            robot.reverseIndexers();
            
        }
        else if (intakePower < -0.05) {
            robot.reverseEverything();
        }
        else {
            if (!gamepad1.left_bumper && !gamepad1.right_bumper) { //second safeguard
            robot.stopShooting();
            robot.reverseIndexers();
            targetVel = 0;
            }
        }

        

        // Resety yaw
        if (gamepad1.y) robot.imu.resetYaw();

        // Telemetry
        telemetry.addData("intakePower", intakePower);
        telemetry.addData("left shooter Velocity", leftVel);
        telemetry.addData("right shooter Velocity", rightVel);
        telemetry.addData("target velocity", targetVel);
        telemetry.addData("Tag ID", tagID);
        telemetry.addData("Range", range);
        telemetry.update();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }

        aprilTag.setDecimation(2);
    }
}
