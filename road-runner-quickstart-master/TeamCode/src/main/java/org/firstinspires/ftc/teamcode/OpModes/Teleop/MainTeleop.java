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

@TeleOp(name="Main_Teleop_Code", group="Training")
public class MainTeleop extends OpMode {

    BaseRobot14471 robot = new BaseRobot14471(false);  // TeleOp robot
    private double SHOOTER_VELOCITY = 0;
    private double SPEED_CONTROL = 1;

    // Camera & vision
    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = -1;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;


    private int tagID = -1;
    private double range = -1;
    boolean bPrev = false; // class-level variable

    @Override
    public void init() {
        robot.init(hardwareMap);
        robot.initPIDF();  // use PIDF, not PID

        // Set drive motors to run without encoders
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

        // ----------- Drive logic -----------
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
            double leftY = -gamepad1.left_stick_y * SPEED_CONTROL;
            double leftX = gamepad1.left_stick_x * SPEED_CONTROL;
            double rightX = gamepad1.right_stick_x * SPEED_CONTROL;

            double yaw = robot.imu.getRobotYawPitchRollAngles().getYaw();
            robot.driveFieldCentric(leftX, leftY, rightX, yaw);
        }

        // ----------- Shooter (save voltage :(  ) -----------
        if (gamepad1.left_bumper) SHOOTER_VELOCITY = 1150;
        else if (gamepad1.right_bumper) SHOOTER_VELOCITY = 1300;


        robot.leftShooter.setVelocity(SHOOTER_VELOCITY);
        robot.rightShooter.setVelocity(-SHOOTER_VELOCITY);

        // ----------- Intake / feeder -----------

        //shooter
        boolean bNow = gamepad1.b;
        if (bNow && !bPrev && robot.leftShooter.getVelocity() >= 1100) {
            //only then u can shoot
            robot.shoot();
        }
        else if (gamepad1.a) {
            //itnake
            robot.reverseIndexers();
            robot.intake.setPower(-1);
            robot.belt.setPower(-1);

        }
        else if (gamepad1.x) {
            robot.reverseEverything();
        }
        else {
              if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                  robot.stopShooting();
                  robot.reverseIndexers();
                  SHOOTER_VELOCITY = 0;
              }
        }
        bPrev = bNow;

        // Reset IMU yaw
        if (gamepad1.y) robot.imu.resetYaw();

        // Telemetry
        telemetry.addData("Shooter Velocity", SHOOTER_VELOCITY);
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
