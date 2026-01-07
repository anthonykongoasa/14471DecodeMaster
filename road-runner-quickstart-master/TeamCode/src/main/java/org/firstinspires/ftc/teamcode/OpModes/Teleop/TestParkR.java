package org.firstinspires.ftc.teamcode.OpModes.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot.BaseRobot14471;

@TeleOp(name = "TestRedPark", group = "Training")
public class TestParkR extends OpMode {

    //use auto constructor (for now)
    BaseRobot14471 robot = new BaseRobot14471(
            true,
            new Pose2d(0, 0, 0)
    );

    private Action parkAction = null;

    @Override
    public void init() {
        robot.init(hardwareMap);

        // Drive motors (manual control)
        robot.drive.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.drive.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        /* ================= MANUAL DRIVE ================= */
        double y = gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double yaw = robot.drive.lazyImu.get()
                .getRobotYawPitchRollAngles()
                .getYaw();

        robot.drive.driveFieldCentric(x, y, rx, yaw);

        /* ================= RR POSE UPDATE ================= */
        robot.drive.updatePoseEstimate();

        /* ================= SNAP POSE ================= */
        if (gamepad1.dpad_up) {
            robot.drive.localizer.setPose(
                    new Pose2d(67, -75, Math.toRadians(90))
            );
        }

        /* ================= START AUTO PARK ================= */
        //need to tune
        if (gamepad1.dpad_down && parkAction == null) {
            parkAction = robot.drive
                    .actionBuilder(robot.drive.localizer.getPose())
                    .strafeToLinearHeading(
                            new Vector2d(50, -35),
                            Math.toRadians(90)
                    )
                    .build();
        }

        /* ================= RUN AUTO PARK ================= */
        if (parkAction != null) {
            TelemetryPacket packet = new TelemetryPacket();
            boolean running = parkAction.run(packet);

            if (!running) {
                parkAction = null; // finished
            }
        }

        /* ================= CANCEL AUTO IF DRIVER MOVES ================= */
        if (Math.abs(x) > 0.05 || Math.abs(y) > 0.05 || Math.abs(rx) > 0.05) {
            parkAction = null;
        }
        if (gamepad1.y) {
            robot.drive.lazyImu.get().resetYaw();
        }

        /* ================= TELEMETRY ================= */
        Pose2d pose = robot.drive.localizer.getPose();
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.addData("Auto Park", parkAction != null);
        telemetry.update();
    }
}