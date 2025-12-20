package org.firstinspires.ftc.teamcode;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Blue12GATE")
public class Blue12Gate extends LinearOpMode {

    double ON = -1;
    double OFF = 0;
    double IN = 0.15;
    double NEARVEL = 1110; //(tune)
    double FARVEL = 1500;

    //PID
    private double NEW_P = 120;
    private double NEW_I = 0;
    private double NEW_D = .8;

    DcMotorEx leftOut;
    DcMotorEx rightOut;
    DcMotor intake;
    DcMotor belt;
    Servo leftIndex;
    Servo rightIndex;
    private MecanumDrive drive;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        leftOut = (DcMotorEx)hardwareMap.get(DcMotor.class, "leftarm");
        rightOut = (DcMotorEx)hardwareMap.get(DcMotor.class, "rightarm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        belt = hardwareMap.get(DcMotor.class, "uptake");
        leftIndex = hardwareMap.get(Servo.class, "lefthand");
        rightIndex = hardwareMap.get(Servo.class, "righthand");

        //PID
        PIDFCoefficients pidSettings = new PIDFCoefficients(NEW_P, NEW_I, NEW_D, 11.5);
        leftOut.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
        rightOut.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings);
        // ---------------- Setup MecanumDrive ----------------
        Pose2d startPose = new Pose2d(-52, -48, Math.toRadians(55));
        drive = new MecanumDrive(hardwareMap, startPose);


        waitForStart();

        if (isStopRequested()) return;

        (leftOut).setVelocity(-NEARVEL);
        rightOut.setVelocity(NEARVEL);

        // ---------------- Convert MeepMeep path to RoadRunner ----------------

        Vector2d shootPos = new Vector2d(-20, -16);


        Actions.runBlocking(
                drive.actionBuilder(startPose)

                        // Shoot #1
                        .strafeToLinearHeading(shootPos, Math.toRadians(53), new TranslationalVelConstraint(100))


                        .build()
        );
        shoot();

        //------------------------------intake POSITION #1----------------
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(47)))
                        //annoying sharp turn
                        .strafeToLinearHeading(new Vector2d(6, -26), Math.toRadians(-88), new TranslationalVelConstraint(100))
                        .strafeToLinearHeading(new Vector2d(6, -52), Math.toRadians(-88), new TranslationalVelConstraint(100))

                        //-------------GATE----------
                        .strafeToLinearHeading(new Vector2d(8, -50), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(8, -75), Math.toRadians(180), new TranslationalVelConstraint(100))

                        .build()
        );
        intake.setPower(-0.1);
        // ------------shoot #2-------------
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(8, -75, Math.toRadians(180)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(50), new TranslationalVelConstraint(100))
                        .build()
        );

        shoot();
        //intake 2
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(50)))
                        .strafeToLinearHeading(new Vector2d(36, -20), Math.toRadians(-90), new TranslationalVelConstraint(100))
                        .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(-90), new TranslationalVelConstraint(100))
                        //.strafeToLinearHeading(new Vector2d(34, -50), Math.toRadians(-90), new TranslationalVelConstraint(100))



                        .build()
        );
        // shoot 3
        intake.setPower(-0.1);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(36, -60, Math.toRadians(-90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(50), new TranslationalVelConstraint(100))
                        .build()
        );


        shoot();
        //intake 3
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(60)))
                        .strafeToLinearHeading(new Vector2d(54, -20), Math.toRadians(-90), new TranslationalVelConstraint(100))
                        .strafeToLinearHeading(new Vector2d(54, -60), Math.toRadians(-90), new TranslationalVelConstraint(100))

                        .build()
        );



// Far shot zone
     //   leftOut.setVelocity(-FARVEL);
   //     rightOut.setVelocity(FARVEL);

        intake.setPower(-0.3);
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(54, -60, Math.toRadians(-90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(55), new TranslationalVelConstraint(100))

                        .build()
        );
        leftOut.setVelocity(-NEARVEL-50);
        rightOut.setVelocity(NEARVEL+50);
        shoot();

// Off-line final // next to gate hoepfully
        Actions.runBlocking(
                drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(50)))
                        .strafeToLinearHeading(new Vector2d(15, -25), Math.toRadians(-90), new TranslationalVelConstraint(100))
                        .build()
        );










    }
    private void shoot() {

        intake.setPower(ON);

        leftIndex.setPosition(0);
        rightIndex.setPosition(1);
        belt.setPower(-1);
        delay(0.85); // wait 1 seconds for shooting

        // Stop all

        //spinning against
        leftIndex.setPosition(0.6);
        rightIndex.setPosition(0.4);
        // belt.setPower(-0.3);

    }

    // -------- Delay helper ----------
    private void delay(double t) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            // Optionally update telemetry

        }
    }
}
