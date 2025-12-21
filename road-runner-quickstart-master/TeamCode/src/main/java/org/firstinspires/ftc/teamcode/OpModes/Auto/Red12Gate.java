package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import org.firstinspires.ftc.teamcode.Robot.BaseRobot14471;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Red12Gate")
public class Red12Gate extends LinearOpMode {

    double ON = -1;
    double OFF = 0;
    double IN = 0.15;
    double NEARVEL = 1110; //(tune)

    //PID
    Pose2d startPose = new Pose2d(52, 48, Math.toRadians(-55));
    BaseRobot14471 robot = new BaseRobot14471(true, startPose);
    ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- Setup
        robot.init(hardwareMap);
        robot.initPIDF();


        waitForStart();

        if (isStopRequested()) return;
        robot.leftShooter.setVelocity(-NEARVEL);
        robot.rightShooter.setVelocity(NEARVEL);



        Vector2d shootPos = new Vector2d(82, 12);

        Actions.runBlocking(
                robot.drive.actionBuilder(startPose)

                        // Shoot #1
                        .strafeToLinearHeading(shootPos, Math.toRadians(-53), new TranslationalVelConstraint(60))

                        .build()
        );
        robot.shoot();

        //------------------------------intake POSITION #1----------------
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(82, 12, Math.toRadians(-47)))
                        .strafeToLinearHeading(new Vector2d(77, 22), Math.toRadians(86), new TranslationalVelConstraint(70))
                        .strafeToLinearHeading(new Vector2d(77, 42), Math.toRadians(86), new TranslationalVelConstraint(70))

                        //-------------GATE----------
                        .strafeToLinearHeading(new Vector2d(87, 36), Math.toRadians(0), new TranslationalVelConstraint(70))
                         .strafeToLinearHeading(new Vector2d(87, 60), Math.toRadians(0), new TranslationalVelConstraint(70))

                        .build()
        );
        robot.intake.setPower(-0.1);
        // ------------shoot #2-------------
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(77, 42, Math.toRadians(86)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(-46), new TranslationalVelConstraint(70))
                        .build()
        );

        robot.shoot();
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(82, 12, Math.toRadians(-46)))
                        .strafeToLinearHeading(new Vector2d(98, 23), Math.toRadians(91), new TranslationalVelConstraint(70))
                        .strafeToLinearHeading(new Vector2d(98, 47), Math.toRadians(91), new TranslationalVelConstraint(60))
                        //.waitSeconds(0.2)


                        .build()
        );
        robot.intake.setPower(-0.1);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(98, 47, Math.toRadians(91)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(-46), new TranslationalVelConstraint(60))
                        .build()
        );


        robot.shoot();

        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(82, 12, Math.toRadians(-46)))
                        .strafeToLinearHeading(new Vector2d(120, 17), Math.toRadians(90), new TranslationalVelConstraint(60))
                        .strafeToLinearHeading(new Vector2d(120, 46), Math.toRadians(90), new TranslationalVelConstraint(60))
                        .build()
        );
        //shoot 4
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(120, 46, Math.toRadians(90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(-46), new TranslationalVelConstraint(60))
                        .build()
        );
        robot.leftShooter.setVelocity(-NEARVEL-25);
        robot.rightShooter.setVelocity(NEARVEL+25);
        robot.shoot();
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(82, 12, Math.toRadians(-46)))
                        .strafeToLinearHeading(new Vector2d(82, 25), Math.toRadians(90), new TranslationalVelConstraint(70))
                        .build()
        );





    }
 //   private void shoot() {

  //      robot.intake.setPower(ON);

    ///    robot.leftIndex.setPosition(0);
    //    robot.rightIndex.setPosition(1);
   //     robot.belt.setPower(-1);
    //    delay(0.85); // wait 1 seconds for shooting

        // Stop all

        //spinning against
  //      robot.leftIndex.setPosition(0.6);
 //       robot.rightIndex.setPosition(0.4);
        // belt.setPower(-0.3);

  //  }
    // -------- Delay helper ----------
    private void delay(double t) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            // Optionally update telemetry

        }
    }
}
