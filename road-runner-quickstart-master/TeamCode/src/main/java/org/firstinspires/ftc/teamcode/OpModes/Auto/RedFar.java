
package org.firstinspires.ftc.teamcode.OpModes.Auto;

import java.util.Arrays;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;

//import com.acmerobotics.roadrunner.TranslationalAccelConstraint;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
//import com.acmerobotics.roadrunner.AngularAccelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot.BaseRobot14471;

@Autonomous(name = "RedFar")
public class RedFar extends LinearOpMode {

    MinVelConstraint velConstraint = new MinVelConstraint(Arrays.asList(
            new TranslationalVelConstraint(30),
            new AngularVelConstraint(2)
    ));

    Pose2d startPose = new Pose2d(62, 16, Math.toRadians(0));
    //AUTO robot constructor
    BaseRobot14471 robot = new BaseRobot14471(true, startPose);
    ElapsedTime runtime = new ElapsedTime();

    double FARVEL = 1300;

    @Override
    public void runOpMode() throws InterruptedException {

        //efficencize code !
        robot.init(hardwareMap);
        robot.testPIDF(120, 0.8, 0, 12);

        waitForStart();

        if (isStopRequested()) return;

        robot.leftShooter.setVelocity(-FARVEL);
        robot.rightShooter.setVelocity(FARVEL);

       Vector2d shootPos = new Vector2d(56, 16);
        Pose2d shootPose2d  = new Pose2d(56, 16, Math.toRadians(-20));

        
        Actions.runBlocking(
                robot.drive.actionBuilder(startPose)

                        // SHOOT #1
                        .strafeToLinearHeading(shootPos, Math.toRadians(-20))
                        .build()
        );
        delay(0.2); // wait for shooter to get warmed up
        robot.shoot();
        delay(0.85);
        robot.stopShootingAuto();

        //intake 1 (motif)
        Actions.runBlocking(
                robot.drive.actionBuilder(shootPose2d)
                        .strafeToLinearHeading(new Vector2d(35,30),Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(35,55),Math.toRadians(90))

                        .build()
        );
        // shoot 2
        robot.intake.setPower(-0.1);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(35, 55, Math.toRadians(90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(-20))
                        .build()
        );

        robot.shoot();
        delay(0.85);
        robot.stopShootingAuto();
        //------------------------------intake POSITION #2---------------- (HP ZONE)
        Actions.runBlocking(
                robot.drive.actionBuilder(shootPose2d)
                        
                        .strafeToLinearHeading(new Vector2d(57 , 16), Math.toRadians(90), velConstraint)
                        .lineToX(60, new TranslationalVelConstraint(30))
                        .waitSeconds(0.1)
          
                        .lineToY(60, new TranslationalVelConstraint(70)) // big move
                        .lineToY(50 , new TranslationalVelConstraint(30))
                        .lineToY(60,  new TranslationalVelConstraint(30))

                        .build()
        );
        robot.intake.setPower(-0.1);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(60, 60, Math.toRadians(90)))
          
                       .strafeToLinearHeading(new Vector2d(50, 16), Math.toRadians(-20), new TranslationalVelConstraint(70))
                        .strafeToLinearHeading(shootPos, Math.toRadians(-20), new TranslationalVelConstraint(30))
                        .build()
        );

        // ------------shoot #2-------------


        robot.shoot();
        delay(0.85);
        robot.stopShootingAuto();
        
        //intake 3 (HP ZONE X2)

        Actions.runBlocking(
                robot.drive.actionBuilder(shootPose2d)
                        .strafeToLinearHeading(new Vector2d(57, 16), Math.toRadians(90), velConstraint)
                        .lineToX(60, new TranslationalVelConstraint(30))
                        .waitSeconds(0.1)
          
                        .lineToY(60, new TranslationalVelConstraint(70)) // big move
                        .lineToY(50 , new TranslationalVelConstraint(30))
                        .lineToY(60,  new TranslationalVelConstraint(30))
                        .build()
        );
        //return + shoot #4
        robot.intake.setPower(-0.1);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(60, 60, Math.toRadians(90)))
                        .strafeToLinearHeading(new Vector2d(50, 16), Math.toRadians(20), new TranslationalVelConstraint(70))
                        .strafeToLinearHeading(shootPos, Math.toRadians(-20), new TranslationalVelConstraint(30))
                        .build()
        );
        robot.shoot();
        delay(0.85);
        robot.stopShooting(); // turn off shooter motors


        Actions.runBlocking( // off line, setup for tele
                robot.drive.actionBuilder(shootPose2d)

                         .strafeToLinearHeading(new Vector2d(45,16),Math.toRadians(90))

                        .build()
        );

    }


    // -------- Delay helper ----------
    private void delay(double t) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            // Optionally update telemetry
        }
    }
}
