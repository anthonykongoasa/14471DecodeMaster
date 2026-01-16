
package org.firstinspires.ftc.teamcode.OpModes.Auto;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot.BaseRobot14471;

@Autonomous(name = "BlueFar")
public class BlueFar extends LinearOpMode {

    Pose2d startPose = new Pose2d(61, -18, Math.toRadians(180));
    //AUTO robot constructor
    BaseRobot14471 robot = new BaseRobot14471(true, startPose);
    ElapsedTime runtime = new ElapsedTime();


    double ON = -1;
    double OFF = 0;
    double IN = 0.15;
    double NEARVEL = 1110; //(tune)
    double FARVEL = 1350;

    @Override
    public void runOpMode() throws InterruptedException {

        //efficencize code !
        robot.init(hardwareMap);
        robot.testPIDF(120, 0.8, 0, 12);

        waitForStart();

        if (isStopRequested()) return;

        robot.leftShooter.setVelocity(-FARVEL);
        robot.rightShooter.setVelocity(FARVEL);

        Vector2d shootPos = new Vector2d(66, -16);
        Pose2d shootPose2d  = new Pose2d(66, -16, Math.toRadians(195);

        
        Actions.runBlocking(
                robot.drive.actionBuilder(startPose)

                        // SHOOT #1
                        .strafeToLinearHeading(shootPos, Math.toRadians(195), new TranslationalVelConstraint(60))
                        .build()
        );
        delay(0.2); // wait for shooter to get warmed up
        robot.shoot();
        delay(0.85);
        robot.stopShootingAuto();

        //intake 2 (motif)
        Actions.runBlocking(
                robot.drive.actionBuilder(shootPose2d)
                        .strafeToLinearHeading(new Vector2d(80, 50), Math.toRadians(90), new TranslationalVelConstraint(70))
                        .strafeToLinearHeading(new Vector2d(80, 30), Math.toRadians(90), new TranslationalVelConstraint(60))

                        .build()
        );
        // shoot 3
        robot.intake.setPower(-0.1);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(80, 30, Math.toRadians(90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(195))
                        .build()
        )

        robot.shoot();
        delay(0.85);
        robot.stopShootingAuto();
        //------------------------------intake POSITION #2---------------- (HP ZONE)
        Actions.runBlocking(
                robot.drive.actionBuilder(shootPose2d)
                        //annoying sharp turn
                        .strafeToLinearHeading(new Vector2d (55, 35), Math.toRadians(90), new TranslationalVelConstraint(60) )
                        .strafeToLinearHeading(new Vector2d (50, 25), Math.toRadians(90), new TranslationalVelConstraint(70))
                        .strafeToLinearHeading(new Vector2d (48, 35), Math.toRadians(90), new TranslationalVelConstraint(70))


                        .build()
        );
        robot.intake.setPower(-0.1);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(48, 35, Math.toRadians(90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(195))
                        .build()
        )

        // ------------shoot #2-------------


        robot.shoot();
        delay(0.85);
        robot.stopShootingAuto();
        
        //intake 3 (HP ZONE AGAIN)


        Actions.runBlocking(
                robot.drive.actionBuilder(shootPose2d))
                        .strafeToLinearHeading(new Vector2d(60, 35), Math.toRadians(90), new TranslationalVelConstraint(70))
                        .strafeToLinearHeading(new Vector2d(63, 25), Math.toRadians(90), new TranslationalVelConstraint(60))
                        .strafeToLinearHeading(new Vector2d(60, 30), Math.toRadians(90), new TranslationalVelConstraint(70))


                        .build()
        );
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(60, 35, Math.toRadians(90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(195))
                        .build()
        )
        robot.shoot();
        delay(0.85);
        robot.stopShooting(); // turn off shotoer motors


        Actions.runBlocking(
                robot.drive.actionBuilder(shootPose2d))
                        .strafeToLinearHeading(new Vector2d(60, 35), Math.toRadians(90), new TranslationalVelConstraint(70))

                        .build()
        );

    }
    // private void shoot() {

    //  robot.intake.setPower(ON);

    //robot.leftIndex.setPosition(0);
    //robot.rightIndex.setPosition(1);
    //robot.belt.setPower(-1);
    // delay(0.85); // wait 1 seconds for shooting

    // Stop all

    //spinning against
    //  robot.leftIndex.setPosition(0.6);
    //   robot.rightIndex.setPosition(0.4);
    // belt.setPower(-0.3);

    // }


    // -------- Delay helper ----------
    private void delay(double t) {
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < t)) {
            // Optionally update telemetry
        }
    }
}
