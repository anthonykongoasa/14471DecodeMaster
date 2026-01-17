
package org.firstinspires.ftc.teamcode.OpModes.Auto;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Robot.BaseRobot14471;

@Autonomous(name = "RedFar")
public class RedFar extends LinearOpMode {

    Pose2d startPose = new Pose2d(61, -16, Math.toRadians(-180));
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

        Vector2d shootPos = new Vector2d(64, -16);
        Pose2d shootPose2d  = new Pose2d(64, -16, Math.toRadians(-200));

        
        Actions.runBlocking(
                robot.drive.actionBuilder(startPose)

                        // SHOOT #1
                        .strafeToLinearHeading(shootPos, Math.toRadians(-200), new TranslationalVelConstraint(60))
                        .build()
        );
        delay(0.2); // wait for shooter to get warmed up
        robot.shoot();
        delay(0.85);
        robot.stopShootingAuto();

        //intake 1 (motif)
        Actions.runBlocking(
                robot.drive.actionBuilder(shootPose2d)
                        .strafeToLinearHeading(new Vector2d(95, -25), Math.toRadians(-90), new TranslationalVelConstraint(70))
                        .strafeToLinearHeading(new Vector2d(95, -45), Math.toRadians(-90), new TranslationalVelConstraint(70))

                        .build()
        );
        // shoot 3
        robot.intake.setPower(-0.1);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(95, -45, Math.toRadians(-90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(-200))
                        .build()
        );

        robot.shoot();
        delay(0.85);
        robot.stopShootingAuto();
        //------------------------------intake POSITION #2---------------- (HP ZONE)
        Actions.runBlocking(
                robot.drive.actionBuilder(shootPose2d)
                        
                        .strafeToLinearHeading(new Vector2d (67, -45), Math.toRadians(-90), new TranslationalVelConstraint(60) )
                        .strafeToLinearHeading(new Vector2d (65, -37), Math.toRadians(-90), new TranslationalVelConstraint(70))
                        .strafeToLinearHeading(new Vector2d (69, -45), Math.toRadians(-90), new TranslationalVelConstraint(70))


                        .build()
        );
        robot.intake.setPower(-0.1);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(67, -45, Math.toRadians(-90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(-200))
                        .build()
        );

        // ------------shoot #2-------------


        robot.shoot();
        delay(0.85);
        robot.stopShootingAuto();
        
        //intake 3 (HP ZONE X2)

        Actions.runBlocking(
                robot.drive.actionBuilder(shootPose2d)
                        .strafeToLinearHeading(new Vector2d(67, -45), Math.toRadians(-90), new TranslationalVelConstraint(70))
                        .strafeToLinearHeading(new Vector2d(65, -37), Math.toRadians(-90), new TranslationalVelConstraint(60))
                        .strafeToLinearHeading(new Vector2d(69, -45), Math.toRadians(-90), new TranslationalVelConstraint(70))


                        .build()
        );
        //shoot #4
        robot.intake.setPower(-0.1);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(67, -45, Math.toRadians(-90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(-200))
                        .build()
        );
        robot.shoot();
        delay(0.85);
        robot.stopShooting(); // turn off shooter motors


        Actions.runBlocking( // off line, setup for tele
                robot.drive.actionBuilder(shootPose2d)
                        .strafeToLinearHeading(new Vector2d(70, -15), Math.toRadians(90), new TranslationalVelConstraint(70))

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
