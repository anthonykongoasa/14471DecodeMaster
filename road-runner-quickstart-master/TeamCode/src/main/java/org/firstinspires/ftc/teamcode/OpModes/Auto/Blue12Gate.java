
package org.firstinspires.ftc.teamcode.OpModes.Auto;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue12Gate")
public class Blue12Gate extends LinearOpMode {

    Pose2d startPose = new Pose2d(-52, -48, Math.toRadians(55));
    //AUTO robot constructor
    BaseRobot14471 robot = new BaseRobot14471(true, startPose);
    ElapsedTime runtime = new ElapsedTime();


    double ON = -1;
    double OFF = 0;
    double IN = 0.15;
    double NEARVEL = 1110; //(tune)
    double FARVEL = 1500;

    @Override
    public void runOpMode() throws InterruptedException {

        //efficencize code !
        robot.init(hardwareMap);
        robot.initPIDF();

        waitForStart();

        if (isStopRequested()) return;

        robot.leftShooter.setVelocity(-NEARVEL);
        robot.rightShooter.setVelocity(NEARVEL);

        Vector2d shootPos = new Vector2d(-20, -16);


        Actions.runBlocking(
                robot.drive.actionBuilder(startPose)

                        // Shoot #1
                        .strafeToLinearHeading(shootPos, Math.toRadians(53), new TranslationalVelConstraint(100))
                        .build()
        );
        shoot();

        //------------------------------intake POSITION #1----------------
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(47)))
                        //annoying sharp turn
                        .strafeToLinearHeading(new Vector2d(6, -26), Math.toRadians(-88), new TranslationalVelConstraint(100))
                        .strafeToLinearHeading(new Vector2d(6, -52), Math.toRadians(-88), new TranslationalVelConstraint(100))
                        .waitSeconds(0.3)
                        .strafeToLinearHeading(new Vector2d(8, -50), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(8, -75), Math.toRadians(180), new TranslationalVelConstraint(100))
                        .build()
        );
        robot.intake.setPower(-0.1);
        // ------------shoot #2-------------
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(6, -52, Math.toRadians(-88)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(50), new TranslationalVelConstraint(100))
                        .build()
        );

        shoot();
        //intake 2
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(50)))
                        .strafeToLinearHeading(new Vector2d(36, -20), Math.toRadians(-90), new TranslationalVelConstraint(100))
                        .strafeToLinearHeading(new Vector2d(36, -60), Math.toRadians(-90), new TranslationalVelConstraint(100))
                        .build()
        );
        // shoot 3
        robot.intake.setPower(-0.1);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(36, -60, Math.toRadians(-90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(50), new TranslationalVelConstraint(100))
                        .build()
        );

        shoot();
        //intake 3
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(60)))
                        .strafeToLinearHeading(new Vector2d(54, -20), Math.toRadians(-90), new TranslationalVelConstraint(100))
                        .strafeToLinearHeading(new Vector2d(54, -60), Math.toRadians(-90), new TranslationalVelConstraint(100))
                        .build()
        );

        robot.intake.setPower(-0.3);
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(54, -60, Math.toRadians(-90)))
                        .strafeToLinearHeading(shootPos, Math.toRadians(55), new TranslationalVelConstraint(100))
                        .build()
        );
        robot.leftShooter.setVelocity(-NEARVEL-50);
        robot.rightShooter.setVelocity(NEARVEL+50);
        shoot();

// Off-line final next to gate hoepfully
        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(-20, -16, Math.toRadians(50)))
                        .strafeToLinearHeading(new Vector2d(15, -25), Math.toRadians(-90), new TranslationalVelConstraint(100))
                        .build()
        );


    }
    private void shoot() {

        robot.intake.setPower(ON);

        robot.leftIndex.setPosition(0);
        robot.rightIndex.setPosition(1);
        robot.belt.setPower(-1);
        delay(0.85); // wait 1 seconds for shooting

        // Stop all

        //spinning against
        robot.leftIndex.setPosition(0.6);
        robot.rightIndex.setPosition(0.4);
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
