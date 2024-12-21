package org.firstinspires.ftc.teamcode.autonomous;
//6.92307692308 is 1 inch
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@Autonomous(group = "drive")
public class BUCKET extends LinearOpMode {
    public static double DISTANCE = 24; // in

    public Slide slide;
    public Motors panningMotor;
    public Servos claw;
    public Servos panningServo;
    public Servos orientation;
    public Motors pulley;
    public int runFrames;

    public HuskyLens.Block currentTarget;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        slide = new Slide(hardwareMap, telemetry, "slide");
        panningMotor = new Motors(hardwareMap, "panningmotor");
        claw = new Servos(hardwareMap, "claw");
        panningServo = new Servos(hardwareMap, "panning");
        orientation = new Servos(hardwareMap, "orientation");
        pulley = new Motors(hardwareMap, "pulley");
//        frontLens = new HuskyLenses(hardwareMap, "frontlens", "color");

        runFrames = 0;
        claw.moveForwardMAX();
        panningServo.moveBackwardMIN();
        orientation.moveBackwardMIN();
        slide.MoveToLevel(Slide.level.zero);

        PathBuilder builder = new PathBuilder();

        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(0.443, 64.689, Point.CARTESIAN),
                                new Point(38.326, 64.468, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(38.326, 64.468, Point.CARTESIAN),
                                new Point(0.443, 38.769, Point.CARTESIAN),
                                new Point(56.271, 55.385, Point.CARTESIAN),
                                new Point(61.145, 23.926, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(61.145, 23.926, Point.CARTESIAN),
                                new Point(9.969, 23.040, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addPath(
                        // Line 4
                        new BezierCurve(
                                new Point(9.969, 23.040, Point.CARTESIAN),
                                new Point(64.025, 51.840, Point.CARTESIAN),
                                new Point(64.468, 13.735, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(180))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(64.468, 13.735, Point.CARTESIAN),
                                new Point(9.526, 13.514, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 6
                        new BezierCurve(
                                new Point(9.526, 13.514, Point.CARTESIAN),
                                new Point(11.742, 65.797, Point.CARTESIAN),
                                new Point(38.769, 68.677, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        // Line 7
                        new BezierLine(
                                new Point(38.769, 68.677, Point.CARTESIAN),
                                new Point(9.526, 28.135, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180));

        waitForStart();

        if (isStopRequested()) return;

        //lineto and drop first specimen
//        drive.followTrajectory(dropfirst);
        panningMotor.rotateForward(0.8, 300);
        panningServo.moveForwardMAX();
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        //drive.turn(Math.toRadians(-30));
        sleep(500);
        panningServo.moveSpecificPos(.35);
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.zero);
        panningServo.moveForwardMAX();
        sleep(1000);
        panningMotor.rotateForward(-0.8, 500);
        //drive.turn(Math.toRadians(30));
        slide.MoveToLevel(Slide.level.pan_highbar);
        sleep(500);
        claw.moveForwardMAX();
        sleep(700);
        panningServo.moveSpecificPos(.35);
        slide.MoveToLevelAsync(Slide.level.zero);
        sleep(1000);
        panningMotor.rotateForward(1, 300);
        panningServo.moveSpecificPos(.35);
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        sleep(500);
       // drive.turn(Math.toRadians(-40));
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.zero);
        panningServo.moveForwardMAX();
        sleep(1000);
        panningMotor.rotateForward(-0.8, 500);
        //drive.turn(Math.toRadians(40));
        //drive.followTrajectory(pickupA);
        slide.MoveToLevel(Slide.level.pan_highbar);
        sleep(500);
        claw.moveForwardMAX();
        sleep(700);
        panningServo.moveSpecificPos(.35);
        slide.MoveToLevelAsync(Slide.level.zero);
        //drive.followTrajectory(pickupB);
        panningServo.moveForwardMAX();
       // drive.turn(Math.toRadians(-40));
        panningMotor.rotateForward(1, 300);
        slide.MoveToLevel(Slide.level.slide_highbucket);
        panningServo.moveSpecificPos(.35);
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.zero);
        panningServo.moveForwardMAX();
        sleep(1000);
        panningMotor.rotateForward(-0.8, 500);
        //drive.turn(Math.toRadians(65));
        claw.moveSpecificPos(0.5);
        slide.MoveToLevel(Slide.level.panigging);
        sleep(500);
        claw.moveForwardMAX();
        sleep(700);
        slide.MoveToLevelAsync(Slide.level.zero);
        panningServo.moveSpecificPos(.35);
        sleep(1000);
        panningMotor.rotateForward(1, 300);
        panningServo.moveSpecificPos(.35);
        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
        //drive.turn(Math.toRadians(-60));
        sleep(500);
        claw.moveBackwardMIN();
        sleep(300);
        slide.MoveToLevelAsync(Slide.level.zero);
        panningServo.moveForwardMAX();
        sleep(1000);
    }
}