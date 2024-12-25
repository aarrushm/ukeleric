//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.hardware.dfrobot.HuskyLens;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
//import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
//
//@Config
//@Autonomous(group = "drive")
//public class BucketPEDRO extends LinearOpMode {
//    public static double DISTANCE = 24; // in
//
//    public Slide slide;
//    public Motors panningMotor;
//    public Servos claw;
//    public Servos panningServo;
//    public Servos orientation;
//    public Motors pulley;
//
//    private Follower follower;
//    private PathChain submersible0;
//    private PathChain submersible1;
//    private PathChain submersible2;
//    private PathChain submersible3;
//    private PathChain submersible35;
//    private PathChain submersible4;
//    private PathChain submersible5;
//    private PathChain submersible6;
//
//    private Pose currentPose;
//    int num1 = 0;
//    double time;
//
//    Thread bigBootyThread;
//
//    public int runFrames;
//
//    public HuskyLens.Block currentTarget;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        slide = new Slide(hardwareMap, telemetry, "slide");
//        panningMotor = new Motors(hardwareMap, "panningmotor");
//        claw = new Servos(hardwareMap, "claw");
//        panningServo = new Servos(hardwareMap, "panning");
//        orientation = new Servos(hardwareMap, "orientation");
//        pulley = new Motors(hardwareMap, "pulley");
////        frontLens = new HuskyLenses(hardwareMap, "frontlens", "color");
//        follower = new Follower(hardwareMap);
//        follower.setStartingPose(new Pose(10.412307692307692, 113.20615384615385,0));
//
//        runFrames = 0;
//        claw.moveForwardMAX();
//        panningServo.moveBackwardMIN();
//        orientation.moveBackwardMIN();
//        slide.MoveToLevel(Slide.level.zero);
//
//        PathBuilder builder0 = new PathBuilder();
//        PathBuilder builder1 = new PathBuilder();
//        PathBuilder builder2 = new PathBuilder();
//        PathBuilder builder3 = new PathBuilder();
//        PathBuilder builder35 = new PathBuilder();
//        PathBuilder builder4 = new PathBuilder();
//        PathBuilder builder5 = new PathBuilder();
//        PathBuilder builder6 = new PathBuilder();
//
//        builder0
//                .addPath(
//                        // Line 1
//                        new BezierLine(
//                                new Point(9.348, 113.370, Point.CARTESIAN),
//                                new Point(15.508, 130.486, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
//                .setZeroPowerAccelerationMultiplier(2);
//        builder1
//                .addPath(
//                        // Line 2
//                        new BezierLine(
//                                new Point(15.508, 130.486, Point.CARTESIAN),
//                                new Point(33.895, 122.954, Point.CARTESIAN)
//                        )
//                )
//                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
//                .setZeroPowerAccelerationMultiplier(2);
//
//                builder2
//                        .addPath(
//                                // Line 3
//                                new BezierLine(
//                                        new Point(33.895, 122.954, Point.CARTESIAN),
//                                        new Point(14.622, 130.929, Point.CARTESIAN)
//                                )
//                        )
//                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
//                        .setZeroPowerAccelerationMultiplier(2);
//                builder3
//                        .addPath(
//                                // Line 4
//                                new BezierLine(
//                                        new Point(15.729, 130.043, Point.CARTESIAN),
//                                        new Point(33.895, 132.480, Point.CARTESIAN)
//                                )
//                        )
//                        .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
//                        .setZeroPowerAccelerationMultiplier(2);
//
//                builder35
//                        .addPath(
//                                // Line 5
//                                new BezierLine(
//                                        new Point(33.895, 132.480, Point.CARTESIAN),
//                                        new Point(15.729, 130.265, Point.CARTESIAN)
//                                )
//                        )
//                        .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
//                        .setZeroPowerAccelerationMultiplier(2);
//
//                builder4
//                        .addPath(
//                                // Line 6
//                                new BezierLine(
//                                        new Point(15.729, 130.265, Point.CARTESIAN),
//                                        new Point(29.635, 133.459, Point.CARTESIAN)
//                                )
//                        )
//                        .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(39))
//                        .setZeroPowerAccelerationMultiplier(2);
//
//                builder5
//                        .addPath(
//                                // Line 7
//                                new BezierLine(
//                                        new Point(29.635, 133.459, Point.CARTESIAN),
//                                        new Point(15.508, 130.708, Point.CARTESIAN)
//                                )
//                        )
//                        .setLinearHeadingInterpolation(Math.toRadians(39), Math.toRadians(-45));
//
//        submersible0 = builder0.build();
//        submersible1 = builder1.build();
//        submersible2 = builder2.build();
//        submersible3 = builder3.build();
//        submersible35 = builder35.build();
//        submersible4 = builder4.build();
//        submersible5 = builder5.build();
//        submersible6 = builder6.build();
//
//
//
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (getRuntime() < 100) {
//            follower.update();
//            currentPose = follower.getPose();
//            time = getRuntime();
//            runAuto(num1, time);
//
//            telemetry.addData("X:", currentPose.getX());
//            telemetry.addData("Y:", currentPose.getY());
//            telemetry.update();
//        }
//
//        //first back to drop the first specimen
//        /*
//
//
//
//
//        drive.turn(Math.toRadians(30));
//        slide.MoveToLevel(Slide.level.pan_highbar);
//        sleep(500);
//        claw.moveForwardMAX();
//        sleep(700);
//        panningServo.moveSpecificPos(.35);
//        slide.MoveToLevelAsync(Slide.level.zero);
//        sleep(1000);
//        panningMotor.rotateForward(1, 300);
//        panningServo.moveSpecificPos(.35);
//        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
//        sleep(500);
//        drive.turn(Math.toRadians(-40));
//        sleep(500);
//        claw.moveBackwardMIN();
//        sleep(300);
//        slide.MoveToLevelAsync(Slide.level.zero);
//        panningServo.moveForwardMAX();
//        sleep(1000);
//        panningMotor.rotateForward(-0.8, 500);
//        drive.turn(Math.toRadians(40));
//        drive.followTrajectory(pickupA);
//        slide.MoveToLevel(Slide.level.pan_highbar);
//        sleep(500);
//        claw.moveForwardMAX();
//        sleep(700);
//        panningServo.moveSpecificPos(.35);
//        slide.MoveToLevelAsync(Slide.level.zero);
//        drive.followTrajectory(pickupB);
//        panningServo.moveForwardMAX();
//        drive.turn(Math.toRadians(-40));
//        panningMotor.rotateForward(1, 300);
//        slide.MoveToLevel(Slide.level.slide_highbucket);
//        panningServo.moveSpecificPos(.35);
//        sleep(500);
//        claw.moveBackwardMIN();
//        sleep(300);
//        slide.MoveToLevelAsync(Slide.level.zero);
//        panningServo.moveForwardMAX();
//        sleep(1000);
//        panningMotor.rotateForward(-0.8, 500);
//        drive.turn(Math.toRadians(65));
//        claw.moveSpecificPos(0.5);
//        slide.MoveToLevel(Slide.level.panigging);
//        sleep(500);
//        claw.moveForwardMAX();
//        sleep(700);
//        slide.MoveToLevelAsync(Slide.level.zero);
//        panningServo.moveSpecificPos(.35);
//        sleep(1000);
//        panningMotor.rotateForward(1, 300);
//        panningServo.moveSpecificPos(.35);
//        slide.MoveToLevelAsync(Slide.level.slide_highbucket);
//        drive.turn(Math.toRadians(-60));
//        sleep(500);
//        claw.moveBackwardMIN();
//        sleep(300);
//        slide.MoveToLevelAsync(Slide.level.zero);
//        panningServo.moveForwardMAX();
//        sleep(1000);*/
//    }
//
//    public void runAuto(int num, double time1) {
//        switch (num) {
//            case 0:
//                //line 1 starts
//                follower.followPath(submersible0);
//                panningMotor.rotateForward(0.8, 300);
//                slide.MoveToLevelAsync(Slide.level.slide_highbucket);
//                num1 = 1;
//                break;
//            case 1:
//                //line 1 ends
//                if (!follower.isBusy()) {
//                    panningServo.moveSpecificPos(.35);
//                    sleep(1000);
//                    claw.moveBackwardMIN();
//                    sleep(300);
//                    //line 2 starts
//                    follower.followPath(submersible1);
//                    panningServo.moveForwardMAX();
//                    sleep(500);
//                    slide.MoveToLevel(Slide.level.zero);
//
//                    num1 = 2;
//                }
//                break;
//            case 2:
//                if (!follower.isBusy()) {
//                    //line2 ends
//                    panningServo.moveForwardMAX();
//                    panningMotor.rotateForward(-0.8, 500);
//                    slide.MoveToLevel(Slide.level.panigging);
//                    claw.moveForwardMAX();
//                    sleep(300);
//                    follower.followPath(submersible2);
//                    //line3 starts
//                    panningMotor.rotateForward(0.8, 300);
//                    panningServo.moveForwardMAX();
//                    slide.MoveToLevel(Slide.level.slide_highbucket);
//                    num1 = 3;
//                }
//                break;
//            case 3:
//                if (!follower.isBusy()) {
//                    panningServo.moveSpecificPos(.35);
//                    sleep(400);
//                    //line3 ends
//                    claw.moveBackwardMIN();
//                    sleep(300);
//                    //line 4 starts
//                    follower.followPath(submersible3);
//                    panningServo.moveForwardMAX();
//                    sleep(500);
//                    slide.MoveToLevel(Slide.level.panigging);
//
//                    num1 = 4;
//                }
//                break;
//            case 4:
//                if (!follower.isBusy()) {
//                    //line4 ends
//                    panningMotor.rotateForward(-0.8, 500);
//                    sleep(300);
//                    claw.moveForwardMAX();
//                    sleep(300);
//                    follower.followPath(submersible35);
//                    //line5 starts
//                    panningMotor.rotateForward(0.8, 300);
//                    panningServo.moveForwardMAX();
//                    slide.MoveToLevel(Slide.level.slide_highbucket);
//                    num1 = 5;
//                }
//                break;
//            case 5:
//                if (!follower.isBusy()) {
//                    panningServo.moveSpecificPos(.35);
//                    sleep(400);
//                    //line5 ends
//                    claw.moveBackwardMIN();
//                    sleep(300);
//                    //line 6 starts
//                    follower.followPath(submersible4);
//                    panningServo.moveForwardMAX();
//                    sleep(500);
//                    slide.MoveToLevelAsync(Slide.level.zero);
//
//                    num1 = 6;
//                }
//                break;
//            case 6:
//                if (!follower.isBusy()) {
//                    panningMotor.rotateForward(-0.8, 500);
//                    slide.MoveToLevel(Slide.level.pan_highbucket);
//                    sleep(300);
//                    //line6 ends
//                    claw.moveForwardMAX();
//                    sleep(300);
//                    slide.MoveToLevel(Slide.level.zero);
//                    sleep(500);
//                    follower.followPath(submersible5);
//                    //line7 starts
//                    panningMotor.rotateForward(0.8, 300);
//                    panningServo.moveForwardMAX();
//                    slide.MoveToLevel(Slide.level.slide_highbucket);
//                    num1 = 7;
//                }
//                break;
//            case 7:
//                if (!follower.isBusy()) {
//                    panningServo.moveSpecificPos(.35);
//                    sleep(1000);
//                    //line7 ends
//                    claw.moveBackwardMIN();
//                    sleep(300);
//                    panningServo.moveForwardMAX();
//                    sleep(500);
//                    slide.MoveToLevel(Slide.level.zero);
//
//                    num1 = -1;
//                }
//                break;
//        }
//    }
//
//}
