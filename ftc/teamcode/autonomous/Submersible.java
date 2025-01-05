package org.firstinspires.ftc.teamcode.autonomous;


import static java.lang.Thread.sleep;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.parts.Claw;
import org.firstinspires.ftc.teamcode.parts.PIDFPanning;
import org.firstinspires.ftc.teamcode.parts.PIDFSlide;
import org.firstinspires.ftc.teamcode.parts.Panning;
import org.firstinspires.ftc.teamcode.parts.Servos;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name = "SUBMERSIBLE", group = "Game Auto")
public class Submersible extends OpMode {
    // Parts
    public Panning panning;
    public Claw claw;
    public Servos orientation;
    public Servos pitching;
    public Servos panningServo;
    public PIDFSlide slides;

    // Follower stuff
    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    /* Poses */
    private final Pose startPose = new Pose(135, 80, 0); // line 1
    private final Pose hangPose1 = new Pose(106, 78, 0); // line 1
    private final Pose pickUpPose1 = new Pose(106, 107, 120); // line 2
    private final Pose dropOffPose1 = new Pose(115, 112, 40); // line 3
    private final Pose pickUpPose2 = new Pose(102, 117, 105); // line 4
    private final Pose dropOffPose2 = new Pose(115, 125, 45); // line 5
    private final Pose pickUpPose3 = new Pose(106, 131, 125); // line 6
    private final Pose grab1 = new Pose(125.5, 110.5, 0); // line 7
    private final Pose grab = new Pose(125.5, 110.5, 0); // lines 9, 11, 13
    private final Pose hangPose2 = new Pose(104.5, 76, 0); // line 8
    private final Pose hangPose3 = new Pose(104.5, 74, 0); // line 10
    private final Pose hangPose4 = new Pose(104.5, 72, 0); // line 12
    private final Pose hangPose5 = new Pose(104.5, 70, 0); // line 14
    private final Pose park = new Pose(122, 97, 60); // line 15

    /* Control Points */
    private final Point pickUpPose1Control = new Point(119, 80);
    private final Point grab1Control = new Point(122, 129);
    private final Point hangControl = new Point(124, 69);
    private final Point grabControl = new Point(115, 80);


    // Path chains to be used in buildPaths()
    private PathChain hangFirst, grabFirst, dropOffFirst, grabSecond,
            dropOffSecond, grabThird, grabSpecSecond, hangSpecSecond,
            grabSpecThird, hangSpecThird, grabSpecFourth, hangSpecFourth,
            grabSpecFifth, hangSpecFifth, endPark, pathTest;

    // Builds all paths prior to running the auto (init)
    public void buildPaths() {
        hangFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(hangPose1)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        grabFirst = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hangPose1), pickUpPose1Control, new Point(pickUpPose1)))
                .setLinearHeadingInterpolation(hangPose1.getHeading(), pickUpPose1.getHeading())
                .build();
        dropOffFirst = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose1), new Point(dropOffPose1)))
                .setLinearHeadingInterpolation(pickUpPose1.getHeading(), dropOffPose1.getHeading())
                .build();
        grabSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropOffPose1), new Point(pickUpPose2)))
                .setLinearHeadingInterpolation(dropOffPose1.getHeading(), pickUpPose2.getHeading())
                .build();
        dropOffSecond = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickUpPose2), new Point(dropOffPose2)))
                .setLinearHeadingInterpolation(pickUpPose2.getHeading(), dropOffPose2.getHeading())
                .build();
        grabThird = follower.pathBuilder()
                .addPath(new BezierLine(new Point(dropOffPose2), new Point(pickUpPose3)))
                .setLinearHeadingInterpolation(dropOffPose2.getHeading(), pickUpPose3.getHeading())
                .build();
        grabSpecSecond = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(pickUpPose3), grab1Control, new Point(grab1)))
                .setLinearHeadingInterpolation(pickUpPose3.getHeading(), grab1.getHeading())
                .build();
        hangSpecSecond = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab1), hangControl, new Point(hangPose2)))
                .setConstantHeadingInterpolation(0)
                .build();
        grabSpecThird = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hangPose2), grabControl, new Point(grab)))
                .setConstantHeadingInterpolation(0)
                .build();
        hangSpecThird = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab), hangControl, new Point(hangPose3)))
                .setConstantHeadingInterpolation(0)
                .build();
        grabSpecFourth = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hangPose3), grabControl, new Point(grab)))
                .setConstantHeadingInterpolation(0)
                .build();
        hangSpecFourth = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab), hangControl, new Point(hangPose4)))
                .setConstantHeadingInterpolation(0)
                .build();
        grabSpecFifth = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(hangPose4), grabControl, new Point(grab)))
                .setConstantHeadingInterpolation(0)
                .build();
        hangSpecFifth = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(grab), hangControl, new Point(hangPose5)))
                .setConstantHeadingInterpolation(0)
                .build();
        endPark = follower.pathBuilder()
                .addPath(new BezierLine(new Point(hangPose5), new Point(park)))
                .setLinearHeadingInterpolation(hangPose5.getHeading(), park.getHeading())
                .build();

        pathTest = follower.pathBuilder()
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(134.917, 79.975, Point.CARTESIAN),
                                new Point(104.446, 78.077, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 2
                        new BezierCurve(
                                new Point(104.446, 78.077, Point.CARTESIAN),
                                new Point(119.279, 80.137, Point.CARTESIAN),
                                new Point(105.888, 106.918, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(120))
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(105.888, 106.918, Point.CARTESIAN),
                                new Point(115.365, 111.657, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(40))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(115.365, 111.657, Point.CARTESIAN),
                                new Point(101.768, 117.013, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(105))
                .addPath(
                        // Line 5
                        new BezierLine(
                                new Point(101.768, 117.013, Point.CARTESIAN),
                                new Point(115.365, 125.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(105), Math.toRadians(45))
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(115.365, 125.000, Point.CARTESIAN),
                                new Point(105.682, 131.227, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(125))
                .addPath(
                        // Line 7
                        new BezierCurve(
                                new Point(105.682, 131.227, Point.CARTESIAN),
                                new Point(121.545, 129.373, Point.CARTESIAN),
                                new Point(125.459, 110.627, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(125), Math.toRadians(0))
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(125.459, 110.627, Point.CARTESIAN),
                                new Point(124.017, 69.219, Point.CARTESIAN),
                                new Point(104.446, 75.811, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 9
                        new BezierCurve(
                                new Point(104.446, 75.811, Point.CARTESIAN),
                                new Point(115.000, 80.000, Point.CARTESIAN),
                                new Point(125.459, 110.627, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(125.459, 110.627, Point.CARTESIAN),
                                new Point(124.017, 69.219, Point.CARTESIAN),
                                new Point(104.652, 73.545, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(104.652, 73.545, Point.CARTESIAN),
                                new Point(115.000, 80.000, Point.CARTESIAN),
                                new Point(125.459, 110.627, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 12
                        new BezierCurve(
                                new Point(125.459, 110.627, Point.CARTESIAN),
                                new Point(124.017, 69.219, Point.CARTESIAN),
                                new Point(104.446, 71.279, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 13
                        new BezierCurve(
                                new Point(104.446, 71.279, Point.CARTESIAN),
                                new Point(115.000, 80.000, Point.CARTESIAN),
                                new Point(125.459, 110.627, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 14
                        new BezierCurve(
                                new Point(125.459, 110.627, Point.CARTESIAN),
                                new Point(124.017, 69.219, Point.CARTESIAN),
                                new Point(104.652, 68.807, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                        // Line 15
                        new BezierLine(
                                new Point(104.652, 68.807, Point.CARTESIAN),
                                new Point(121.957, 96.618, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(60)).build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() throws InterruptedException {
        switch (pathState) {
            case 0:
                // Hang the first block
                panning.runUp();
                while (pathTimer.getElapsedTime() < 1600) {}
                panning.stop();
                follower.followPath(hangFirst);
                movePanServoScore();
                setPathState(1);
                break;
            case 1:
                // Follow the path to the first block
                if (follower.getPose().getX() > (hangPose1.getX() - 1) && follower.getPose().getY() > (hangPose1.getY() - 1)) {
                    panningServo.moveToMin();
                    sleep(1000);
                }
                break;
                /*
            case 2:
                // Once you reach the block, grab it
                if (follower.getPose().getX() > (pickUpPose1.getX() - 1) && follower.getPose().getY() > (pickUpPose1.getY() - 1)) {
                    do {
                        detectedBlock = clawLens.getFirstObject();
                    } while (detectedBlock != null && detectedBlock.width < 20 && detectedBlock.height < 20);
                    orientation.moveSpecificPos(clawLens.calculateServoPosition(detectedBlock.height, detectedBlock.width));
                    panningServo.moveToMin();
                    claw.closeClaw();
                    follower.followPath(dropOffFirst);
                    setPathState(3);
                }
                break;
            case 3:
                // Drop off the first block in area
                if (follower.getPose().getX() > (dropOffPose1.getX() - 1) && follower.getPose().getY() > (dropOffPose1.getY() - 1)) {
                    follower.followPath(grabSecond);
                    claw.openClaw();
                    movePanServoReady();
                    setPathState(4);
                }
                break;
            case 4:
                // Pick up second block
                if (follower.getPose().getX() > (pickUpPose2.getX() - 1) && follower.getPose().getY() > (pickUpPose2.getY() - 1)) {
                    do {
                        detectedBlock = clawLens.getFirstObject();
                    } while (detectedBlock != null && detectedBlock.width < 20 && detectedBlock.height < 20);
                    orientation.moveSpecificPos(clawLens.calculateServoPosition(detectedBlock.height, detectedBlock.width));
                    panningServo.moveToMin();
                    claw.closeClaw();
                    follower.followPath(dropOffSecond);
                    setPathState(5);
                }
                break;
            case 5:
                // Drop off second block
                if (follower.getPose().getX() > (dropOffPose2.getX() - 1) && follower.getPose().getY() > (dropOffPose2.getY() - 1)) {
                    follower.followPath(grabThird);
                    claw.openClaw();
                    movePanServoReady();
                    setPathState(6);
                }
                break;
            case 6:
                // Pick up third block
                if (follower.getPose().getX() > (pickUpPose3.getX() - 1) && follower.getPose().getY() > (pickUpPose3.getY() - 1)) {
                    do {
                        detectedBlock = clawLens.getFirstObject();
                    } while (detectedBlock != null && detectedBlock.width < 20 && detectedBlock.height < 20);
                    orientation.moveSpecificPos(clawLens.calculateServoPosition(detectedBlock.height, detectedBlock.width));
                    panningServo.moveToMin();
                    claw.closeClaw();
                    follower.followPath(grabSpecSecond);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTime() > 500) {
                    claw.openClaw();
                    movePanServoGrab();
                    setPathState(8);
                }
                break;
            case 8:
                if (follower.getPose().getX() > (grab1.getX() - 1) && follower.getPose().getY() > (grab1.getY() - 1)) {
                    slides.runSlidesForward(300);
                    claw.closeClaw();
                    follower.followPath(hangSpecSecond);
                    movePanServoScore();
                    slides.runSlidesAsync(0);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTime() > 1) {
                    movePanningUp(300);
                    setPathState(10);
                }
                break;
            case 10:
                if (follower.getPose().getX() > (hangPose2.getX() - 1) && follower.getPose().getY() > (hangPose2.getY() - 1)) {
                    claw.openClaw();
                    follower.followPath(grabSpecThird);
                    movePanningDown(300);
                    movePanServoReady();
                    slides.runSlidesAsync(1000);
                    setPathState(11);
                }
                break;
            case 11:
                if (follower.getPose().getX() > (grab.getX() - 1) && follower.getPose().getY() > (grab.getY() - 1)) {
                    slides.runSlidesForward(300);
                    claw.closeClaw();
                    follower.followPath(hangSpecThird);
                    movePanServoScore();
                    slides.runSlidesAsync(0);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTime() > 1) {
                    movePanningUp(300);
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.getPose().getX() > (hangPose3.getX() - 1) && follower.getPose().getY() > (hangPose3.getY() - 1)) {
                    claw.openClaw();
                    follower.followPath(grabSpecFourth);
                    movePanningDown(300);
                    movePanServoReady();
                    slides.runSlidesAsync(1000);
                    setPathState(14);
                }
                break;
            case 14:
                if (follower.getPose().getX() > (grab.getX() - 1) && follower.getPose().getY() > (grab.getY() - 1)) {
                    slides.runSlidesForward(300);
                    claw.closeClaw();
                    follower.followPath(hangSpecFourth);
                    movePanServoScore();
                    slides.runSlidesAsync(0);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTime() > 1) {
                    movePanningUp(300);
                    setPathState(16);
                }
                break;
            case 16:
                if (follower.getPose().getX() > (hangPose4.getX() - 1) && follower.getPose().getY() > (hangPose4.getY() - 1)) {
                    claw.openClaw();
                    follower.followPath(grabSpecFifth);
                    movePanningDown(300);
                    movePanServoReady();
                    slides.runSlidesAsync(1000);
                    setPathState(17);
                }
                break;
            case 17:
                if (follower.getPose().getX() > (grab.getX() - 1) && follower.getPose().getY() > (grab.getY() - 1)) {
                    slides.runSlidesForward(300);
                    claw.closeClaw();
                    follower.followPath(hangSpecFifth);
                    movePanServoScore();
                    slides.runSlidesAsync(0);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTime() > 1) {
                    movePanningUp(300);
                    setPathState(19);
                }
                break;
            case 19:
                if (follower.getPose().getX() > (hangPose5.getX() - 1) && follower.getPose().getY() > (hangPose5.getY() - 1)) {
                    claw.openClaw();
                    follower.followPath(endPark);
                    movePanningDown(300);
                    slides.runSlidesAsync(1000);
                    setPathState(-1);
                }
                break;*/
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        try {
            autonomousPathUpdate();
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    // Runs when "init" is pressed
    @Override
    public void init() {
        // Get things from hardware map
        slides = new PIDFSlide(hardwareMap);
        panning = new Panning(hardwareMap);
        claw = new Claw(hardwareMap);
        pitching = new Servos(hardwareMap, "pitching");
        orientation = new Servos(hardwareMap, "orientation");
        panningServo = new Servos(hardwareMap, "panning");

        // Make timers
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Set up follower
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Place parts in initial positions
        claw.closeClaw();
        panningServo.moveToMin();
        pitching.moveToMin();
        orientation.moveToMin();
        slides.setTargetPos(20);
    }

    // Runs when "start" is pressed
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    // Stop method to stop the opMode
    @Override
    public void stop() {}

    // Helper functions

    public void movePanServoScore() { panningServo.moveSpecificPos(0.2); }
    public void movePanServoGrab() { panningServo.moveSpecificPos(0.35); }
    public void movePanServoReady() {panningServo.moveSpecificPos(0.2);}
}