package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.HuskyLenses;
import org.firstinspires.ftc.teamcode.autonomous.IntakeClaw;
import org.firstinspires.ftc.teamcode.autonomous.Motors;
import org.firstinspires.ftc.teamcode.autonomous.Servos;
import org.firstinspires.ftc.teamcode.autonomous.Slides;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

@TeleOp(name="PedroDriverControl", group="Linear Opmode") // @Autonomous(...) is the other common choice
public class PedroDriver extends LinearOpMode /*implements Runnable*/ {
    public Slides slide;
    public Motors panningMotor;
    public Servos claw;
    public Servos orientation;
    public Servos panningServo;
    public HuskyLenses aprilLens;
    public IntakeClaw intakeClaw = new IntakeClaw();

    private Follower follower;
    private Pose currentPose;

    double velocity;

    public DcMotor fl;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor br;

    boolean dpadRightPressed;
    boolean dpadLeftPressed;

    HuskyLens.Block tag;
    int runFrames = 0;
    int bucketCase = 0;
    double orientationPos = 0;

    private PathChain toBucket, toSubmersible;
    private Pose bucketPose, subPose;

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        slide = new Slides(hardwareMap, "slide", 6000);
        panningMotor = new Motors(hardwareMap, "panningmotor");
        intakeClaw.getHardware(hardwareMap);
        claw = new Servos(hardwareMap, "claw");
        orientation = new Servos(hardwareMap, "orientation");
        panningServo = new Servos(hardwareMap, "panning");

        orientation.moveBackwardMIN();

        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
        follower.setStartingPose(new Pose(133.809, 86.622,0));

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.8) {
                velocity = 1;
            } else if (gamepad1.left_trigger > 0.8) {
                velocity = 0.25;
            } else {
                velocity = 0.75;
            }

            if (gamepad2.y) {
                panningServo.moveBackwardMIN();
            } else if (gamepad2.a) {
                panningServo.moveForwardMAX();
            }

            if (gamepad2.x) {
                if (orientationPos > 0) {
                    moveToPos(0);
                }
            } else if (gamepad2.b) {
                if (orientationPos < 1) {
                    moveToPos(orientationPos + 0.01);
                }
            }

            // Slide movement
            if (gamepad1.right_bumper) {
                slide.runForward(100000, 0);
            } else if (gamepad1.left_bumper) {
                slide.runBackward(100000, 0);
            } else {
                slide.stopSlide();
            }


            if (gamepad2.right_trigger > 0.8) {
                panningMotor.rotateForward(1.0, 0);
            } else if (gamepad2.left_trigger > 0.8) {
                panningMotor.rotateBackward(-1.0, 0);
            } else {
                panningMotor.stopRotation();
            }

            if (gamepad2.dpad_up) {
                claw.moveForwardMAX();
            } else if (gamepad2.dpad_down) {
                claw.moveBackwardMIN();
            }


            if (gamepad2.dpad_right && !dpadRightPressed) {
                if (bucketCase == 0) {
                    bucketCase = 1;
                } else {
                    bucketCase = 0;
                }
            }

            if (gamepad2.dpad_left && !dpadLeftPressed) {
                claw.moveBackwardMIN();
                sleep(500);
                panningServo.moveForwardMAX();
                sleep(500);
                slide.runBackward(1.0, 1600);
                panningMotor.rotateBackward(-1.0, 300);
            }

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y * velocity, -gamepad1.left_stick_x * velocity, -gamepad1.right_stick_x * velocity);

            follower.update();
            currentPose = follower.getPose();
            runToBucket(bucketCase);

            dpadLeftPressed = gamepad2.dpad_left;
            dpadRightPressed = gamepad2.dpad_right;
        }
    }
    public void runToBucket(int num) {
        switch (num) {
            case 1:

                break;
        }
    }

    public void moveToPos(double pos) {
        orientationPos = pos;
        orientation.moveSpecificPos(pos);
    }
}
