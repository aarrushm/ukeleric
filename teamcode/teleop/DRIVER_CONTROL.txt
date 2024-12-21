package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.autonomous.HuskyLenses;
import org.firstinspires.ftc.teamcode.autonomous.IntakeClaw;
import org.firstinspires.ftc.teamcode.autonomous.Motors;
import org.firstinspires.ftc.teamcode.autonomous.Servos;
import org.firstinspires.ftc.teamcode.autonomous.Slides;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="DRIVER CONTROL", group="Linear Opmode") // @Autonomous(...) is the other common choice
public class DRIVER_CONTROL extends LinearOpMode /*implements Runnable*/ {
    public Slides slide;
    public Motors panningMotor;
    public Servos claw;
    public Servos orientation;
    public Servos panningServo;
    public HuskyLenses aprilLens;
    public IntakeClaw intakeClaw = new IntakeClaw();

    public DcMotor fl;
    public DcMotor bl;
    public DcMotor fr;
    public DcMotor br;

    boolean dpadRightPressed;
    boolean dpadLeftPressed;

    HuskyLens.Block tag;
    int xInitial;

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

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive()) {
            double slidespeed = 1.0;
            double velocity;
            if (gamepad1.right_trigger > 0.8) {
                velocity = 1.0;
            } else if (gamepad1.left_trigger > 0.8) {
                velocity = 0.3;
            } else {
                velocity = 0.75;
            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * velocity,
                            -gamepad1.right_stick_x*velocity,
                            -gamepad1.left_stick_x*velocity
                    )
            );

            if (gamepad1.x) {
                intakeClaw.moveClawLeft();
                claw.moveForwardMAX();
            } else if (gamepad1.b) {
                intakeClaw.moveClawLeft();
                claw.moveForwardMAX();
            }

            if (gamepad2.y) {
                panningServo.moveBackwardMIN();
            } else if (gamepad2.a) {
                panningServo.moveForwardMAX();
            }

            if (gamepad2.x) {
                orientation.moveSpecificPos(0.5);
            } else if (gamepad2.b) {
                orientation.moveBackwardMIN();
            }

            // Slide movement
            if (gamepad1.right_bumper) {
                slide.runForward(slidespeed, 0);
            } else if (gamepad1.left_bumper) {
                slide.runBackward(slidespeed, 0);
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
                /*aprilLens.updateObservedObjects();
                tag = aprilLens.getFirstObject();
                if (tag != null) {
                    xInitial = 120 - tag.x;

                    strafeRight(500);
                }*/
                // Pan up
                slidespeed = 1.0;
                panningMotor.rotateForward(0.8, 300);
                // Move slide up
                slide.runForward(1.0, 1600);
                // Move panning up
                panningServo.moveBackwardMIN();


            }

            if (gamepad2.dpad_left && !dpadLeftPressed) {
                claw.moveBackwardMIN();
                sleep(500);
                panningServo.moveForwardMAX();
                sleep(500);
                slide.runBackward(1.0, 1600);
                panningMotor.rotateBackward(-1.0, 300);
                sleep(500);
                slidespeed = 0.65;

            }

            dpadLeftPressed = gamepad2.dpad_left;
            dpadRightPressed = gamepad2.dpad_right;

        }
    }

    /*public void strafeRight(int timeMs) {
        fl.setPower(1.0); // backward
        fr.setPower(-1.0); // forward
        bl.setPower(-1.0); // forward
        br.setPower(1.0); // backward
        if (timeMs > 0 ) {
            sleep(timeMs);
            stopMovement();
        }
    }

    public void moveBackward(int timeMs) {
        fl.setPower(-1.0);
        fr.setPower(-1.0);
        bl.setPower(-1.0);
        br.setPower(-1.0);
        if (timeMs > 0 ) {
            sleep(timeMs);
            stopMovement();
        }
    }

    public void stopMovement() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }*/
}
