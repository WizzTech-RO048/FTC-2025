package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@TeleOp(name = "FTC2025")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private Controller controller1, controller2;
    private SampleMecanumDrive drive;

    private int raise_value, arm_value = 0, lift_value;
    public double RAISE_POWER = 1.0;

    public int ARM_MAX_POS = 600;
    public int SLIDER_MAX_POS = 5700;

    public double arm_percentage = 0.0;      // procent din cat sa ridice din ARM_MAX_POS (are valoarea intre 0.0 si 1.0)
    public double slider_percentage = 0.0;   // procent din cat sa ridice din SLIDER_MAX_POS (are valoarea intre 0.0 si 1.0)

    private boolean closed, armIsUp;
    private int gripper_position = 0; //0-oprit 1-aduna piesa 2-beleste piesa
    private boolean sculatoare;
    private int last_arm_position; // 0 - a, 1 - x, 2 - b, 3 - y
    private int slider_level = 0;
    private ScheduledFuture<?> lastArmMove, lastSliderMove;
    private ScheduledFuture<?> lastRightLift, lastLeftLift;
    //boolean isPressed = false;

    int slider_target_positionup = 0;
    int slider_target_positiondown = 0;

    int slider_target_position = 0;
    int arm_target_positionup = 0;
    int arm_target_positiondown = 0;

    int arm_target_position = 0;

    boolean isExtended_down = false;
    boolean isExtended_up = false;
    boolean gripper_rotating = false;
    boolean lb_down = false;
    boolean lift_position = false;

    double movement_speed = 1.0;

    // ----- for generating telemetry logs -----
    private FileWriter writer;
    private long startTime;

    @Override
    public void init() {
        robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );

        // ====================================================
        // === controller1 - movement si control intake     ===
        // === controller1 - control slider si control arm  ===
        // ====================================================
        controller1 = new Controller(gamepad1);
        controller2 = new Controller(gamepad2);


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        last_arm_position = 0;


        // ----- for generating telemetry logs -----
        File logFile = new File(Environment.getExternalStorageDirectory().getPath() + "/FIRST/encoder_log.txt");
        try {
            writer = new FileWriter(logFile, false); // Overwrites existing file
            writer.write("Time(ms), Encoder Position\n"); // Header row
            telemetry.addData("Log", "Initialized. File: encoder_log.txt");
        } catch (IOException e) {
            telemetry.addData("Error", "File write failed: " + e.getMessage());
        }
        startTime = System.currentTimeMillis();


    }

    // ------ the emergency stop function ---------
//    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();
        controller2.update();

        //isPressed=false;
//        slider_target_positionup = robot.slider.getCurrentPositionSlider();
//        slider_target_positiondown = robot.slider.getCurrentPositionSlider();
//        arm_target_positionup = robot.arm.getCurrentPositionArm();
//        arm_target_positiondown = robot.arm.getCurrentPositionArm();

        if (arm_value < 300) {
            isExtended_down = true;
            isExtended_up = false;
        } else {
            isExtended_up = true;
            isExtended_down = false;
        }
        // controller 1
        // - movement
        // - ridcare arm
        // - intake gripper
        // - control slider
        // - lift


        // --- limitare de viteza in cazul in care e arm-ul e ridicat sus ---
        if (robot.arm.getCurrentPositionArm() >= 350) {
            movement_speed = 0.5;
        } else {
            movement_speed = 1.0;
        }

        // TODO: normalizare valori motor arm / schimbare valori

        // =======================
        // ===== DRIVER 1 ========
        // =======================

        // --------- movement general al robotului ---------
        drive.setWeightedDrivePower(
                new Pose2d( /// imi pare rau pt ce urmeaza sa fac, timpuri disperate...
                        (controller1.left_stick_y * movement_speed),
                        (controller1.left_stick_x * movement_speed),
                        (controller1.right_stick_x * movement_speed)  // gen astea negative / pozitive sau schimbate intre ele
                )
        );

        //----------- gripper ---------------
        double left_trig = controller1.left_trigger;
        double right_trig = controller1.right_trigger;
        if (left_trig > 0) {
            robot.gripper.grab_position();
        } else if (right_trig > 0) {
            robot.gripper.release_position();
        }
        else robot.gripper.no_position();

        // =======================
        // ===== DRIVER 2 ========
        // =======================

        if (!Utils.isDone(lastArmMove) || !Utils.isDone(lastSliderMove)) {
            return;
        } else {
            // --------- extindere slider ---------
            if (controller1.YOnce()) {
                if (robot.arm.getCurrentPositionArm() <= 200) {
                    arm_percentage = 1.0;
                    robot.arm.raiseArm((int)(arm_percentage * ARM_MAX_POS), RAISE_POWER);
                }
                else{
                    slider_percentage = 1.0;
                    robot.slider.raiseSlider((int)(slider_percentage * SLIDER_MAX_POS), RAISE_POWER);
                }
            }

            // --------- retractie slider ---------
            if (controller1.AOnce()) {
                if (robot.arm.getCurrentPositionArm() <= 200) {
                    arm_percentage = 0.25;
                    robot.arm.raiseArm((int) (arm_percentage * ARM_MAX_POS), RAISE_POWER / 2);
                }

                robot.slider.raiseSlider(0, RAISE_POWER);
            }


            // --------- extindere slider controlat ---------
            if (controller1.B()) {  /// extend slider a bittt
                if (slider_target_position <= 5500) {
                    // if not at max
                    if (robot.arm.getCurrentPositionArm() < 300) {
                        arm_percentage = 0.25;
                        robot.arm.raiseArm((int)(arm_percentage * ARM_MAX_POS), RAISE_POWER);
                        robot.slider.raiseSlider(2100, RAISE_POWER);
                    } else {
                        robot.slider.raiseSlider(robot.slider.getCurrentPositionSlider() + 200, RAISE_POWER);
                    }
                } else {
                    robot.slider.raiseSlider(5700, RAISE_POWER);
                }
            }

            // --------- retractie slider controlat ---------
            if (controller1.X()) {
                if (slider_percentage >= 0.1) {
                    if (arm_percentage <= 0.5) {
                        arm_percentage = 0.5;
                        robot.arm.raiseArm((int)(arm_percentage * ARM_MAX_POS), RAISE_POWER);
                    }
                    slider_percentage = slider_percentage - 0.1;
                    robot.slider.raiseSlider((int)(slider_percentage * SLIDER_MAX_POS), RAISE_POWER);
                } else {
                    slider_percentage = 0.0;
                    robot.slider.raiseSlider((int)(slider_percentage * SLIDER_MAX_POS), RAISE_POWER);
                }
            }


            // -------  controlling the arm positions -----

            if (controller1.dpadUpOnce()) {
                if (robot.slider.getCurrentPositionSlider() > 10) {
                    slider_target_position = 0;
                    robot.slider.raiseSlider(slider_target_position, RAISE_POWER);
                }
                arm_value = 700;
                robot.arm.raiseArm(arm_value, RAISE_POWER);
            }
            else if (controller1.dpadDownOnce()) {
                if (robot.slider.getCurrentPositionSlider() > 10 && robot.arm.getCurrentPositionArm() > 400) {
                    slider_target_position = 0;
                    robot.slider.raiseSlider(slider_target_position, RAISE_POWER);
                }
                if (robot.slider.getCurrentPositionSlider() < 2500) {
                    arm_value = 175;
                    robot.arm.raiseArm(arm_value, RAISE_POWER / 2);
                }
            }

            if (controller1.rightBumper()) {
//            if(robot.slider.getCurrentPositionSlider() > 10){
//                slider_target_positiondown = 0;
//                robot.slider.raiseSlider(slider_target_positiondown, RAISE_POWER);
//            }
                if (arm_target_position <= 600) {
                    arm_target_position = robot.arm.getCurrentPositionArm() + 100;
                    robot.arm.raiseArm(arm_target_position, RAISE_POWER);
                }
            }
            if (controller1.leftBumper()) {
//            if(robot.slider.getCurrentPositionSlider() > 10){
//                slider_target_positiondown = 0;
//                robot.slider.raiseSlider(slider_target_positiondown, RAISE_POWER);
//            }
                if (arm_target_position >= 100) {
                    arm_target_position = robot.arm.getCurrentPositionArm() - 75;
                    robot.arm.raiseArm(arm_target_position, RAISE_POWER / 2);
                }
                else { robot.arm.raiseArm(0,RAISE_POWER/2);}
            }
        }
            // ---------- controale lift -------------
//        if (controller1.dpadLeftOnce()) {
//            if (!lift_position) {
//                lift_value = 0;
//                lastRightLift = robot.lift.liftUpLeft(lift_value, 1);
//                lastLeftLift = robot.lift.liftUpRight(lift_value, 1);
//
//                robot.lift.setUpPosition();
//
//                lift_position = !lift_position;
//            } else {
//                lift_value = -1000;
//                lastRightLift = robot.lift.liftUpLeft(lift_value, 1);
//                lastLeftLift = robot.lift.liftUpRight(lift_value, 1);
//
//                robot.lift.setDownPosition();
//
//                lift_position = !lift_position;
//            }
//        }
//        if (!Utils.isDone(lastRightLift) || !Utils.isDone(lastLeftLift)) {
//            return;
//        } else if (controller1.dpadRightOnce()) {
//            arm_value = 0;
//            robot.arm.raiseArm(arm_value, RAISE_POWER);
//            lift_value = 7000;
//            lastRightLift = robot.lift.liftUpLeft(lift_value, RAISE_POWER);
//            lastLeftLift = robot.lift.liftUpRight(lift_value, RAISE_POWER);
//        }


            // ------- printing the slider position -------
//            telemetry.addData("Slider target value", isPressed );
        telemetry.addData("Slider position", robot.slider.getCurrentPositionSlider());
        telemetry.addLine("---------------------");
        telemetry.addData("Arm target value", arm_value);
        telemetry.addData("Arm position", robot.arm.getCurrentPositionArm());
        telemetry.addLine("---------------------");
        telemetry.addData("Lift 1 level", robot.lift.getCurrentPositionServoLeft());
        telemetry.addData("Lift 2 level", robot.lift.getCurrentPositionServoRight());
        telemetry.addLine("---------------------");
        telemetry.addLine("---------------------");
        telemetry.addData("Lift_Value", lift_value);


            //telemetry.addData("Lift target value", arm_value);
            //telemetry.addData("lift position", robot.lift.getCurrentPositionArm());



        // ------ printing data in the telemetry logs file ------
        long timestamp = System.currentTimeMillis() - startTime;
        try {
            if (writer != null) {
                writer.write(timestamp + "," + robot.arm.getCurrentPositionArm() + "\n");
                writer.flush(); // Ensure immediate writing
            }
        } catch (IOException e) {
            telemetry.addData("Error", "Logging failed: " + e.getMessage());
        }

        // Show telemetry
        telemetry.addData("Encoder", robot.arm.getCurrentPositionArm());
        telemetry.addData("Log", "Saving to encoder_log.txt");
        telemetry.update();

        // TODO: use 'adb pull /sdcard/FIRST/encoder_log.txt' for getting the file

    }
}