package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@TeleOp(name = "FTC2025")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private Controller controller1;
    private SampleMecanumDrive drive;

    private int raise_value, arm_value = 0, lift_value;
    public double RAISE_POWER = 1.0;

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
    int arm_target_positionup = 0;
    int arm_target_positiondown = 0;

    boolean isExtended_down = false;
    boolean isExtended_up = false;
    boolean gripper_rotating = false;
    boolean lb_down = false;
    boolean lift_position = false;

    double movement_speed = 1.0;

    @Override
    public void init() {
        robot = new Robot(
                hardwareMap,
                telemetry,
                Executors.newScheduledThreadPool(1)
        );
        controller1 = new Controller(gamepad1);


        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //robot.gripper.openBarier();
        //robot.lift.setDownPosition();
        //closed = false;
        //sculatoare = false;
        last_arm_position = 0;
//        arm_value = 150;
//        robot.arm.raiseArm(arm_value, RAISE_POWER - 0.6);
        /*robot.lift.setUpPosition();*/
    }

    // ------ the emergency stop function ---------
//    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();
        //isPressed=false;
        slider_target_positionup = robot.slider.getCurrentPositionSlider();
        slider_target_positiondown = robot.slider.getCurrentPositionSlider();
        arm_target_positionup = robot.arm.getCurrentPositionArm();
        arm_target_positiondown = robot.arm.getCurrentPositionArm();

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



        // --------- movement general al robotului ---------
        drive.setWeightedDrivePower(
                new Pose2d(
                        (controller1.left_stick_y * movement_speed),
                        (controller1.right_stick_x * movement_speed),
                        (controller1.left_stick_x * movement_speed)  // gen astea negative / pozitive sau schimbate intre ele
                )
        );
        if (!Utils.isDone(lastArmMove) || !Utils.isDone(lastSliderMove)) {
            return;
        } else {
            // --------- extindere slider ---------
            if (controller1.YOnce()) {
                if (robot.arm.getCurrentPositionArm() <= 200)  /// check if arm is raised || raise arm
                    robot.arm.raiseArm(200, RAISE_POWER);
                robot.slider.raiseSlider(5700, RAISE_POWER);
            }

            // --------- retractie slider ---------
            if (controller1.AOnce()) {
                if (robot.arm.getCurrentPositionArm() <= 200)  /// check if arm is raised || raise arm
                    robot.arm.raiseArm(200, RAISE_POWER / 2);
                robot.slider.raiseSlider(0, RAISE_POWER);
            }


            // --------- extindere slider controlat ---------
            if (controller1.B()) {  /// extend slider a bittt
                if (slider_target_positionup <= 5500) {
                    // if not at max
                    if (robot.arm.getCurrentPositionArm() < 300) {
                        robot.arm.raiseArm(100, RAISE_POWER);
                        robot.slider.raiseSlider(2100, RAISE_POWER);
                    } else {
//                        if(robot.arm.getCurrentPositionArm() <= 180) {
//                            robot.arm.raiseArm(100, RAISE_POWER);
//                        }
                        robot.slider.raiseSlider(robot.slider.getCurrentPositionSlider() + 200, RAISE_POWER);
                    }
                } else {
                    robot.slider.raiseSlider(5700, RAISE_POWER);
                }
            }

            // --------- retractie slider controlat ---------
            if (controller1.X()) {
                if (slider_target_positiondown >= 200) {
                    if (robot.arm.getCurrentPositionArm() <= 200)  /// check if arm is raised || raise arm
                        robot.arm.raiseArm(200, RAISE_POWER);
                    robot.slider.raiseSlider(robot.slider.getCurrentPositionSlider() - 200, RAISE_POWER);

                } else {
                    robot.slider.raiseSlider(0, RAISE_POWER);
                }
            }

//        // --------- verificare slider extins in jos ---------
//        if(isExtended_down){
//            arm_value = 200;
//            robot.arm.raiseArm(arm_value, RAISE_POWER);
//            isExtended_down = false;
//        }
//
//        // --------- verificare slider extins in sus ---------
//        if(isExtended_up){
//            arm_value = 800;
//            robot.arm.raiseArm(arm_value, RAISE_POWER);
//            isExtended_up = false;
//        }
        }

        // -------  controlling the arm positions -----
        if (!Utils.isDone(lastArmMove) || !Utils.isDone(lastSliderMove)) {
            return;
        } else if (controller1.dpadUpOnce()) {
            if (robot.slider.getCurrentPositionSlider() > 10) {
                slider_target_positiondown = 0;
                robot.slider.raiseSlider(slider_target_positiondown, RAISE_POWER);
            }
            arm_value = 400;
            robot.arm.raiseArm(arm_value, RAISE_POWER);
        } else if (controller1.dpadDownOnce()) {
            if (robot.slider.getCurrentPositionSlider() > 10 && robot.arm.getCurrentPositionArm() > 400) {
                slider_target_positiondown = 0;
                robot.slider.raiseSlider(slider_target_positiondown, RAISE_POWER);
            }
            if (robot.slider.getCurrentPositionSlider() < 2500) {
                arm_value = 100;
                robot.arm.raiseArm(arm_value, RAISE_POWER / 2);
            }
        }
        if (controller1.rightBumper()) {
//            if(robot.slider.getCurrentPositionSlider() > 10){
//                slider_target_positiondown = 0;
//                robot.slider.raiseSlider(slider_target_positiondown, RAISE_POWER);
//            }
            if (arm_target_positionup <= 400) {
                arm_target_positionup = robot.arm.getCurrentPositionArm() + 100;
                robot.arm.raiseArm(arm_target_positionup, RAISE_POWER);
            }
        }
        if (controller1.leftBumper()) {
//            if(robot.slider.getCurrentPositionSlider() > 10){
//                slider_target_positiondown = 0;
//                robot.slider.raiseSlider(slider_target_positiondown, RAISE_POWER);
//            }
            if (arm_target_positiondown >= 100) {
                arm_target_positiondown = robot.arm.getCurrentPositionArm() - 75;
                robot.arm.raiseArm(arm_target_positiondown, RAISE_POWER / 2);
            }
        }


        //----------- gripper ---------------
        double left_trig = controller1.left_trigger;
        double right_trig = controller1.right_trigger;
        if (left_trig > 0) {
            robot.gripper.grab_position();
            gripper_position = 1;
        } else if (right_trig > 0) {
            robot.gripper.release_position();
            gripper_position = 2;
        } else {
            robot.gripper.no_position();
            gripper_position = 0;
        }

//        if(controller1.leftBumperOnce())
//        {
//            if(lb_down==false){
//                arm_value=50;
//                robot.arm.raiseArm(arm_value, RAISE_POWER - 0.6);
//                lb_down =true;
//            }else{
//                arm_value = 150;
//                robot.arm.raiseArm(arm_value, RAISE_POWER-0.6);
//                lb_down =false;
//            }
//
//        }


        //if(controller1.rightBumperOnce())
        //{

        //}
        // ---------- controale lift -------------
        if (controller1.dpadLeftOnce()) {
            if (!lift_position) {
                lift_value = 0;
                lastRightLift = robot.lift.liftUpLeft(lift_value, 1);
                lastLeftLift = robot.lift.liftUpRight(lift_value, 1);

                robot.lift.setUpPosition();

                lift_position = !lift_position;
            } else {
                lift_value = -1000;
                lastRightLift = robot.lift.liftUpLeft(lift_value, 1);
                lastLeftLift = robot.lift.liftUpRight(lift_value, 1);

                robot.lift.setDownPosition();

                lift_position = !lift_position;
            }
        }
        if (!Utils.isDone(lastRightLift) || !Utils.isDone(lastLeftLift)) {
            return;
        } else if (controller1.dpadRightOnce()) {
            arm_value = 0;
            robot.arm.raiseArm(arm_value, RAISE_POWER);
            lift_value = 7000;
            lastRightLift = robot.lift.liftUpLeft(lift_value, RAISE_POWER);
            lastLeftLift = robot.lift.liftUpRight(lift_value, RAISE_POWER);
        }


        // ------- printing the slider position -------
        //telemetry.addData("Slider target value", isPressed );
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

        telemetry.update();
    }

}

//test