package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Robot.*;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;

@TeleOp(name="FTC2025")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private Controller controller1;
    private SampleMecanumDrive drive;

    private  int raise_value, arm_value,lift_value;
    public double RAISE_POWER = 1.0;

    private boolean closed, armIsUp;
    private int gripper_position = 0; //0-oprit 1-aduna piesa 2-beleste piesa
    private boolean sculatoare;
    private int last_arm_position; // 0 - a, 1 - x, 2 - b, 3 - y
    private int slider_level = 0;
    private ScheduledFuture<?> lastArmMove, lastSliderMove;
    private ScheduledFuture<?> lastRightLift, lastLeftLift;
    //boolean isPressed = false;

    int slider_target_positionup =0;
    int slider_target_positiondown =0;
    int arm_target_positionup=0;
    int arm_target_positiondown=0;

    boolean isExtended_down = false;
    boolean isExtended_up =false;
    boolean gripper_rotating = false;
    boolean lb_down =false;

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
        arm_value = 150;
        robot.arm.raiseArm(arm_value, RAISE_POWER - 0.6);
        /*robot.lift.setUpPosition();*/
    }

    // ------ the emergency stop function ---------
//    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();
        //isPressed=false;
        slider_target_positionup=robot.slider.getCurrentPositionSlider();
        slider_target_positiondown=robot.slider.getCurrentPositionSlider();
        arm_target_positionup=robot.arm.getCurrentPositionArm();
        arm_target_positiondown=robot.arm.getCurrentPositionArm();
        if(arm_value < 300){
            isExtended_down = true;
            isExtended_up = false;
        }else{
            isExtended_up = true;
            isExtended_down = false;
        }
        // controller 1
        // - movement
        // - ridcare arm
        // - intake gripper
        // - control slider
        // - lift



        // --------- movement general al robotului ---------
        drive.setWeightedDrivePower(
                new Pose2d(
                        (controller1.left_stick_y) ,
                        (controller1.right_stick_x),
                        (controller1.left_stick_x)// gen astea negative / pozitive sau schimbate intre ele
                )
        );

        // --------- extindere slider ---------
        if (controller1.YOnce()){
            robot.slider.raiseSlider(5700, 16);
        }

        // --------- retractie slider ---------
        if (controller1.AOnce()){
            robot.slider.raiseSlider(0, 16);
        }

        // --------- extindere slider controlat ---------
        if (controller1.B()){
            if (slider_target_positionup <=5600)
                slider_target_positionup =robot.slider.getCurrentPositionSlider()+200;
            robot.slider.raiseSlider(slider_target_positionup, 16);
        }

        // --------- retractie slider controlat ---------
        if (controller1.X()){
            if (slider_target_positiondown >=0)
                slider_target_positiondown =robot.slider.getCurrentPositionSlider()-200;
            robot.slider.raiseSlider(slider_target_positiondown, 16);
        }

        // --------- verificare slider extins in jos ---------
        if(isExtended_down){
            arm_value = 200;
            robot.arm.raiseArm(arm_value, RAISE_POWER + 1);
            isExtended_down = false;

        }

        // --------- verificare slider extins in sus ---------
        if(isExtended_up){
            arm_value = 800;
            robot.arm.raiseArm(arm_value, RAISE_POWER);
            isExtended_up = false;

        }

        // -------  controlling the arm positions -----
        if(!Utils.isDone(lastArmMove) || !Utils.isDone(lastSliderMove)) {
            return ;
        }
        else if (controller1.dpadUpOnce()) {
            if(robot.slider.getCurrentPositionSlider() > 10){
                slider_target_positiondown = 0;
                robot.slider.raiseSlider(slider_target_positiondown, 16);
            }
            arm_value = 750;
            robot.arm.raiseArm(arm_value, RAISE_POWER -0.4);
        } else if (controller1.dpadDownOnce()) {
            if(robot.slider.getCurrentPositionSlider() > 10){
                slider_target_positiondown = 0;
                robot.slider.raiseSlider(slider_target_positiondown, 16);
            }
            arm_value = 200;
            robot.arm.raiseArm(arm_value, RAISE_POWER - 0.6);
        }
        if (controller1.leftBumper()){
            if(robot.slider.getCurrentPositionSlider() > 10){
                slider_target_positiondown = 0;
                robot.slider.raiseSlider(slider_target_positiondown, 10);
            }
            if (arm_target_positionup <=800) {
                arm_target_positionup = robot.arm.getCurrentPositionArm() + 150;
                robot.arm.raiseArm(arm_target_positionup, 16);
            }
        }
        if(controller1.rightBumper()) {
            if(robot.slider.getCurrentPositionSlider() > 10){
                slider_target_positiondown = 0;
                robot.slider.raiseSlider(slider_target_positiondown, 10);
            }
            if (arm_target_positiondown >= 100) {
                arm_target_positiondown = robot.arm.getCurrentPositionArm() - 150;
                robot.arm.raiseArm(arm_target_positiondown,16);
            }
        }



        //----------- gripper ---------------
        double left_trig = controller1.left_trigger;
        double right_trig = controller1.right_trigger;
        if(left_trig > 0) {
            robot.gripper.grab_position();
            gripper_position=1;
        } else if (right_trig > 0) {
            robot.gripper.release_position();
            gripper_position=2;
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
            if (robot.lift.getCurrentPositionServoLeft() > 900 && robot.lift.getCurrentPositionServoRight() > 900) {
                lift_value = 0;
                lastRightLift = robot.lift.liftUpLeft(lift_value, 1);
                lastLeftLift = robot.lift.liftUpRight(lift_value, 1);

                robot.lift.setUpPosition();
            } else {
                lift_value = -1000;
                lastRightLift = robot.lift.liftUpLeft(lift_value, 1);
                lastLeftLift = robot.lift.liftUpRight(lift_value, 1);

                robot.lift.setDownPosition();
            }
        }
        if(!Utils.isDone(lastRightLift) || !Utils.isDone(lastLeftLift)) {
            return ;
        } else if (controller1.dpadRightOnce()) {
            arm_value = 0;
            robot.arm.raiseArm(arm_value, RAISE_POWER - 0.6);
            lift_value = 7000;
            lastRightLift = robot.lift.liftUpLeft(lift_value, 1);
            lastLeftLift = robot.lift.liftUpRight(lift_value, 1);
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