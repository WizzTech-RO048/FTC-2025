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
    private boolean gripper_positioned, gripper_released;
    private boolean sculatoare;
    private int last_arm_position; // 0 - a, 1 - x, 2 - b, 3 - y
    private int slider_level = 0;
    private ScheduledFuture<?> lastArmMove, lastSliderMove;
    private ScheduledFuture<?> lastRightLift, lastLeftLift;
    //boolean isPressed = false;

    int slider_target_positionup;
    int slider_target_positiondown;

    boolean isExtended_down = false;
    boolean isExtended_up =false;

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
        gripper_released = false;
        gripper_positioned = true;
        arm_value = 200;
        robot.arm.raiseArm(arm_value, RAISE_POWER - 0.6);
        robot.lift.setUpPosition();
    }

    // ------ the emergency stop function ---------
//    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();
        //isPressed=false;
        slider_target_positionup=robot.slider.getCurrentPositionSlider();
        slider_target_positiondown=robot.slider.getCurrentPositionSlider();

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
            robot.slider.raiseSlider(5500 , 16);
            if(arm_value <300) {
                isExtended_down = true;
                isExtended_up = false;
            }else{
                isExtended_up =true;
                isExtended_down = false;
            }
        }

        // --------- retractie slider ---------
        if (controller1.AOnce()){
            robot.slider.raiseSlider(0, 16);
            if(arm_value <300) {
                isExtended_down = true;
                isExtended_up = false;
            }else{
                isExtended_up =true;
                isExtended_down = false;
            }
        }

        // --------- extindere slider controlat ---------
        if (controller1.B()){
            if (slider_target_positionup <=5500)
                slider_target_positionup =robot.slider.getCurrentPositionSlider()+400;
            robot.slider.raiseSlider(slider_target_positionup, 16);
            if(arm_value < 300){
                isExtended_down = true;
                isExtended_up = false;
            }else{
                isExtended_up = true;
                isExtended_down = false;
            }
        }

        // --------- retractie slider controlat ---------
        if (controller1.X()){
            if (slider_target_positiondown >=0)
                slider_target_positiondown =robot.slider.getCurrentPositionSlider()-400;
            robot.slider.raiseSlider(slider_target_positiondown, 16);
            robot.lift.setUpPosition();
        }

        // --------- verificare slider extins in jos ---------
        if(isExtended_down){
            arm_value = 200;
            robot.arm.raiseArm(arm_value, RAISE_POWER + 1);
            isExtended_down = false;

        }

        // --------- verificare slider extins in sus ---------
        if(isExtended_up){
            arm_value = 600;
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
            arm_value = 600;
            robot.arm.raiseArm(arm_value, RAISE_POWER -0.4);
        } else if (controller1.dpadDownOnce()) {
            if(robot.slider.getCurrentPositionSlider() > 10){
                slider_target_positiondown = 0;
                robot.slider.raiseSlider(slider_target_positiondown, 16);
            }
            arm_value = 200;
            robot.arm.raiseArm(arm_value, RAISE_POWER - 0.6);
        }


        //----------- gripper ---------------
        if(controller1.dpadLeftOnce()) {
            robot.gripper.grab_position();
            gripper_positioned=!gripper_positioned;
        }
        if(controller1.dpadRightOnce()) {
            robot.gripper.release_position();
            gripper_positioned=!gripper_positioned;
        }


        if(controller1.leftBumperOnce())
        {
            if(gripper_released)
            {
                robot.gripper.grab();
                gripper_released=!gripper_released;
            }
            if (gripper_released == false)
            {
                robot.gripper.release();
                gripper_released=!gripper_released;
            }
        }
        //if(controller1.rightBumperOnce())
        //{

        //}
        // ---------- controale lift -------------
        double left_trig = controller1.left_trigger;
        double right_trig = controller1.right_trigger;
        if (left_trig > 0) {

            lift_value = 0;
            lastRightLift = robot.lift.liftUpLeft(lift_value, 1);
            lastLeftLift = robot.lift.liftUpRight(lift_value, 1);

            robot.lift.setDownPosition();
        }
        else if (right_trig > 0)
        {

            lift_value = 1000;
            lastRightLift = robot.lift.liftUpLeft(lift_value, 1);
            lastLeftLift = robot.lift.liftUpRight(lift_value, 1);

            robot.lift.setUpPosition();
        }
        if(!Utils.isDone(lastRightLift) || !Utils.isDone(lastLeftLift)) {
            return ;
        } else if (controller1.rightBumper()) {
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