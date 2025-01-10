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

    private  int raise_value, arm_value;
    public double RAISE_POWER = 1.0;
    private boolean closed, gripper_released , armIsUp;
    private boolean sculatoare;
    private int last_arm_position; // 0 - a, 1 - x, 2 - b, 3 - y
    private int slider_level = 0;
    private ScheduledFuture<?> lastArmMove, lastSliderMove;
    private ScheduledFuture<?> lastRightLift, lastLeftLift;
    boolean isPressed = false;
    int slider_target_positionup;
    int slider_target_positiondown;

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
        robot.lift.setDownPosition();
        closed = false;
        sculatoare = false;
        last_arm_position = 0;
        gripper_released = false;

    }

    // ------ the emergency stop function ---------
//    public void stop() { robot.stopRobot(); }

    @Override
    public void loop() {
        controller1.update();
        isPressed=false;
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
                        (-controller1.left_stick_y) , // gen astea negative / pozitive sau schimbate intre ele
                        (-controller1.left_stick_x) ,
                        (-controller1.right_stick_x)
                )
        );

        // --------- ridicare brat ---------
        if (controller1.YOnce()){
            robot.slider.raiseSlider(5500 , 2);
        }
        if (controller1.AOnce()){
            robot.slider.raiseSlider(0, 2);
        }
        if (controller1.B()){
            if (slider_target_positionup <=5500)
                slider_target_positionup =robot.slider.getCurrentPositionSlider()+100;
            robot.slider.raiseSlider(slider_target_positionup, 4);
        }
        if (controller1.X()){
            if (slider_target_positiondown >=0)
                slider_target_positiondown =robot.slider.getCurrentPositionSlider()-100;
            robot.slider.raiseSlider(slider_target_positiondown, 4);
        }

        // --------- (BODO) intake primire ---------
        double rotation_speed1 = controller1.right_trigger - controller1.left_trigger;
        robot.gripper.rotateIntake(rotation_speed1);

        // --------- (BODO) control lift ---------
        if (controller1.leftBumper()) {
            robot.lift.setDownPosition();
            gripper_released = true;
        } else if (controller1.rightBumper()) {
            robot.lift.setUpPosition();
            gripper_released = false;
        }

        if(!Utils.isDone(lastRightLift) || !Utils.isDone(lastLeftLift)) {
            return ;
        } //else if (controller1.YOnce()) {
            //arm_value = 3500;
           // lastRightLift = robot.lift.liftUpLeft(arm_value, 1);
           // lastLeftLift = robot.lift.liftUpRight(arm_value, 1);
        //}



        // ------- (BELE) controlling the arm positions -----
        if(!Utils.isDone(lastArmMove) || !Utils.isDone(lastSliderMove)) {
            return ;
        }

        //else if (controller1.dpadUpOnce()) {
          //  arm_value = 910;//trebe testat

            /*if (last_arm_position == 0) {
                robot.arm.gripperSafety();
                robot.gripper.closeBarier();
            }*/

            //armIsUp = true;
            //robot.arm.raiseArm(arm_value, RAISE_POWER);
            //last_arm_position = 3;
        //} else if (controller1.BOnce()) {
        //    arm_value = 750;

           /* if (last_arm_position == 0) {
                robot.arm.gripperSafety();
                robot.gripper.closeBarier();
            }

            armIsUp = false;
            robot.arm.gripperSafety();
            robot.gripper.closeBarier();

            robot.arm.raiseArm(arm_value, RAISE_POWER);
            last_arm_position = 2;*/
        //} else if (controller1.XOnce()) {
          //  arm_value = 250;

            //if (last_arm_position == 0) {
//                robot.arm.gripperSafety();
//                robot.gripper.closeBarier();
            //}

           // armIsUp = false;
            //robot.arm.gripperSafety();


            //robot.arm.raiseArm(arm_value, RAISE_POWER);
            //last_arm_position = 1;
        //} else if (controller1.AOnce()) {
          //  arm_value = 0;

            //robot.arm.raiseArm(arm_value, RAISE_POWER);
         //   last_arm_position = 0;

            //robot.arm.gripperAfterArm();

        //}


        // --------- (BELE) intake iesire ---------
//        double rotation_speed2 = controller2.right_trigger - controller2.left_trigger;
//        robot.gripper.rotateIntake(rotation_speed2);

        // ------- (BELE) basculare cutie intake -------
        if (controller1.dpadLeftOnce()) {
            if(gripper_released == true) {
               // robot.arm.gripperInitialPos();
            } else {
                //robot.arm.gripperReleasePos();
            }
            gripper_released = !gripper_released;
        }

        // ------- (BELE) controlare bariera -------
        //if (controller1.dpadRightOnce()) {
        //    if (closed == true) {

        //    } else {

        //    }
        //    closed = !closed;
        //}
        // ------- controlling the slider positions -----
        if (last_arm_position != 0) {
            if (controller1.dpadUpOnce()) { //again se suprapune
                if (slider_level < 5) {
                    slider_level = slider_level + 1;
                    if (slider_level == 1) {
                        slider_level = slider_level + 1;
                    }
                }

                raise_value = 600 * slider_level;
                robot.slider.raiseSlider(raise_value, RAISE_POWER);
            } else if (controller1.dpadDownOnce()) {
                slider_level = 0;
                raise_value = 600 * slider_level;
                robot.slider.raiseSlider(raise_value, RAISE_POWER);
            }
        }

        // emergency stop button
        //if (controller2.startButtonOnce()) {          //nu cred ca ne trebe
          //  stop();
        //}

        // ------- printing the slider position -------
        telemetry.addData("Slider target value", isPressed );
        telemetry.addData("Slider position", robot.slider.getCurrentPositionSlider());
        telemetry.addLine("---------------------");
        telemetry.addData("Arm target value", arm_value);
       // telemetry.addData("Arm position", robot.arm.getCurrentPositionArm());
        telemetry.addLine("---------------------");
        telemetry.addData("Lift target value", arm_value);
        telemetry.addData("lift position", robot.lift.getCurrentPositionArm());

        telemetry.update();
    }

}