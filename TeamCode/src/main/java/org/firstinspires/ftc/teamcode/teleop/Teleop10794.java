package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.libs.MotorFunctions;

import org.firstinspires.ftc.teamcode.libs.Robot;

import java.text.SimpleDateFormat;
import java.util.Date;

@TeleOp(name="10794 Teleop", group="Teleop")  // @Autonomous(...) is the other common choice
//@Disabled
public class Teleop10794 extends OpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private Robot robot;
    private MotorFunctions motorFunctions;

    private boolean servoStateLeftArm = false, servoStateRightArm = false,
            servoStateLeftWheel = false, servoStateRightWheel = false,
            servoStateShoulder = false, servoStateElbow = false;
    private double servoPositionFlyAngle, servoPositionPlaid = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        /**
         * Initializes the library functions
         * Robot hardware and motor functions
         */
        robot = new Robot(hardwareMap);
        motorFunctions = new MotorFunctions(-1, 1, 0, 1, .05);

        //servo wheels are flipped in configuration file
        robot.servoLeftWheel.setPosition(.45);
        robot.servoRightWheel.setPosition(.25);

        robot.servoLeftArm.setPosition(0);
        robot.servoRightArm.setPosition(0.7);

        robot.servoFlyAngle.setPosition(1);

        robot.servoElbow.setPosition(0.95);
        robot.servoShoulder.setPosition(0.1);

        robot.servoFeed.setPosition(.498);

        robot.servoPlaid.setPosition(.85);
        telemetry.addData("Servos: ", "Initialized");
    }

    /*
    * Code to run REPEATEDLY when the driver hits INIT
    * We don't need this method
    */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        robot.servoFlyAngle.setPosition(.5);

        robot.servoLeftWheel.setPosition(.5);
        robot.servoRightWheel.setPosition(.25);

        robot.servoFeed.setPosition(.498);
        robot.servoPlaid.setPosition(.85);

        runtime.reset();
    }

    /*
     * Assigns the gamepad buttons to the motor and servos
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        /**
         * Sets the power level of the drive motors to the joystick values
         */
        robot.motorDriveLeft.setPower(MotorFunctions.dcMotor(gamepad1.left_stick_y));
        robot.motorDriveRight.setPower(MotorFunctions.dcMotor(gamepad1.right_stick_y));

        /**
         * Gamepad 1
         */
        if (gamepad1.a) {
            //not in use
        }
        if (gamepad1.b) {
            //not in use
        }
        if (gamepad1.x) {
            //not in use
        }
        if (gamepad1.y) {
            if (servoStateRightArm) {
                robot.servoRightArm.setPosition(0);
            } else {
                robot.servoRightArm.setPosition(.7);
            }
            servoStateRightArm = !servoStateRightArm;
        }
        if (gamepad1.dpad_up) {
            if (servoStateLeftArm) {
                robot.servoLeftArm.setPosition(0);
            } else {
                robot.servoLeftArm.setPosition(.72);
            }
            servoStateLeftArm = !servoStateLeftArm;
        }
        if (gamepad1.dpad_down) {
            //not in use
        }
        if (gamepad1.dpad_left) {
            //not in use
        }
        if (gamepad1.dpad_right) {
            //not in use
        }
        if (gamepad1.left_bumper) {
            if(servoStateLeftWheel) {
                robot.servoLeftWheel.setPosition(.45);
            } else {
                robot.servoLeftWheel.setPosition(.60);
            }
            servoStateLeftWheel = !servoStateLeftWheel;
        }
        if (gamepad1.right_bumper) {
            if(servoStateRightWheel) {
                robot.servoRightWheel.setPosition(.25);
            } else {
                robot.servoRightWheel.setPosition(.3);
            }
            servoStateRightWheel = !servoStateRightWheel;
        }
        if (gamepad1.left_trigger > 0.25) {
            if (servoStateShoulder) {
                robot.servoShoulder.setPosition(0.1);
            } else {
                robot.servoShoulder.setPosition(0.8);
            }
            servoStateShoulder = !servoStateShoulder;
        }
        if (gamepad1.right_trigger > 0.25) {
            if (servoStateElbow) {
                robot.servoElbow.setPosition(0.95);
            } else {
                robot.servoElbow.setPosition(0.05);
            }
            servoStateElbow = !servoStateElbow;
        }

        /**
         * Gamepad 2
         */
        robot.motorLift.setPower(MotorFunctions.dcMotor(gamepad2.right_stick_y));

        if (gamepad2.a) {
            servoPositionPlaid = MotorFunctions.servoDecrement(servoPositionPlaid);
            servoPositionPlaid = Range.clip(servoPositionPlaid, 0, .9);
        }
        if (gamepad2.y) {
            servoPositionPlaid = MotorFunctions.servoIncrement(servoPositionPlaid);
            servoPositionPlaid = Range.clip(servoPositionPlaid, 0, .9);
        }
        if (gamepad2.x) {
            //Automatic fire -- separate targeting class
            robot.servoFeed.setPosition(.52);
        }
        if (gamepad2.b) {
            robot.motorIntakeElevator.setPower(0);
        }
        if (gamepad2.dpad_up) {
            robot.motorIntakeElevator.setPower(.85);
        }
        if (gamepad2.dpad_down) {
            robot.motorIntakeElevator.setPower(-.85);
        }
        if (gamepad2.dpad_left) {
            robot.servoFeed.setPosition(0);
        }
        if (gamepad2.dpad_right) {
            robot.servoFeed.setPosition(1);
        }
        if (gamepad2.left_bumper) {
            servoPositionFlyAngle = MotorFunctions.servoDecrement(servoPositionFlyAngle);
            servoPositionFlyAngle = Range.clip(servoPositionFlyAngle, 0.3, 1);
        }
        if (gamepad2.right_bumper) {
            servoPositionFlyAngle = MotorFunctions.servoIncrement(servoPositionFlyAngle);
            servoPositionFlyAngle = Range.clip(servoPositionFlyAngle, 0.3, 1);
        }
        if (gamepad2.left_trigger > 0.1) {
            robot.motorFlyLeft.setPower(0);
            robot.motorFlyRight.setPower(0);
            robot.servoFeed.setPosition(1);
        }
        if (gamepad2.right_trigger > 0.1) {
            robot.motorFlyRight.setPower(1);
            robot.motorFlyLeft.setPower(.9);
            robot.servoFeed.setPosition(0);
        }
        if (gamepad2.left_stick_y > 0.25) {
            servoPositionFlyAngle = MotorFunctions.servo(servoPositionFlyAngle, .01, 0.3, 1);
        }

        /**
         * Set robot servos to binary values -- not currently in use
         */
//        robot.setArms(servoStateArms ? MotorFunctions.servo(1) : MotorFunctions.servo(0));
//        robot.servoFlip.setPosition(servoStateFlip ? MotorFunctions.servo(1) : MotorFunctions.servo(0));

        /**
         * Sets the position for the manual fly wheel
         */
        robot.servoFlyAngle.setPosition(servoPositionFlyAngle);
        robot.servoPlaid.setPosition(servoPositionPlaid);
        sleep(50);
    }

    /**
     * Code to run ONCE after STOP is pressed
     * DO NOT USE THIS METHOD!!
     * Violation of game rules
     */
    @Override
    public void stop() {
        robot.servoFeed.setPosition(.498);
    }

    /**
     * Sleeps for the given amount of milliseconds, or until the thread is interrupted. This is
     * simple shorthand for the operating-system-provided {@link Thread#sleep(long) sleep()} method.
     *
     * @param milliseconds amount of time to sleep, in milliseconds
     * @see Thread#sleep(long)
     */
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
