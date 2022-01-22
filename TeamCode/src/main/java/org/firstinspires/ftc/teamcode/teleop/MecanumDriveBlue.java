
package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BlueTeleOp", group = "TeleOp")
public class MecanumDriveBlue extends OpMode {

    private DcMotorEx right_front;
    private DcMotorEx left_front;
    private DcMotorEx right_back;
    private DcMotorEx left_back;

    private DcMotorEx carousel;

    private DcMotorEx intake_spinner;
    private Servo intake_transfer;
    private DcMotorEx viper;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double encoderConstant = 89.1267682333;
    private double encoderViper = 97.8029862845;

    public boolean xAxisLockLoop = false;
    public boolean yAxisLockLoop = false;

    @Override
    // code to run when driver hits INIT
    public void init() {

        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        intake_spinner = hardwareMap.get(DcMotorEx.class, "intake_spinner");
        intake_transfer = hardwareMap.get(Servo.class, "intake_transfer");
        viper = hardwareMap.get(DcMotorEx.class, "viper");

        right_front.setDirection(DcMotorEx.Direction.REVERSE);
        right_back.setDirection(DcMotorEx.Direction.REVERSE);
        left_front.setDirection(DcMotorEx.Direction.REVERSE);
        left_back.setDirection(DcMotorEx.Direction.FORWARD);

        carousel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intake_spinner.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake_spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        viper.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        left_front.setPower(0);
        left_back.setPower(0);
        right_front.setPower(0);
        right_back.setPower(0);
        carousel.setPower(0);

        intake_transfer.setPosition(0);


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.dpad_right == true) {
            if (yAxisLockLoop == true) {
                telemetry.addData("Error: ", "Please disengage the Y-Axis lock!");
                telemetry.update();
            } else if (yAxisLockLoop == false) {
                xAxisLockLoop = true;
                telemetry.addData("X Axis Lock: ", "Activated!");
            }
        } else if (gamepad1.dpad_left == true && xAxisLockLoop == true) {
            xAxisLockLoop = false;
            telemetry.addData("X Axis Lock: ", "Disabled!");

        }

        if (gamepad1.dpad_up == true) {
            if (xAxisLockLoop == true) {
                telemetry .addData("Error: ", "Please disengage the X-Axis lock!");
                telemetry.update();
            } else if (xAxisLockLoop == false) {
                yAxisLockLoop = true;
                telemetry.addData("Y Axis Lock: ", "Activated!");

            }
        } else if (gamepad1.dpad_down == true && yAxisLockLoop == true) {
            yAxisLockLoop = false;
            telemetry.addData("Y Axis Lock: ", "Disabled!");

        }


        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double intakePower = gamepad2.left_stick_y;


        if(xAxisLockLoop == true) {
            ly = 0;
            rx = 0;
        }

        if(yAxisLockLoop == true) {
            lx = 0;
            rx = 0;
        }

        double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

        /* max power is is 0.7 cuz testing and don't want to go crazy;  change if necessary
        divide by denominator to correct for imperfect strafing */
        double left_front_power = (ly + lx - rx) / denominator;
        double left_back_power = (ly - lx - rx) / denominator;
        double right_front_power = (ly - lx + rx) / denominator;
        double right_back_power = (ly + lx + rx) / denominator;

        left_front.setPower(-1 * (0.15 * left_front_power));
        left_back.setPower(0.15 * left_back_power);
        right_front.setPower(0.15 * right_front_power);
        right_back.setPower(0.15 * right_back_power);

        intake_spinner.setPower(intakePower);


        /*if (viperExtend == true) {
            viper.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            viper.setTargetPosition((int) encoderViper);
            viper.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            viper.setVelocity(12);

            while (viper.isBusy()) {

            }

            viper.setPower(0);
        }*/

        if (gamepad2.right_bumper == true) {
            carousel.setPower(0.23);

        }

        if (gamepad2.right_bumper == false) {
            carousel.setPower(0);
        }

        if (gamepad2.left_bumper == true) {
            carousel.setPower(-0.23);
        }

        if (gamepad2.left_bumper == false) {
            carousel.setPower(0);
        }

        if (gamepad2.a == true) {
            intake_transfer.setPosition(1);

        }

        if (gamepad2.b == true) {
            telemetry.addData("Activated", "no");
            telemetry.update();
            intake_transfer.setPosition(0);
        }

        telemetry.addData("Runtime: ", runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }


}
