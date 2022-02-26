
package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BlueTeleOp", group = "TeleOp")
public class MecanumDriveBlue extends OpMode {
    FtcDashboard ftcdashboard;

    private boolean viperextended = false;

    private double directPower = 0;

    private DcMotorEx right_front;
    private DcMotorEx left_front;
    private DcMotorEx right_back;
    private DcMotorEx left_back;
    private DcMotorEx viper_direct = null;
    private DcMotorEx viper_indirect = null;
    private DcMotorEx carousel;
    private DcMotorEx intake_spinner;
    private DigitalChannel intake_touch;
    private Servo intake_transfer;

    private int servoAmount = 0;

    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    private double encoderConstant = 89.1267682333;
    private double encoderViper = 294;

    public boolean xAxisLockLoop = false;
    public boolean yAxisLockLoop = false;

    private final double speed = 1500;

    public static PIDCoefficients pidCoeffsDir = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsDir = new PIDCoefficients(0, 0, 0);
    ElapsedTime PIDTimerDir = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public static PIDCoefficients pidCoeffsIndir = new PIDCoefficients(0, 0, 0);
    public PIDCoefficients pidGainsIndir = new PIDCoefficients(0, 0, 0);
    ElapsedTime PIDTimerIndir = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    @Override
    // code to run when driver hits INIT
    public void init() {

        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");
        intake_touch = hardwareMap.get(DigitalChannel.class, "intake_touch");

        intake_spinner = hardwareMap.get(DcMotorEx.class, "intake_spinner");
        intake_transfer = hardwareMap.get(Servo.class, "intake_transfer");
        viper_direct = hardwareMap.get(DcMotorEx.class, "viper_direct");
        viper_indirect = hardwareMap.get(DcMotorEx.class, "viper_indirect");

        viper_direct.setDirection(DcMotorEx.Direction.REVERSE);
        right_front.setDirection(DcMotorEx.Direction.REVERSE);
        right_back.setDirection(DcMotorEx.Direction.REVERSE);
        left_front.setDirection(DcMotorEx.Direction.FORWARD);
        left_back.setDirection(DcMotorEx.Direction.FORWARD);
        viper_indirect.setDirection(DcMotorEx.Direction.FORWARD);

        carousel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        intake_spinner.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake_spinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

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

        //intake_transfer.setPosition(0);


    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        boolean viperExtend = gamepad2.dpad_up;
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
                telemetry.addData("Error: ", "Please disengage the X-Axis lock!");
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
        double ly = gamepad1.left_stick_y;
        double rx = -gamepad1.right_stick_x;

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

        left_front.setPower(0.5 * left_front_power);
        left_back.setPower(0.5 * left_back_power);
        right_front.setPower(0.5 * right_front_power);
        right_back.setPower(0.5 * right_back_power);

        intake_spinner.setPower(0.55 * intakePower);

        if (gamepad2.dpad_up == true) {
            viper_extend();
            //viperextended = true;
        }

        if (gamepad2.dpad_down == true) {
            telemetry.addData("Vipers: ", "Retracting!");
            telemetry.update();
            viper_lower();
            //viperextended = false;
        }

        if (gamepad2.right_bumper == true) {
            carousel.setPower(0.6);
        }

        if (gamepad2.right_bumper == false) {
            carousel.setPower(0);
        }

        if (gamepad2.left_bumper == true) {
            carousel.setPower(-0.6);
        }

        if (gamepad2.left_bumper == false) {
            carousel.setPower(0);
        }

        if (gamepad2.a == true) {
            intake_transfer.setPosition(1);
            }

        if (gamepad2.b == true) {
            intake_transfer.setPosition(0);
        }

        /*if (intakePower > 0) {
            if (intake_touch.getState() == true) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
           `     intake_spinner.setPower(0);
            } else if (intake_touch.getState() == false) {
                intake_spinner.setPower(intakePower);
            }

        }*/

        telemetry.addData("Runtime: ", runtime.toString());
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }

    public void viper_extend() {
        viper_direct.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        viper_direct.setTargetPosition(1300);
        viper_indirect.setTargetPosition(1300);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        viper_direct.setVelocity(speed);
        viper_indirect.setVelocity(speed);

        while (viper_direct.isBusy() && viper_indirect.isBusy()) {
            PIDdirect(speed);
            directPower = viper_direct.getVelocity();
            PIDindirect(directPower);

            telemetry.addData("Direct: ", viper_direct.getVelocity());
            telemetry.addData("Indirect: ", viper_indirect.getVelocity());
            telemetry.update();

        }

        viper_direct.setPower(0);
        viper_indirect.setPower(0);

    }

    public void viper_lower() {
        /*viper_direct.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);*/

        viper_direct.setTargetPosition(-10);
        viper_indirect.setTargetPosition(-10);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        viper_direct.setVelocity(-speed);
        viper_indirect.setVelocity(-speed);
        telemetry.addData("Function: ", "Retracting ran!");
        telemetry.update();

        while (viper_direct.isBusy() && viper_indirect.isBusy()) {
            PIDdirect(-speed);
            directPower = viper_direct.getVelocity();
            PIDindirect(directPower);
        }

        viper_direct.setPower(0);
        viper_indirect.setPower(0);

    }

    double integralDir = 0;
    double lastErrorDir = 0;
    public void PIDdirect(double targetVelocity) {
        PIDTimerDir.reset();

        double currentVelocity = viper_direct.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralDir += error * PIDTimerDir.time();

        double deltaError = error - lastErrorDir;
        double derivative = deltaError / PIDTimerDir.time();

        pidGainsDir.p = pidCoeffsDir.p * error;
        pidGainsDir.i = pidCoeffsDir.i * integralDir;
        pidGainsDir.d = pidCoeffsDir.d * derivative;

        viper_direct.setVelocity(targetVelocity + pidGainsDir.p + pidGainsDir.i + pidGainsDir.d);

        lastErrorDir = error;
    }

    double integralIndir = 0;
    double lastErrorIndir = 0;
    public void PIDindirect(double targetVelocity) {
        PIDTimerIndir.reset();

        double currentVelocity = viper_indirect.getVelocity();

        double error = targetVelocity - currentVelocity;

        integralDir += error * PIDTimerIndir.time();

        double deltaError = error - lastErrorDir;
        double derivative = deltaError / PIDTimerIndir.time();

        pidGainsIndir.p = pidCoeffsIndir.p * error;
        pidGainsIndir.i = pidCoeffsIndir.i * integralDir;
        pidGainsIndir.d = pidCoeffsIndir.d * derivative;

        viper_indirect.setVelocity(targetVelocity + pidGainsIndir.p + pidGainsIndir.i + pidGainsIndir.d);

        lastErrorDir = error;
    }


}
