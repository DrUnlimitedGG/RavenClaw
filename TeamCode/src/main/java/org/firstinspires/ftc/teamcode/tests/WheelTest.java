package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="Wheel Test", group="Testing Stuff")
public class WheelTest extends OpMode
{
    // Declare OpMode members.

    private DcMotorEx RF = null;
    private DcMotorEx RB = null;
    private DcMotorEx LF = null;
    private DcMotorEx LB = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        RF = hardwareMap.get(DcMotorEx.class, "right_front");
        LF = hardwareMap.get(DcMotorEx.class, "left_front");
        RB = hardwareMap.get(DcMotorEx.class, "right_back");
        LB = hardwareMap.get(DcMotorEx.class, "left_back");

        RF.setDirection(DcMotorEx.Direction.REVERSE);
        RB.setDirection(DcMotorEx.Direction.REVERSE);
        LF.setDirection(DcMotorEx.Direction.FORWARD);
        LB.setDirection(DcMotorEx.Direction.FORWARD);

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

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double lx = gamepad1.left_stick_x;
        double ly = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(ly) + Math.abs(lx) + Math.abs(rx), 1);

        /* max power is is 0.7 cuz testing and don't want to go crazy;  change if necessary
        divide by denominator to correct for imperfect strafing */
        double left_front_power = (ly + lx - rx) / denominator;
        double left_back_power = (ly - lx - rx) / denominator;
        double right_front_power = (ly - lx + rx) / denominator;
        double right_back_power = (ly + lx + rx) / denominator;

        LF.setPower(-1 * (1 * left_front_power));
        LB.setPower(-1 * (1 * left_back_power));
        RF.setPower(1 * right_front_power);
        RB.setPower(1 * right_back_power);

        telemetry.addData("Left Front ", LF.getPower());
        telemetry.addData("Left Back ", LB.getPower());
        telemetry.addData("Right Front ", RF.getPower());
        telemetry.addData("Right Back ", RB.getPower());

        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
