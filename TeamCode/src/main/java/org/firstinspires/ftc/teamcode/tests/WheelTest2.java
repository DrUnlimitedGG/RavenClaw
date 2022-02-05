package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="WheelTest2", group="Tests")
public class WheelTest2 extends OpMode
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
        if (gamepad1.a == true) {
            LF.setPower(1);
            LB.setPower(1);
            RF.setPower(1);
            RB.setPower(1);

        }

        if (gamepad1.a == false) {
            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            LB.setPower(0);
        }

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
