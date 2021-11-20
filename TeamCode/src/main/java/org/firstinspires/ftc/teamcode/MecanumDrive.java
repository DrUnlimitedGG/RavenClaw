

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;





@TeleOp(name = "MecanumDrive")
public class MecanumDrive extends OpMode {

    public DcMotorEx RF = hardwareMap.get(DcMotorEx.class, "right_front");
    public DcMotorEx LF = hardwareMap.get(DcMotorEx.class, "left_front");
    public DcMotorEx RB = hardwareMap.get(DcMotorEx.class, "right_back");
    public DcMotorEx LB = hardwareMap.get(DcMotorEx.class, "left_back");

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");



        RF.setDirection(DcMotorEx.Direction.REVERSE);
        RB.setDirection(DcMotorEx.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        LF.setPower(ly + lx + rx);
        LB.setPower(ly - lx + rx);
        RF.setPower(ly - lx - rx);
        RB.setPower(ly + lx - rx);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

    }
}



