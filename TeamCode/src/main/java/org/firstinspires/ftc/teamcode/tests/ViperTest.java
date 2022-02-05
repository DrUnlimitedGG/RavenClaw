
package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ViperTest",group="Tests")
public class ViperTest extends OpMode {
    private DcMotorEx viper_direct = null;
    private DcMotorEx viper_indirect = null;

    private boolean viperextended = false;
    private double encoderConstant = 89.1267682333;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        viper_direct = hardwareMap.get(DcMotorEx.class, "viper_direct");
        viper_indirect = hardwareMap.get(DcMotorEx.class, "viper_indirect");

        viper_direct.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        boolean viperactive = gamepad2.b;

        if (viperactive == true) {
            viper_extend();

            viperextended = true;
        }

        if (viperactive == false) {
            if (viperextended == true) {
                viper_lower();

                viperextended = false;
            }
        }

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

        viper_direct.setTargetPosition(100);
        viper_indirect.setTargetPosition(100);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        viper_direct.setVelocity(50);
        viper_indirect.setVelocity(50);

        while (viper_direct.isBusy() && viper_indirect.isBusy()) {

        }

        viper_direct.setPower(0);
        viper_indirect.setPower(0);

    }

    public void viper_lower() {
        viper_direct.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        viper_indirect.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        viper_direct.setTargetPosition(-100);
        viper_indirect.setTargetPosition(-100);

        viper_direct.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        viper_indirect.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        viper_direct.setVelocity(50);
        viper_indirect.setVelocity(50);

        while (viper_direct.isBusy() && viper_indirect.isBusy()) {

        }

        viper_direct.setPower(0);
        viper_indirect.setPower(0);

    }

    public void PIDdirect() {

    }

    public void PIDindirect() {

    }

}
